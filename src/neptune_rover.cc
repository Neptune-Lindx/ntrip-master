// neptune_rover.cc - Ubuntu-B (rover): NtripClient -> write RTCM to /dev/ttyACM0
// RTCM downlink only; no GGA uplink.
// Optional: save raw bytes read FROM serial (board output) to .rtcm3 file.
//
// Usage:
//   ./neptune_rover --caster-ip 192.168.2.106 [--device /dev/ttyACM0] [--baud 921600]
//   [--save-obs] [--obs-file /path/to/out.rtcm3]
//   [--obs-dir /path/to/dir] [--obs-name basename.rtcm3]

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "ntrip/ntrip_client.h"

namespace {

std::atomic_bool g_running{true};
int g_serial_fd = -1;
std::atomic<uint64_t> g_bytes_received{0};

bool g_save_obs = false;
FILE* g_obs_fp = nullptr;
std::thread g_serial_rx_thread;
std::atomic<uint64_t> g_obs_bytes_saved{0};

void OnSignal(int) { g_running.store(false); }

int OpenSerial(const char* dev, int baud) {
  int fd = open(dev, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror("open serial");
    return -1;
  }
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0) {
    perror("tcgetattr");
    close(fd);
    return -1;
  }
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  speed_t spd = B115200;
  switch (baud) {
    case 9600: spd = B9600; break;
    case 38400: spd = B38400; break;
    case 460800: spd = B460800; break;
    case 921600: spd = B921600; break;
    default: spd = B115200; break;
  }
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("tcsetattr");
    close(fd);
    return -1;
  }
  return fd;
}

void SetSerialNonBlocking(int fd) {
  int flags = fcntl(fd, F_GETFL);
  if (flags >= 0) {
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
}

std::string EnsureObsExtension(std::string path) {
  constexpr char kSaveExt[] = ".rtcm3";
  constexpr size_t kSaveExtLen = 6;
  if (path.size() >= kSaveExtLen &&
      path.compare(path.size() - kSaveExtLen, kSaveExtLen, kSaveExt) == 0) {
    return path;
  }
  return path + kSaveExt;
}

std::string DefaultObsFileName(void) {
  char buf[256];
  time_t now = time(nullptr);
  struct tm tm_now;
  localtime_r(&now, &tm_now);
  snprintf(buf, sizeof(buf), "rover_serial_%04d%02d%02d_%02d%02d%02d.rtcm3",
           tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
           tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
  return std::string(buf);
}

std::string BuildObsPath(const char* obs_file, const char* obs_dir,
                         const char* obs_name) {
  if (obs_file != nullptr && obs_file[0] != '\0') {
    return EnsureObsExtension(obs_file);
  }
  if (obs_name != nullptr && obs_name[0] != '\0') {
    std::string name = EnsureObsExtension(obs_name);
    if (obs_dir != nullptr && obs_dir[0] != '\0') {
      std::string dir = obs_dir;
      if (dir.back() != '/') {
        dir.push_back('/');
      }
      return dir + name;
    }
    return name;
  }
  return DefaultObsFileName();
}

bool StartObsRecording(const std::string& path) {
  g_obs_fp = fopen(path.c_str(), "wb");
  if (g_obs_fp == nullptr) {
    perror("[rover] fopen obs file");
    return false;
  }
  g_obs_bytes_saved.store(0);
  printf("[rover] saving serial RX from device to %s\n", path.c_str());
  SetSerialNonBlocking(g_serial_fd);
  g_serial_rx_thread = std::thread([]() {
    std::vector<char> buf(8192);
    while (g_running.load()) {
      if (g_serial_fd < 0 || g_obs_fp == nullptr) {
        break;
      }
      ssize_t n = read(g_serial_fd, buf.data(), buf.size());
      if (n > 0) {
        size_t w = fwrite(buf.data(), 1, static_cast<size_t>(n), g_obs_fp);
        if (w != static_cast<size_t>(n)) {
          fprintf(stderr, "[rover] obs fwrite short write\n");
          g_running.store(false);
          break;
        }
        fflush(g_obs_fp);
        g_obs_bytes_saved.fetch_add(static_cast<uint64_t>(n));
      } else if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        perror("[rover] serial read for obs");
        break;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  });
  return true;
}

void StopObsRecording(void) {
  if (g_serial_rx_thread.joinable()) {
    g_serial_rx_thread.join();
  }
  if (g_obs_fp != nullptr) {
    fclose(g_obs_fp);
    g_obs_fp = nullptr;
    printf("[rover] obs saved %lu bytes from serial RX\n",
           static_cast<unsigned long>(g_obs_bytes_saved.load()));
  }
}

void PrintRtcmHex(const uint8_t* data, int len) {
  if (!data || len <= 0) return;
  std::cout << "RTCM:";
  std::cout << std::hex << std::uppercase << std::setfill('0');
  for (int i = 0; i < len; ++i) {
    std::cout << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  std::cout << std::dec << std::endl;
}

void OnRtcmReceived(const char* buffer, int size) {
  if (!buffer || size <= 0) return;

  std::cout << "rtcm receive ok" << std::endl;
  g_bytes_received.fetch_add(static_cast<uint64_t>(size));

  if (g_serial_fd < 0) return;

  PrintRtcmHex(reinterpret_cast<const uint8_t*>(buffer), size);

  const char* p = buffer;
  int left = size;
  while (left > 0) {
    ssize_t w = write(g_serial_fd, p, static_cast<size_t>(left));
    if (w < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      perror("serial write");
      g_running.store(false);
      return;
    }
    p += w;
    left -= static_cast<int>(w);
  }
}

void PrintUsage(const char* prog) {
  fprintf(stderr,
          "Usage: %s --caster-ip <base_station_LAN_IP>\n"
          "  [--device /dev/ttyACM0] [--baud 115200] [--port 8090]\n"
          "  [--save-obs]  save raw data read FROM serial port to .rtcm3\n"
          "  [--obs-file /path/to/file.rtcm3]  output file (overrides dir/name)\n"
          "  [--obs-dir /path/to/dir] [--obs-name name.rtcm3]  output path parts\n",
          prog);
}

}  // namespace

int main(int argc, char* argv[]) {
  const char* serial_dev = "/dev/ttyACM0";
  const char* caster_ip = nullptr;
  int baud = 115200;
  int caster_port = 8090;
  const char* obs_file = nullptr;
  const char* obs_dir = nullptr;
  const char* obs_name = nullptr;

  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--device") && i + 1 < argc) {
      serial_dev = argv[++i];
    } else if (!strcmp(argv[i], "--baud") && i + 1 < argc) {
      baud = atoi(argv[++i]);
    } else if (!strcmp(argv[i], "--caster-ip") && i + 1 < argc) {
      caster_ip = argv[++i];
    } else if (!strcmp(argv[i], "--port") && i + 1 < argc) {
      caster_port = atoi(argv[++i]);
    } else if (!strcmp(argv[i], "--save-obs")) {
      g_save_obs = true;
    } else if (!strcmp(argv[i], "--obs-file") && i + 1 < argc) {
      obs_file = argv[++i];
      g_save_obs = true;
    } else if (!strcmp(argv[i], "--obs-dir") && i + 1 < argc) {
      obs_dir = argv[++i];
    } else if (!strcmp(argv[i], "--obs-name") && i + 1 < argc) {
      obs_name = argv[++i];
    } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
      PrintUsage(argv[0]);
      return 0;
    } else {
      fprintf(stderr, "[rover] unknown option: %s\n", argv[i]);
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (caster_ip == nullptr) {
    PrintUsage(argv[0]);
    return 1;
  }

  signal(SIGINT, OnSignal);
  signal(SIGTERM, OnSignal);

  g_serial_fd = OpenSerial(serial_dev, baud);
  if (g_serial_fd < 0) return 1;
  printf("[rover] serial %s @ %d baud opened\n", serial_dev, baud);

  if (g_save_obs) {
    std::string obs_path = BuildObsPath(obs_file, obs_dir, obs_name);
    if (!StartObsRecording(obs_path)) {
      close(g_serial_fd);
      g_serial_fd = -1;
      return 1;
    }
  }

  libntrip::NtripClient client;
  client.Init(caster_ip, caster_port, "test01", "123456", "RTCM32");
  client.OnReceived(OnRtcmReceived);

  while (g_running.load()) {
    if (!client.service_is_running()) {
      client.Stop();
      printf("[rover] NtripClient disconnected, reconnecting to %s:%d...\n",
             caster_ip, caster_port);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (!client.Run()) {
        fprintf(stderr, "[rover] reconnect failed, retry in 2s\n");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      printf("[rover] connected to caster %s:%d (RTCM only, no GGA)\n",
             caster_ip, caster_port);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  client.Stop();
  StopObsRecording();
  close(g_serial_fd);
  g_serial_fd = -1;
  printf("[rover] stopped, total NTRIP RTCM bytes %lu\n",
         static_cast<unsigned long>(g_bytes_received.load()));
  return 0;
}
