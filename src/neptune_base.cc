// neptune_base.cc - Ubuntu-A (base): NtripCaster + serial RTCM read + NtripServer
//
// Topology:
//   base board -> /dev/ttyACM0 -> read RTCM -> NtripServer -> local NtripCaster:8090
//   rover connects to this host's LAN IP on port 8090.
//
// Optional: save raw bytes read FROM serial (board output) to .rtcm3 file.
//   [--save-obs] [--obs-file path] [--obs-dir dir] [--obs-name name]

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
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "ntrip/ntrip_caster.h"
#include "ntrip/ntrip_server.h"

namespace {

std::atomic_bool g_running{true};
bool g_save_obs = false;
FILE* g_obs_fp = nullptr;
uint64_t g_obs_bytes_saved = 0;

void OnSignal(int) { g_running.store(false); }

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
  snprintf(buf, sizeof(buf), "base_serial_%04d%02d%02d_%02d%02d%02d.rtcm3",
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

bool OpenObsFile(const std::string& path) {
  g_obs_fp = fopen(path.c_str(), "wb");
  if (g_obs_fp == nullptr) {
    perror("[base] fopen obs file");
    return false;
  }
  g_obs_bytes_saved = 0;
  printf("[base] saving serial RX from device to %s\n", path.c_str());
  return true;
}

void CloseObsFile(void) {
  if (g_obs_fp != nullptr) {
    fclose(g_obs_fp);
    g_obs_fp = nullptr;
    printf("[base] obs saved %lu bytes from serial RX\n",
           static_cast<unsigned long>(g_obs_bytes_saved));
  }
}

bool SaveSerialRxToObs(const void* data, size_t len) {
  if (!g_save_obs || g_obs_fp == nullptr || !data || len == 0) {
    return true;
  }
  size_t w = fwrite(data, 1, len, g_obs_fp);
  if (w != len) {
    fprintf(stderr, "[base] obs fwrite short write\n");
    return false;
  }
  fflush(g_obs_fp);
  g_obs_bytes_saved += len;
  return true;
}

void PrintUsage(const char* prog) {
  fprintf(stderr,
          "Usage: %s [--device /dev/ttyACM0] [--baud 115200] [--port 8090]\n"
          "  [--save-obs]  save raw data read FROM serial port to .rtcm3\n"
          "  [--obs-file /path/to/file.rtcm3]  output file (overrides dir/name)\n"
          "  [--obs-dir /path/to/dir] [--obs-name name.rtcm3]  output path parts\n",
          prog);
}

void PrintRtcmRawHex(const uint8_t* data, int len) {
  if (!data || len <= 0) return;
  std::cout << "RTCM:";
  std::cout << std::hex << std::uppercase << std::setfill('0');
  for (int i = 0; i < len; ++i) {
    std::cout << std::setw(2) << static_cast<unsigned>(data[i]);
  }
  std::cout << std::dec << std::endl;
}

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
  tty.c_cc[VTIME] = 10;
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

class Rtcm3Extractor {
 public:
  void Feed(const uint8_t* data, int len,
            std::vector<std::vector<uint8_t>>* out) {
    if (!data || len <= 0 || !out) return;
    buf_.insert(buf_.end(), data, data + len);
    while (buf_.size() >= 3) {
      size_t i = 0;
      while (i < buf_.size() && buf_[i] != 0xD3) ++i;
      if (i > 0) buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i));
      if (buf_.size() < 3) return;
      uint16_t payload_len =
          (static_cast<uint16_t>(buf_[1] & 0x03) << 8) | buf_[2];
      size_t frame_len = 3 + payload_len + 3;
      if (frame_len > 40960) {
        buf_.erase(buf_.begin());
        continue;
      }
      if (buf_.size() < frame_len) return;
      out->emplace_back(buf_.begin(), buf_.begin() + static_cast<long>(frame_len));
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(frame_len));
    }
  }

 private:
  std::vector<uint8_t> buf_;
};

}  // namespace

int main(int argc, char* argv[]) {
  const char* serial_dev = "/dev/ttyACM0";
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
      fprintf(stderr, "[base] unknown option: %s\n", argv[i]);
      PrintUsage(argv[0]);
      return 1;
    }
  }

  signal(SIGINT, OnSignal);
  signal(SIGTERM, OnSignal);

  const std::string user = "test01";
  const std::string passwd = "123456";
  const std::string mountpoint = "RTCM32";
  const std::string ntrip_str =
      "STR;RTCM32;RTCM32;RTCM 3.2;1004(1),1005/1007(5),PBS(10);"
      "2;GPS;SGNET;CHN;31;121;1;1;SGCAN;None;B;N;0;;";

  libntrip::NtripCaster caster;
  caster.Init(caster_port, 64, 2000);
  if (!caster.Run()) {
    fprintf(stderr, "[base] NtripCaster Run failed\n");
    return 1;
  }
  printf("[base] NtripCaster listening on 0.0.0.0:%d\n", caster_port);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  libntrip::NtripServer server;
  server.Init("127.0.0.1", caster_port, user, passwd, mountpoint, ntrip_str);
  if (!server.Run()) {
    fprintf(stderr, "[base] NtripServer connect to local caster failed\n");
    caster.Stop();
    return 1;
  }
  printf("[base] NtripServer registered mountpoint %s\n", mountpoint.c_str());

  int serial_fd = OpenSerial(serial_dev, baud);
  if (serial_fd < 0) {
    server.Stop();
    caster.Stop();
    return 1;
  }
  printf("[base] Reading RTCM from %s @ %d baud\n", serial_dev, baud);

  if (g_save_obs) {
    std::string obs_path = BuildObsPath(obs_file, obs_dir, obs_name);
    if (!OpenObsFile(obs_path)) {
      close(serial_fd);
      server.Stop();
      caster.Stop();
      return 1;
    }
  }

  Rtcm3Extractor extractor;
  std::vector<uint8_t> read_buf(4096);
  uint64_t frames_sent = 0;

  while (g_running.load() && caster.service_is_running() &&
         server.service_is_running()) {
    ssize_t n = read(serial_fd, read_buf.data(), read_buf.size());
    if (n < 0) {
      if (errno == EINTR) continue;
      perror("serial read");
      break;
    }
    if (n == 0) continue;

    if (!SaveSerialRxToObs(read_buf.data(), static_cast<size_t>(n))) {
      g_running.store(false);
      break;
    }

    PrintRtcmRawHex(read_buf.data(), static_cast<int>(n));

    std::vector<std::vector<uint8_t>> frames;
    extractor.Feed(read_buf.data(), static_cast<int>(n), &frames);
    for (const auto& frame : frames) {
      if (server.SendData(reinterpret_cast<const char*>(frame.data()),
                          static_cast<int>(frame.size())) != 0) {
        fprintf(stderr, "[base] SendData failed, frame size %zu\n",
                frame.size());
        g_running.store(false);
        break;
      }
      std::cout << "rtcm send tcp ok" << std::endl;
      ++frames_sent;
    }
  }

  close(serial_fd);
  CloseObsFile();
  server.Stop();
  caster.Stop();
  printf("[base] stopped\n");
  return 0;
}
