// neptune_rover.cc - Ubuntu-B (rover): NtripClient -> write RTCM to /dev/ttyACM0
// RTCM downlink only; no GGA uplink.
//
// Usage:
//   ./neptune_rover --caster-ip 192.168.2.106 [--device /dev/ttyACM0] [--baud 921600]

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "ntrip/ntrip_client.h"

namespace {

std::atomic_bool g_running{true};
int g_serial_fd = -1;
std::atomic<uint64_t> g_bytes_received{0};

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
      perror("serial write");
      g_running.store(false);
      return;
    }
    p += w;
    left -= static_cast<int>(w);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  const char* serial_dev = "/dev/ttyACM0";
  const char* caster_ip = nullptr;
  int baud = 115200;
  int caster_port = 8090;

  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--device") && i + 1 < argc) serial_dev = argv[++i];
    else if (!strcmp(argv[i], "--baud") && i + 1 < argc) baud = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--caster-ip") && i + 1 < argc)
      caster_ip = argv[++i];
    else if (!strcmp(argv[i], "--port") && i + 1 < argc)
      caster_port = atoi(argv[++i]);
  }

  if (caster_ip == nullptr) {
    fprintf(stderr,
            "Usage: %s --caster-ip <base_station_LAN_IP> "
            "[--device /dev/ttyACM0] [--baud 115200] [--port 8090]\n",
            argv[0]);
    return 1;
  }

  signal(SIGINT, OnSignal);
  signal(SIGTERM, OnSignal);

  g_serial_fd = OpenSerial(serial_dev, baud);
  if (g_serial_fd < 0) return 1;
  printf("[rover] serial %s @ %d baud opened\n", serial_dev, baud);

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
  close(g_serial_fd);
  g_serial_fd = -1;
  printf("[rover] stopped, total RTCM bytes %lu\n",
         static_cast<unsigned long>(g_bytes_received.load()));
  return 0;
}
