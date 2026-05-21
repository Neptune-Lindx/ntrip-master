// neptune_base.cc - Ubuntu-A (base): NtripCaster + serial RTCM read + NtripServer
//
// Topology:
//   base board -> /dev/ttyACM0 -> read RTCM -> NtripServer -> local NtripCaster:8090
//   rover connects to this host's LAN IP on port 8090.

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
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "ntrip/ntrip_caster.h"
#include "ntrip/ntrip_server.h"

namespace {

std::atomic_bool g_running{true};

void OnSignal(int) { g_running.store(false); }

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

  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--device") && i + 1 < argc) serial_dev = argv[++i];
    else if (!strcmp(argv[i], "--baud") && i + 1 < argc) baud = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--port") && i + 1 < argc)
      caster_port = atoi(argv[++i]);
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
  server.Stop();
  caster.Stop();
  printf("[base] stopped\n");
  return 0;
}
