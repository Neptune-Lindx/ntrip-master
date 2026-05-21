// MIT License
//
// Copyright (c) 2021 Yuming Meng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ntrip/ntrip_client.h"

#if defined(__linux__)
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#define ENABLE_TCP_KEEPALIVE
#endif  // defined(__linux__)
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <string>
#include <thread>

#include "ntrip/ntrip_util.h"
#include "cmake_definition.h"

namespace libntrip {

namespace {

using socket_t = decltype(socket(AF_INET, SOCK_STREAM, 0));

constexpr int kBufferSize = 65536;

}  // namespace

bool NtripClient::Run(void) {
  if (service_is_running_.load()) return true;
  Stop();
  socket_t socket_fd;
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(server_port_);
  server_addr.sin_addr.s_addr = inet_addr(server_ip_.c_str());
#if defined(WIN32) || defined(_WIN32)
  WSADATA ws_data;
  if (WSAStartup(MAKEWORD(2, 2), &ws_data) != 0) {
    return false;
  }
  socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (socket_fd == INVALID_SOCKET) {
    printf("Create socket failed!\r\n");
    WSACleanup();
    return false;
  }
#else
  socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd == -1) {
    printf("Create socket failed, errno=%d (%s)\r\n", errno, strerror(errno));
    return false;
  }
#endif
  if (connect(socket_fd, reinterpret_cast<struct sockaddr*>(&server_addr),
              sizeof(server_addr)) < 0) {
    printf("Connect to NtripCaster[%s:%d] failed, errno=%d (%s)\r\n",
           server_ip_.c_str(), server_port_, errno, strerror(errno));
#if defined(WIN32) || defined(_WIN32)
    closesocket(socket_fd);
    WSACleanup();
#else
    close(socket_fd);
#endif
    return false;
  }

  int ret = -1;
  std::string user_passwd = user_ + ":" + passwd_;
  std::string user_passwd_base64;
  std::unique_ptr<char[]> buffer(
      new char[kBufferSize], std::default_delete<char[]>());
  Base64Encode(user_passwd, &user_passwd_base64);
  ret = snprintf(buffer.get(), kBufferSize - 1,
                 "GET /%s HTTP/1.1\r\n"
                 "User-Agent: %s\r\n"
                 "Authorization: Basic %s\r\n"
                 "\r\n",
                 mountpoint_.c_str(), kClientAgent, user_passwd_base64.c_str());
  if (send(socket_fd, buffer.get(), ret, 0) < 0) {
    printf("Send request failed, errno=%d (%s)\r\n", errno, strerror(errno));
#if defined(WIN32) || defined(_WIN32)
    closesocket(socket_fd);
    WSACleanup();
#else
    close(socket_fd);
#endif
    return false;
  }

  int timeout = 30;
  while (timeout--) {
    ret = recv(socket_fd, buffer.get(), kBufferSize, 0);
    if (ret > 0) {
      std::string result(buffer.get(), ret);
      if ((result.find("HTTP/1.1 200 OK") != std::string::npos) ||
          (result.find("ICY 200 OK") != std::string::npos)) {
        break;
      }
      printf("Request result: %s\r\n", result.c_str());
    } else if (ret == 0) {
      printf("Remote socket close during handshake\r\n");
#if defined(WIN32) || defined(_WIN32)
      closesocket(socket_fd);
      WSACleanup();
#else
      close(socket_fd);
#endif
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (timeout <= 0) {
    printf("NtripCaster[%s:%d %s %s %s] access failed!!!\r\n",
           server_ip_.c_str(), server_port_, user_.c_str(), passwd_.c_str(),
           mountpoint_.c_str());
#if defined(WIN32) || defined(_WIN32)
    closesocket(socket_fd);
    WSACleanup();
#else
    close(socket_fd);
#endif
    return false;
  }

#if defined(__linux__)
  int flags = fcntl(socket_fd, F_GETFL);
  if (flags >= 0) {
    fcntl(socket_fd, F_SETFL, flags & ~O_NONBLOCK);
  }
#if defined(ENABLE_TCP_KEEPALIVE)
  int keepalive = 1;
  int keepidle = 30;
  int keepinterval = 5;
  int keepcount = 3;
  setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
  setsockopt(socket_fd, SOL_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
  setsockopt(socket_fd, SOL_TCP, TCP_KEEPINTVL, &keepinterval,
             sizeof(keepinterval));
  setsockopt(socket_fd, SOL_TCP, TCP_KEEPCNT, &keepcount, sizeof(keepcount));
#endif
#endif

  socket_fd_ = socket_fd;
  thread_.reset(&NtripClient::ThreadHandler, this);
  return true;
}

void NtripClient::Stop(void) {
  service_is_running_.store(false);
#if defined(WIN32) || defined(_WIN32)
  if (socket_fd_ != INVALID_SOCKET) {
    shutdown(socket_fd_, SD_BOTH);
  }
#else
  if (socket_fd_ >= 0) {
    shutdown(socket_fd_, SHUT_RDWR);
  }
#endif
  thread_.join();
#if defined(WIN32) || defined(_WIN32)
  if (socket_fd_ != INVALID_SOCKET) {
    closesocket(socket_fd_);
    WSACleanup();
    socket_fd_ = INVALID_SOCKET;
  }
#else
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
#endif
}

void NtripClient::ThreadHandler(void) {
  service_is_running_.store(true);
  std::unique_ptr<char[]> buffer(
      new char[kBufferSize], std::default_delete<char[]>());
  printf("NtripClient service running (RTCM only, no GGA uplink)...\r\n");
  while (service_is_running_.load()) {
    int ret = recv(socket_fd_, buffer.get(), kBufferSize, 0);
    if (ret == 0) {
      printf("Remote socket close\r\n");
      break;
    }
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      printf("Remote socket error, errno=%d (%s)\r\n", errno, strerror(errno));
      break;
    }
    callback_(buffer.get(), ret);
  }
  printf("NtripClient service done.\r\n");
  service_is_running_.store(false);
}

}  // namespace libntrip
