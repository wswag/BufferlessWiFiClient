/*
  Client.h - Client class for Raspberry Pi
  Copyright (c) 2016 Hristo Gochkov  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "BufferlessWiFiClient.h"
#include "WiFi.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>

#ifndef WIFI_CLIENT_DEF_CONN_TIMEOUT_MS
  #define WIFI_CLIENT_DEF_CONN_TIMEOUT_MS  (3000)
#endif
#ifndef WIFI_CLIENT_MAX_WRITE_RETRY
  #define WIFI_CLIENT_MAX_WRITE_RETRY      (10)
#endif
#ifndef WIFI_CLIENT_SELECT_TIMEOUT_US
  #define WIFI_CLIENT_SELECT_TIMEOUT_US    (1000000)
#endif

#undef connect
#undef write
#undef read

class BufferlessWiFiClientSocketHandle {
private:
    int sockfd;

public:
    BufferlessWiFiClientSocketHandle(int fd):sockfd(fd)
    {
    }

    ~BufferlessWiFiClientSocketHandle()
    {
        close(sockfd);
    }

    int fd()
    {
        return sockfd;
    }
};

BufferlessWiFiClient::BufferlessWiFiClient():_connected(false)
{
}

BufferlessWiFiClient::BufferlessWiFiClient(int fd):_connected(true)
{
    clientSocketHandle.reset(new BufferlessWiFiClientSocketHandle(fd));
}

BufferlessWiFiClient::~BufferlessWiFiClient()
{
    stop();
}

BufferlessWiFiClient & BufferlessWiFiClient::operator=(const BufferlessWiFiClient &other)
{
    stop();
    clientSocketHandle = other.clientSocketHandle;
    _connected = other._connected;
    return *this;
}

void BufferlessWiFiClient::stop()
{
    clientSocketHandle = NULL;
    _connected = false;
}

int BufferlessWiFiClient::connect(IPAddress ip, uint16_t port)
{
    return connect(ip,port,getTimeout());
}
int BufferlessWiFiClient::connect(IPAddress ip, uint16_t port, int32_t timeout_ms)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        log_e("socket: %d", errno);
        return 0;
    }
    fcntl( sockfd, F_SETFL, fcntl( sockfd, F_GETFL, 0 ) | O_NONBLOCK );

    uint32_t ip_addr = ip;
    struct sockaddr_in serveraddr;
    memset((char *) &serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    memcpy((void *)&serveraddr.sin_addr.s_addr, (const void *)(&ip_addr), 4);
    serveraddr.sin_port = htons(port);
    fd_set fdset;
    struct timeval tv;
    FD_ZERO(&fdset);
    FD_SET(sockfd, &fdset);
    int timeout = getTimeout();
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout  % 1000) * 1000;

#ifdef ESP_IDF_VERSION_MAJOR
    int res = lwip_connect(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr));
#else
    int res = lwip_connect_r(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr));
#endif
    if (res < 0 && errno != EINPROGRESS) {
        log_e("connect on fd %d, errno: %d, \"%s\"", sockfd, errno, strerror(errno));
        close(sockfd);
        return 0;
    }

    res = select(sockfd + 1, nullptr, &fdset, nullptr, timeout<0 ? nullptr : &tv);
    if (res < 0) {
        log_e("select on fd %d, errno: %d, \"%s\"", sockfd, errno, strerror(errno));
        close(sockfd);
        return 0;
    } else if (res == 0) {
        log_i("select returned due to timeout %d ms for fd %d", timeout, sockfd);
        close(sockfd);
        return 0;
    } else {
        int sockerr;
        socklen_t len = (socklen_t)sizeof(int);
        res = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &sockerr, &len);

        if (res < 0) {
            log_e("getsockopt on fd %d, errno: %d, \"%s\"", sockfd, errno, strerror(errno));
            close(sockfd);
            return 0;
        }

        if (sockerr != 0) {
            log_e("socket error on fd %d, errno: %d, \"%s\"", sockfd, sockerr, strerror(sockerr));
            close(sockfd);
            return 0;
        }
    }

#define ROE_WIFICLIENT(x,msg) { if (((x)<0)) { log_e("Setsockopt '" msg "'' on fd %d failed. errno: %d, \"%s\"", sockfd, errno, strerror(errno)); return 0; }}
    ROE_WIFICLIENT(setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)),"SO_SNDTIMEO");
    ROE_WIFICLIENT(setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)),"SO_RCVTIMEO");

    // These are also set in WiFiClientSecure, should be set here too?
    //ROE_WIFICLIENT(setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable)),"TCP_NODELAY"); 
    //ROE_WIFICLIENT (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable)),"SO_KEEPALIVE");

    fcntl( sockfd, F_SETFL, fcntl( sockfd, F_GETFL, 0 ) & (~O_NONBLOCK) );
    clientSocketHandle.reset(new BufferlessWiFiClientSocketHandle(sockfd));

    _connected = true;
    return 1;
}

int BufferlessWiFiClient::connect(const char *host, uint16_t port)
{
    return connect(host,port,getTimeout());
}

int BufferlessWiFiClient::connect(const char *host, uint16_t port, int32_t timeout_ms)
{
    IPAddress srv((uint32_t)0);
    if(!WiFiGenericClass::hostByName(host, srv)){
        return 0;
    }
    return connect(srv, port, timeout_ms);
}

int BufferlessWiFiClient::setSocketOption(int option, char* value, size_t len)
{
    return setSocketOption(SOL_SOCKET, option, (const void*)value, len);
}

int BufferlessWiFiClient::setSocketOption(int level, int option, const void* value, size_t len)
{
    int res = setsockopt(fd(), level, option, value, len);
    if(res < 0) {
        log_e("fail on %d, errno: %d, \"%s\"", fd(), errno, strerror(errno));
    }
    return res;
}

int BufferlessWiFiClient::setTimeout(uint32_t milliSeconds)
{
    Client::setTimeout(milliSeconds); // This should be here?
    if(clientSocketHandle) {
        struct timeval tv;
        tv.tv_sec = milliSeconds / 1000;
        tv.tv_usec = (milliSeconds % 1000) * 1000;
        if(setSocketOption(SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) < 0) {
            return -1;
        }
        return setSocketOption(SO_SNDTIMEO, (char *)&tv, sizeof(struct timeval));
    }
    else {
        return 0;
    }
}

int BufferlessWiFiClient::setOption(int option, int *value)
{
    return setSocketOption(IPPROTO_TCP, option, (const void*)value, sizeof(int));
}

int BufferlessWiFiClient::getOption(int option, int *value)
{
	socklen_t size = sizeof(int);
    int res = getsockopt(fd(), IPPROTO_TCP, option, (char *)value, &size);
    if(res < 0) {
        log_e("fail on fd %d, errno: %d, \"%s\"", fd(), errno, strerror(errno));
    }
    return res;
}

int BufferlessWiFiClient::setNoDelay(bool nodelay)
{
    int flag = nodelay;
    return setOption(TCP_NODELAY, &flag);
}

bool BufferlessWiFiClient::getNoDelay()
{
    int flag = 0;
    getOption(TCP_NODELAY, &flag);
    return flag;
}

size_t BufferlessWiFiClient::write(uint8_t data)
{
    return write(&data, 1);
}

int BufferlessWiFiClient::read()
{
    uint8_t data = 0;
    int res = read(&data, 1);
    if(res < 0) {
        return res;
    }
    if (res == 0) {  //  No data available.
        return -1;
    }
    return data;
}

void BufferlessWiFiClient::fail(bool stopClient)
{
  log_e("fail on fd %d, errno: %d, \"%s\"", fd(), errno, strerror(errno));
  if (stopClient) stop();
}

size_t BufferlessWiFiClient::write(const uint8_t *buf, size_t size)
{
    int res =0;
    int retry = WIFI_CLIENT_MAX_WRITE_RETRY;
    int socketFileDescriptor = fd();
    size_t totalBytesSent = 0;
    size_t bytesRemaining = size;

    if(!_connected || (socketFileDescriptor < 0)) {
        return 0;
    }

    while(retry) {
        //use select to make sure the socket is ready for writing
        fd_set set;
        struct timeval tv;
        FD_ZERO(&set);        // empties the set
        FD_SET(socketFileDescriptor, &set); // adds FD to the set
        tv.tv_sec = 0;
        tv.tv_usec = WIFI_CLIENT_SELECT_TIMEOUT_US;
        retry--;

        if(select(socketFileDescriptor + 1, NULL, &set, NULL, &tv) < 0) {
            return 0;
        }

        if(FD_ISSET(socketFileDescriptor, &set)) {
            res = send(socketFileDescriptor, (void*) buf, bytesRemaining, MSG_DONTWAIT);
            if(res > 0) {
                totalBytesSent += res;
                if (totalBytesSent >= size) {
                    //completed successfully
                    retry = 0;
                } else {
                    buf += res;
                    bytesRemaining -= res;
                    retry = WIFI_CLIENT_MAX_WRITE_RETRY;
                }
            }
            else if(res < 0) {
                bool stopClient = errno != EAGAIN;
                fail(stopClient);
                if (stopClient) break;
            }
            else {
                // Try again
            }
        }
    }
    return totalBytesSent;
}

size_t BufferlessWiFiClient::write_P(PGM_P buf, size_t size)
{
    return write(buf, size);
}

size_t BufferlessWiFiClient::write(Stream &stream)
{
    // dont use heap memory to avoid fragmentation
    uint8_t buf[1360];
    size_t toRead = 0, toWrite = 0, written = 0;
    size_t available = stream.available();
    while(available){
        toRead = (available > 1360)?1360:available;
        toWrite = stream.readBytes(buf, toRead);
        written += write(buf, toWrite);
        available = stream.available();
    }
    return written;
}

int BufferlessWiFiClient::read(uint8_t *buf, size_t size)
{
    if(!clientSocketHandle){
        return 0;
    }

    int availableBytes = available();
    if (availableBytes == 0) {
      return 0;
    }

    size_t bytesToRead = availableBytes < size ? availableBytes : size;
    int res = recv(fd(), buf, bytesToRead, MSG_DONTWAIT);
    if(res < 0) {
        fail();
        return -1;
    }
    return res;
}

int BufferlessWiFiClient::peek()
{
    uint8_t buffer;
    int res = recv(fd(), &buffer, 1, MSG_PEEK);
    if (res < 0) {
      fail();
      return 0; 
    }
    return buffer;
}

int BufferlessWiFiClient::available()
{
    if(!clientSocketHandle){
        return 0;
    }

    int count;
#ifdef ESP_IDF_VERSION_MAJOR
    int res = lwip_ioctl(fd(), FIONREAD, &count);
#else
    int res = lwip_ioctl_r(fd(), FIONREAD, &count);
#endif
    if(res < 0) {
        fail();
        return 0;
    }
    return count;
}

// Though flushing means to send all pending data,
// seems that in Arduino it also means to clear RX
void BufferlessWiFiClient::flush() {
  if(!clientSocketHandle){
    return;
  }
  uint8_t tmp;
  while (available())
    read(&tmp, 1);
}

uint8_t BufferlessWiFiClient::connected()
{
    if (_connected) {
        uint8_t dummy;
        int res = recv(fd(), &dummy, 0, MSG_DONTWAIT);
        // avoid unused var warning by gcc
        (void)res;
        // recv only sets errno if res is <= 0
        if (res <= 0){
          switch (errno) {
              case EWOULDBLOCK:
              case ENOENT: //caused by vfs
                  _connected = true;
                  break;
              case ENOTCONN:
              case EPIPE:
              case ECONNRESET:
              case ECONNREFUSED:
              case ECONNABORTED:
                  _connected = false;
                  log_d("Disconnected: RES: %d, ERR: %d", res, errno);
                  break;
              default:
                  log_i("Unexpected: RES: %d, ERR: %d", res, errno);
                  _connected = true;
                  break;
          }
        } else {
          _connected = true;
        }
    }
    return _connected;
}

IPAddress BufferlessWiFiClient::remoteIP(int fd) const
{
    struct sockaddr_storage addr;
    socklen_t len = sizeof addr;
    getpeername(fd, (struct sockaddr*)&addr, &len);
    struct sockaddr_in *s = (struct sockaddr_in *)&addr;
    return IPAddress((uint32_t)(s->sin_addr.s_addr));
}

uint16_t BufferlessWiFiClient::remotePort(int fd) const
{
    struct sockaddr_storage addr;
    socklen_t len = sizeof addr;
    getpeername(fd, (struct sockaddr*)&addr, &len);
    struct sockaddr_in *s = (struct sockaddr_in *)&addr;
    return ntohs(s->sin_port);
}

IPAddress BufferlessWiFiClient::remoteIP() const
{
    return remoteIP(fd());
}

uint16_t BufferlessWiFiClient::remotePort() const
{
    return remotePort(fd());
}

IPAddress BufferlessWiFiClient::localIP(int fd) const
{
    struct sockaddr_storage addr;
    socklen_t len = sizeof addr;
    getsockname(fd, (struct sockaddr*)&addr, &len);
    struct sockaddr_in *s = (struct sockaddr_in *)&addr;
    return IPAddress((uint32_t)(s->sin_addr.s_addr));
}

uint16_t BufferlessWiFiClient::localPort(int fd) const
{
    struct sockaddr_storage addr;
    socklen_t len = sizeof addr;
    getsockname(fd, (struct sockaddr*)&addr, &len);
    struct sockaddr_in *s = (struct sockaddr_in *)&addr;
    return ntohs(s->sin_port);
}

IPAddress BufferlessWiFiClient::localIP() const
{
    return localIP(fd());
}

uint16_t BufferlessWiFiClient::localPort() const
{
    return localPort(fd());
}

bool BufferlessWiFiClient::operator==(const BufferlessWiFiClient& rhs)
{
    return clientSocketHandle == rhs.clientSocketHandle && remotePort() == rhs.remotePort() && remoteIP() == rhs.remoteIP();
}

int BufferlessWiFiClient::fd() const
{
  WiFiClient c;
    if (clientSocketHandle == NULL) {
        return -1;
    } else {
        return clientSocketHandle->fd();
    }
}

