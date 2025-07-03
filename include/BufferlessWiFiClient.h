/*
  This is a fork of the available WiFiClient library provided within the
  PlatformIO Arduino Framework for esp32. It removes the need of a dedicated
  buffer by directly passing Client interface code to lwip socket operations.
  The main reason for this is occuring memory fragmentation when the application
  connects / stops clients frequently in the original code, as the buffer will
  alwas be freed and allocated again.
  
  Copyright (c) 2025 Sebastian Wagner.  All right reserved.

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

#ifndef _BUFFERLESSWIFICLIENT_H_
#define _BUFFERLESSWIFICLIENT_H_


#include "Arduino.h"
#include "Client.h"
#include <memory>

class BufferlessWiFiClientSocketHandle;

class ESPLwIPClientBase : public Client
{
public:
        virtual int connect(IPAddress ip, uint16_t port, int32_t timeout) = 0;
        virtual int connect(const char *host, uint16_t port, int32_t timeout) = 0;
        virtual int setTimeout(uint32_t seconds) = 0;
};

class BufferlessWiFiClient : public ESPLwIPClientBase
{
private:
    void fail(bool stopClient = true);

protected:
    std::shared_ptr<BufferlessWiFiClientSocketHandle> clientSocketHandle;
    bool _connected;
    int _timeout;

public:
    BufferlessWiFiClient *next;
    BufferlessWiFiClient();
    BufferlessWiFiClient(int fd);
    ~BufferlessWiFiClient();
    int connect(IPAddress ip, uint16_t port);
    int connect(IPAddress ip, uint16_t port, int32_t timeout_ms);
    int connect(const char *host, uint16_t port);
    int connect(const char *host, uint16_t port, int32_t timeout_ms);
    size_t write(uint8_t data);
    size_t write(const uint8_t *buf, size_t size);
    size_t write_P(PGM_P buf, size_t size);
    size_t write(Stream &stream);
    int available();
    int read();
    int read(uint8_t *buf, size_t size);
    int peek();
    void flush();
    void stop();
    uint8_t connected();

    operator bool()
    {
        return connected();
    }
    BufferlessWiFiClient & operator=(const BufferlessWiFiClient &other);
    bool operator==(const bool value)
    {
        return bool() == value;
    }
    bool operator!=(const bool value)
    {
        return bool() != value;
    }
    bool operator==(const BufferlessWiFiClient&);
    bool operator!=(const BufferlessWiFiClient& rhs)
    {
        return !this->operator==(rhs);
    };

    virtual int fd() const;

    int setSocketOption(int option, char* value, size_t len);
    int setSocketOption(int level, int option, const void* value, size_t len);
    int setOption(int option, int *value);
    int getOption(int option, int *value);
    int setTimeout(uint32_t milliSeconds);
    int setNoDelay(bool nodelay);
    bool getNoDelay();

    IPAddress remoteIP() const;
    IPAddress remoteIP(int fd) const;
    uint16_t remotePort() const;
    uint16_t remotePort(int fd) const;
    IPAddress localIP() const;
    IPAddress localIP(int fd) const;
    uint16_t localPort() const;
    uint16_t localPort(int fd) const;

    //friend class WiFiServer;
    using Print::write;
};

#endif /* _BUFFERLESSWIFICLIENT_H_ */
