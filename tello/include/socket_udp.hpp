#ifndef SOCKETUDP_H
#define SOCKETUDP_H

#include <stdio.h>
#include <iostream>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

class SocketUdp {
private:
  int socket_fd;
  std::string IP;
  int port;
  sockaddr_in serv_addr;
  std::vector<unsigned char> buffer;
  sockaddr_storage dest_addr;

private:
  void configuration();
  bool setDest_addr();

public:
  SocketUdp(const std::string& IP = "0.0.0.0", int port = 0, uint bufferSize = 1024);
  ~SocketUdp();  // closing socket

  bool bindServer();
  int getSocketfd() const;
  const char* getIP() const;
  int getPort() const;

  inline void setSocketfd(int socket_fd) { this->socket_fd = socket_fd; }
  inline void setIP(const char* IP) { this->IP = IP; }
  inline void setPort(int port) { this->port = port; }

  bool sending(std::string message);
  std::string receiving(const int flags = MSG_DONTWAIT);
};

#endif  // SOCKETUDP_H