#include "socket_udp.hpp"

SocketUdp::SocketUdp(const std::string& IP, int port, uint bufferSize) {
  std::cout << "Creating socket ..." << std::endl;
  socket_fd  = socket(AF_INET, SOCK_DGRAM, 0);
  this->IP   = IP;
  this->port = port;
  buffer.resize(bufferSize, '\0');
}

SocketUdp::~SocketUdp() {
  std::cout << "closing socket ..." << std::endl;
  close(socket_fd);
}

void SocketUdp::configuration() {
  serv_addr.sin_port        = htons(port);
  serv_addr.sin_addr.s_addr = inet_addr(IP.c_str());
  serv_addr.sin_family      = AF_INET;
}

bool SocketUdp::setDest_addr() {
  bool success = true;
  addrinfo* addrInfo_{nullptr};
  addrinfo hints{};
  hints.ai_family      = AF_INET;
  hints.ai_socktype    = SOCK_DGRAM;
  std::string port_str = std::to_string(port);
  int ret              = getaddrinfo(IP.c_str(), port_str.c_str(), &hints, &addrInfo_);
  if (ret != 0) {
    std::cout << "Error: setting dest_addr sockaddr_storage" << std::endl;
    success = false;
  }
  memcpy(&dest_addr, addrInfo_->ai_addr, addrInfo_->ai_addrlen);
  freeaddrinfo(addrInfo_);
  return success;
}

bool SocketUdp::bindServer() {
  std::cout << "Binding server ..." << std::endl;
  bool connectionSucceded = true;
  configuration();
  int connectionState = bind(socket_fd, reinterpret_cast<sockaddr*>(&serv_addr), sizeof(serv_addr));
  if (connectionState == -1) {
    std::cout << "Unable to connect to IP: " << IP << " port: " << port << std::endl;
    connectionSucceded = false;
  }
  bool setStorage = setDest_addr();

  if (!setStorage) {
    std::cout << "Unable to setDest_addr" << std::endl;
    connectionSucceded = false;
  }
  return connectionSucceded;
}

bool SocketUdp::sending(std::string message) {
  bool sendOK = true;
  const std::vector<unsigned char> msgs{std::cbegin(message), std::cend(message)};
  const socklen_t dest_addr_len{sizeof(dest_addr)};

  int ret = sendto(socket_fd, msgs.data(), msgs.size(), 0, reinterpret_cast<sockaddr*>(&dest_addr),
                   dest_addr_len);
  if (ret == -1) {
    std::cout << "sending: It has been impossible to send the message " << message << std::endl;
    sendOK = false;
  }
  return sendOK;
}

std::string SocketUdp::receiving(const int flags) {
  std::string resp_res = "";
  socklen_t serv_addr_len{sizeof(dest_addr)};
  int ret = recvfrom(socket_fd, buffer.data(), buffer.size(), flags,
                     reinterpret_cast<sockaddr*>(&dest_addr), &serv_addr_len);

  if (ret == -1) {
    return resp_res;
  } else if (ret < 1) {
    return resp_res;
  }

  std::string resp{buffer.cbegin(), buffer.cbegin() + ret};
  resp.erase(resp.find_last_not_of(" \n\r\t") + 1);
  resp_res = resp;
  /*cout<<resp<<endl;
  cout<<resp_res<<endl;*/
  return resp_res;
}

inline int SocketUdp::getSocketfd() const { return socket_fd; }

inline const char* SocketUdp::getIP() const { return IP.c_str(); }

inline int SocketUdp::getPort() const { return port; }
