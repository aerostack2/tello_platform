#ifndef TELLO_H
#define TELLO_H

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "socketudp.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

const int port_command = 8889;
const int port_state   = 8890;
const char* const IP_command{"192.168.10.1"};
const char* const URL_stream{"udp://0.0.0.0:11111"};

struct coordinates {
  double x, y, z;
};

class Tello {
private:
  SocketUdp* commandSender;
  SocketUdp* stateRecv;
  // State information.
  std::vector<double> state;
  coordinates orientation;
  coordinates velocity;
  coordinates acceleration;

  double battery;
  double timeMotor;
  double timeOF;
  double height;
  double barometer;

  cv::Mat frame;
  bool connected;

private:
  bool filterState(std::string data);
  void update();
  void threadStateFnc();

public:
  Tello();   // creating sockets
  ~Tello();  // closing sockets

  std::pair<bool, std::vector<double>> getState();
  std::pair<bool, std::string> sendCommand(std::string command);

  std::vector<coordinates> getIMU();
  coordinates getOrientation();
  coordinates getVelocity();
  coordinates getAcceleration();
  double getBarometer();
  bool isConnected();
  double getHeight();
  void streamVideo();
  double getBattery();
  cv::Mat getFrame();

  bool x_motion(double x);                                     // Forward or backward move.
  bool y_motion(double y);                                     // right or left move.
  bool z_motion(double z);                                     // up or left down.
  bool yaw_twist(double yaw);                                  // clockwise or counterclockwise
  bool speedMotion(double x, double y, double z, double yaw);  //
};

#endif  // TELLO_H