#include "tello.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>

static std::vector<std::string> split(const std::string& target, char c) {
  std::string temp;
  std::stringstream stringstream{target};
  std::vector<std::string> result;

  while (std::getline(stringstream, temp, c)) {
    result.push_back(temp);
  }

  return result;
}

Tello::Tello() {
  commandSender = new SocketUdp;
  stateRecv     = new SocketUdp;

  commandSender->setIP((char*)IP_command);
  commandSender->setPort(port_command);

  stateRecv->setPort(port_state);

  commandSender->bindServer();

  std::pair<bool, std::string> response = sendCommand("command");
  if (response.first == false or response.second == "Error") {
    std::cout << "Error: Connecting to tello" << std::endl;
    exit(-1);
  }
  stateRecv->bindServer();
  state.resize(16);
  update();

  std::thread stateThd(&Tello::threadStateFnc, this);
  stateThd.detach();
  std::thread videoThd(&Tello::streamVideo, this);
  videoThd.detach();
}

Tello::~Tello() {
  commandSender->~SocketUdp();
  stateRecv->~SocketUdp();

  delete (commandSender);
  delete (stateRecv);
}

std::pair<bool, std::string> Tello::sendCommand(std::string command) {
  std::pair<bool, std::string> ret;
  bool success         = false;
  uint cont            = 0;
  const int timeLimit  = 10;
  std::string msgsBack = "";
  std::cout << command << std::endl;
  do {
    commandSender->sending(command);
    sleep(1);
    msgsBack = commandSender->receiving();
    cont++;
    // cout<<cont<<endl;
  } while ((msgsBack.length() == 0) and (cont <= timeLimit));

  if (cont > timeLimit) {
    std::cout << "The command '" << command << "' is not received." << std::endl;
  } else {
    success = true;
  }
  ret.first  = success;
  ret.second = msgsBack;
  return ret;
}

void Tello::threadStateFnc() {
  std::pair<bool, std::vector<double>> resp;

  for (;;) {
    resp = getState();
    sleep(0.2);
  }
}

std::pair<bool, std::vector<double>> Tello::getState() {
  // const int timeLimit = 10;
  std::string msgs = "";
  std::pair<bool, std::vector<double>> ret;
  ret.first = true;

  msgs = stateRecv->receiving();

  if (msgs.length() == 0) {
    ret.first = false;
  } else
    ret.first = filterState(msgs);

  ret.second = state;
  return ret;
}

bool Tello::filterState(std::string data) {
  bool success = true;
  std::vector<std::string> values, values_;
  values = split(data, ';');
  for (int value_idx = 0; value_idx < static_cast<int>(values.size()); value_idx++) {
    if (values[value_idx].length() != 0) {
      values_ = split(values[value_idx], ':');
      // cout<<values[value_idx]<<endl;
      if (value_idx <= static_cast<int>(state.size())) {
        state[value_idx] = stod(values_[1]);
      } else {
        std::cout << "Error: Adding data to the 'state' attribute" << std::endl;
        success = false;
      }
    }
  }
  if (success) update();
  return success;
}

void Tello::update() {
  orientation.x = state[0];
  orientation.y = state[1];
  orientation.z = state[2];

  velocity.x = state[3];
  velocity.y = state[4];
  velocity.z = state[5];

  timeOF    = state[8];
  height    = state[9];
  battery   = (int)state[10];
  timeMotor = state[12];
  barometer = state[11];

  acceleration.x = state[13];
  acceleration.y = state[14];
  acceleration.z = state[15];
}

coordinates Tello::getOrientation() { return orientation; }

coordinates Tello::getVelocity() { return velocity; }

coordinates Tello::getAcceleration() { return acceleration; }

double Tello::getBarometer() { return barometer; }

std::vector<coordinates> Tello::getIMU() {
  std::vector<coordinates> imu;
  imu.push_back(orientation);
  imu.push_back(velocity);
  imu.push_back(acceleration);
  return imu;
}

double Tello::getHeight() { return height; }

double Tello::getBattery() { return battery; }

bool Tello::isConnected() { return connected; }

void Tello::streamVideo() {
  std::pair<bool, std::string> response = sendCommand("streamon");

  if (response.first) {
    cv::VideoCapture capture{URL_stream, cv::CAP_FFMPEG};
    while (true) {
      capture >> frame;

      if (!frame.empty()) {
        imshow("Tello Stream", frame);
      }
      if (cv::waitKey(1) == 27) {
        break;
      }
    }
  }
}

// Forward or backward move.
bool Tello::x_motion(double x) {
  std::pair<bool, std::string> response;
  std::string msg;
  response.first  = false;
  response.second = "";
  bool ret        = false;
  if (x > 0) {
    msg      = "forward " + std::to_string(abs(x));
    response = sendCommand(msg);
  } else if (x < 0) {
    msg      = "back " + std::to_string(abs(x));
    response = sendCommand(msg);
  } else {
    response.first = true;
  }
  if (response.first and response.second != "Error") {
    ret = true;
  }
  return ret;
}

// right or left move.
bool Tello::y_motion(double y) {
  std::pair<bool, std::string> response;
  std::string msg;
  response.first  = false;
  response.second = "";
  bool ret        = false;
  if (y > 0) {
    msg      = "right " + std::to_string(abs(y));
    response = sendCommand(msg);
  } else if (y < 0) {
    msg      = "left " + std::to_string(abs(y));
    response = sendCommand(msg);
  } else {
    response.first = true;
  }
  if (response.first and response.second != "Error") {
    ret = true;
  }
  return ret;
}

// up or left down.
bool Tello::z_motion(double z) {
  std::pair<bool, std::string> response;
  std::string msg;
  response.first  = false;
  response.second = "";
  bool ret        = false;
  if (z > 0) {
    msg      = "up " + std::to_string(int(abs(z)));
    response = sendCommand(msg);
  } else if (z < 0) {
    msg      = "down " + std::to_string(int(abs(z)));
    response = sendCommand(msg);
  } else {
    response.first = true;
  }
  if (response.first and response.second != "Error") {
    ret = true;
  }
  return ret;
}

// clockwise or counterclockwise
bool Tello::yaw_twist(double yaw) {
  std::pair<bool, std::string> response;
  std::string msg;
  response.first  = false;
  response.second = "";
  bool ret        = false;
  if (yaw > 0) {
    msg      = "cw " + std::to_string(abs(yaw));
    response = sendCommand(msg);
  } else if (yaw < 0) {
    msg      = "ccw " + std::to_string(abs(yaw));
    response = sendCommand(msg);
  } else {
    response.first = true;
  }
  if (response.first and response.second != "Error") {
    ret = true;
  }
  return ret;
}

bool Tello::speedMotion(double x, double y, double z, double yaw) {
  bool ret = false;
  std::pair<bool, std::string> response;
  std::string msg;
  response.first  = false;
  response.second = "";

  msg = "rc " + std::to_string(int(x)) + " " + std::to_string(int(y)) + " " +
        std::to_string(int(z)) + " " + std::to_string(int(yaw));
  response = sendCommand(msg);

  if (response.first and response.second != "Error") {
    ret = true;
  }
  return ret;
}