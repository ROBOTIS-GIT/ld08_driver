// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: LD Robot, Will Son

#include <stdio.h>
#include <iostream>
#include "../include/cmd_interface_linux.hpp"
#include "../include/lipkg.hpp"
#include "../include/transform.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

std::atomic_bool shutdown_requested{false};

void signal_handler(int signum)
{
  RCLCPP_WARN(rclcpp::get_logger("signal"), "Caught signal %d", signum);
  rclcpp::shutdown();  // Triggers on_shutdown hooks
}

void node_cleanup()
{
  RCLCPP_INFO(rclcpp::get_logger("shutdown"), "Shutting down gracefully...");
  shutdown_requested.store(true);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Catch SIGINT and SIGTERM
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  rclcpp::on_shutdown(node_cleanup);

  auto node = rclcpp::Node::make_shared("laser_scan_publisher");
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;

  // default Set polling rate to 40Hz, this should be enough for most cases
  double poll_rate_hz = node->declare_parameter("poll_rate_hz", 40.0);
  rclcpp::Rate loop_rate(poll_rate_hz);
  std::cout << "Lidar poll rate set to " << poll_rate_hz << " Hz"<< std::endl;

  LiPkg * pkg;
  std::string product;
  int32_t ver = 8;
  pkg = new LD08_LiPkg;

  CmdInterfaceLinux cmd_port(ver);
  std::vector<std::pair<std::string, std::string>> device_list;
  std::string port_name;
  cmd_port.GetCmdDevices(device_list);
  for (auto n : device_list) {
    std::cout << n.first << "    " << n.second << std::endl;
    if (strstr(n.second.c_str(), "CP2102")) {
      port_name = n.first;
    }
  }

  if (port_name.empty() == false) {
    std::cout << "FOUND LDS-02" << product << std::endl;
    cmd_port.SetReadCallback(
      [&pkg](const char * byte, size_t len) {
        if (pkg->Parse((const uint8_t *)(byte), len)) {
          pkg->AssemblePacket();
        }
      });

    if (cmd_port.Open(port_name)) {
      std::cout << "LDS-02" << product << " started successfully " << std::endl;
    }

    // char topic_name[20]={0};
    // strcat(topic_name,product.c_str());
    // strcat(topic_name,"/LDLiDAR");
    lidar_pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::QoS(rclcpp::SensorDataQoS())
    );
    
    while (rclcpp::ok() && !shutdown_requested.load()) {
      if (pkg->IsFrameReady()) {
        pkg->setStamp(node->now());
        lidar_pub->publish(pkg->GetLaserScan());
        pkg->ResetFrameReady();
      }
      // Take a short nap
      loop_rate.sleep();
    }
    cmd_port.Close();
  } else {
    std::cout << "Can't find LDS-02" << product << std::endl;
  }

  return 0;
}
