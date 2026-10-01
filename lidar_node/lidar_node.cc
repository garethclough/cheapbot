/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>


std::string topic_pub = "/cmd_vel";   //publish to this topic
gz::transport::Node node;
auto pub = node.Advertise<gz::msgs::Twist>(topic_pub);

void imuCb(const gz::msgs::IMU &_msg)
{
    const auto &orientation = _msg.orientation();

    std::cout
        << "Orientation: "
        << "x=" << orientation.x()
        << " y=" << orientation.y()
        << " z=" << orientation.z()
        << " w=" << orientation.w()
        << std::endl;

    const auto &angularVelocity = _msg.angular_velocity();

    std::cout
        << "Angular velocity: "
        << "x=" << angularVelocity.x()
        << " y=" << angularVelocity.y()
        << " z=" << angularVelocity.z()
        << std::endl;

    const auto &linearAcceleration = _msg.linear_acceleration();

    std::cout
        << "Acceleration: "
        << "x=" << linearAcceleration.x()
        << " y=" << linearAcceleration.y()
        << " z=" << linearAcceleration.z()
        << std::endl;
}


//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const gz::msgs::LaserScan &_msg)
{
  gz::msgs::Twist data;

  bool allMore = true;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < 1.0)
    {
      allMore = false;
      break;
    }
  }
  if (allMore) //if all bigger than one
  {
    data.mutable_linear()->set_x(0.5);
    data.mutable_angular()->set_z(0.0);
  }
  else
  {
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.5);
  }
  pub.Publish(data);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  std::string topic_sub = "/lidar";   // subscribe to this topic
  // Subscribe to a topic by registering a callback.
  if (!node.Subscribe(topic_sub, cb))
  {
    std::cerr << "Error subscribing to topic [" << topic_sub << "]" << std::endl;
    return -1;
  }

    std::string topic_imu = "/imu";

    if (!node.Subscribe(topic_imu, imuCb))
    {
        std::cerr << "Error subscribing to topic ["
                << topic_imu << "]" << std::endl;
        return -1;
    }

  // Zzzzzz.
  gz::transport::waitForShutdown();

  return 0;
}