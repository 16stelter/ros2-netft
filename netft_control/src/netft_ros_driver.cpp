/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, CRI Lab at Nanyang Technological University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Francisco Suarez Ruiz
   Desc:   ros-control hardware interface layer for netft usign the rdt driver.
*/

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

// ros-control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

// netft_rdt_driver
#include "netft_control/netft_rdt_driver.h"


#define DEFAULT_IP_ADDRESS        "192.168.0.11"


class NetftRosDriver : public hardware_interface::RobotHW
{
public:
  NetftRosDriver() : nh_private_("~")
  {
    // Load configuration parameters
    double rate;
    std::string frame_id, ip_address, topic;
    nh_private_.param("ip", ip_address, std::string(DEFAULT_IP_ADDRESS));
    if (!nh_private_.hasParam("ip"))
      RCLCPP_WARN_STREAM(this->get_logger(), "Parameter [~ip] not found, using default: " << ip_address);
    nh_private_.param("rate", rate, 125.0);
    if (!nh_private_.hasParam("rate"))
      RCLCPP_WARN(this->get_logger(), "Parameter [~rate] not found, using default: %.2f", rate);
    nh_private_.param(std::string("topic"), topic, std::string("raw"));
    if (!nh_private_.hasParam("topic"))
      RCLCPP_WARN_STREAM(this->get_logger(), "Parameter [~topic] not found, using default: " << topic);
    nh_private_.param(std::string("frame_id"), frame_id, std::string("base_link"));
    if (!nh_private_.hasParam("frame_id"))
      RCLCPP_WARN_STREAM(this->get_logger(), "Parameter [~frame_id] not found, using default: " << frame_id);
    // Set initial values
    force_[0] = 0;
    force_[1] = 0;
    force_[2] = 0;
    torque_[0] = 0;
    torque_[1] = 0;
    torque_[2] = 0;
    // Connect the handle with the hardware interface
    hardware_interface::ForceTorqueSensorHandle ft_sensor_handle(topic, frame_id, force_, torque_);
    ft_sensor_interface_.registerHandle(ft_sensor_handle);
    registerInterface(&ft_sensor_interface_);
    RCLCPP_INFO(this->get_logger(), "Registered ForceTorqueSensorInterface");
    // Will publish diagnostics every second
    diag_publisher_.reset(new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(nh_, "diagnostics", 2));
    diag_pub_duration_ = rclcpp::Duration(1.0);
    diag_array_.status.reserve(1);
    // Connect to the netft sensor
    try {
      netft_.reset(new netft_rdt_driver::NetFTRDTDriver(ip_address));
    }
    catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to netft with IP address: %s. Please DOUBLE CHECK the connection with the sensor", ip_address.c_str());
      return;
    }
    // Start the controller manager 
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    rclcpp::Duration update_period = rclcpp::Duration(1.0/rate);
    non_realtime_loop_ = nh_.createTimer(update_period, &NetftRosDriver::update, this);
    RCLCPP_INFO(this->get_logger(), "Loaded netft_ros_driver.");
  }
  
  ~NetftRosDriver()
  {
  }
  
  void update(const rclcpp::TimerEvent& e)
  {
    rclcpp::Duration period = rclcpp::Duration(e.current_real - e.last_real);
    controller_manager_->update(rclcpp::Time::now(), period);
    // Read data and publish diagnostics
    geometry_msgs::WrenchStamped data;
    if (netft_->waitForNewData())
    {
      netft_->getData(data);
      force_[0] =   data.wrench.force.x;
      force_[1] =   data.wrench.force.y;
      force_[2] =   data.wrench.force.z;
      torque_[0] =  data.wrench.torque.x;
      torque_[1] =  data.wrench.torque.y;
      torque_[2] =  data.wrench.torque.z;
    }
    rclcpp::Time current_time(rclcpp::Time::now());
    if ( (current_time - last_diag_pub_time_) > diag_pub_duration_ )
    {
      diag_array_.status.clear();
      netft_->diagnostics(diag_status_);
      diag_array_.status.push_back(diag_status_);
      diag_array_.header.stamp = rclcpp::Time::now();
      if(diag_publisher_->trylock())
      {
        diag_publisher_->msg_.header.stamp = rclcpp::Time::now();
        diag_publisher_->msg_.status = diag_array_.status;
        diag_publisher_->unlockAndPublish();
      }
      last_diag_pub_time_ = current_time;
    }
  }

private:
  // ROS
  rclcpp::NodeHandle   nh_, nh_private_;
  // NetFT
  boost::scoped_ptr<netft_rdt_driver::NetFTRDTDriver>       netft_;
  // Diagnostics
  boost::scoped_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> >  diag_publisher_;
  diagnostic_msgs::DiagnosticArray                          diag_array_;
  diagnostic_updater::DiagnosticStatusWrapper               diag_status_;
  rclcpp::Time                                                 last_diag_pub_time_;
  rclcpp::Duration                                             diag_pub_duration_;
  // Controller Manager
  rclcpp::Timer                                                non_realtime_loop_;
  boost::shared_ptr<controller_manager::ControllerManager>  controller_manager_;
  // Interfaces
  hardware_interface::ForceTorqueSensorInterface            ft_sensor_interface_;
  // Data is read from / written to these internal variables
  double force_[3];
  double torque_[3];
};


int main(int argc, char **argv)
{
  rclcpp::init (argc, argv, "netft_ros_driver");
  rclcpp::NodeHandle nh;
  // Multi-threaded spinning
  rclcpp::MultiThreadedSpinner spinner(4);
  // Start the ros driver
  NetftRosDriver driver;
  spinner.spin();
  return 0;
}
