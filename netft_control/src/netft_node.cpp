/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/** 
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "netft_control/netft_rdt_driver.h"

namespace po = boost::program_options;
using namespace std;
using namespace std::placeholders;

namespace netft {
  class NetftNode : public rclcpp::Node {
  public:

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;

    NetftNode(int argc, char *argv[]) :
      Node("netft_node"),
      driver_(""){

      string address;

      po::options_description desc("Options");
      desc.add_options()
        ("help", "display help")
        ("rate", po::value<double>(&pub_rate_hz_)->default_value(100.0), "set publish rate (in hertz)")
        ("wrench", "publish older Wrench message type instead of WrenchStamped")
        ("address", po::value<string>(&address), "IP address of NetFT box")
        ("frame_id", po::value<string>(&frame_id_)->default_value("base_link"), "Frame ID for Wrench data");

      po::positional_options_description p;
      p.add("address", 1);

      po::variables_map vm;
      po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
      po::notify(vm);

      if (vm.count("help")) {
        cout << desc << endl;
        //usage(progname);
        exit(EXIT_SUCCESS);
      }

      if (!vm.count("address")) {
        cout << desc << endl;
        cerr << "Please specify address of NetFT" << endl;
        exit(EXIT_FAILURE);
      }

      if (vm.count("wrench")) {
        RCLCPP_ERROR(this->get_logger(), "Publishing NetFT data as geometry_msgs::msg::Wrench is deprecated");
      }

      pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("netft_data", 100);

      geometry_msgs::msg::WrenchStamped data;

      diag_pub_duration_ = 1.0s;
      diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 2);
      diag_array_.status.reserve(1);
      last_diag_pub_time_ = std::chrono::system_clock::now();

      rclcpp::Service<std_srvs::srv::Empty>::SharedPtr bias_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/Bias_sensor", std::bind(&NetftNode::bias_srv_cb, this, _1, _2));
      driver_ = netft_rdt_driver::NetFTRDTDriver(address);
      this->loop();
    }

    bool bias_srv_cb(const shared_ptr <std_srvs::srv::Empty::Request> req,
                                shared_ptr <std_srvs::srv::Empty::Response> rsp) {
      return driver_.biasSensor();
    }

    void loop() {

      rclcpp::Rate pub_rate = rclcpp::Rate(pub_rate_hz_);
      while (rclcpp::ok()) {
        if (driver_.waitForNewData()) {
          driver_.getData(data);

          data.header.frame_id = frame_id_;
          pub_->publish(data);
        }

        std::chrono::system_clock::time_point current_time(std::chrono::system_clock::now());
        if ((current_time - last_diag_pub_time_) > diag_pub_duration_) {
          diag_array_.status.clear();
          driver_.diagnostics(diag_status_);
          diag_array_.status.push_back(diag_status_);
          diag_pub_->publish(diag_array_);
          last_diag_pub_time_ = current_time;
        }

        rclcpp::spin_some(this->get_node_base_interface());
        pub_rate.sleep();
      }

    }

  private:
    netft_rdt_driver::NetFTRDTDriver driver_;
    string frame_id_;
    double pub_rate_hz_;

    diagnostic_msgs::msg::DiagnosticArray diag_array_;
    diagnostic_updater::DiagnosticStatusWrapper diag_status_;
    std::chrono::system_clock::time_point last_diag_pub_time_;
    std::chrono::duration<double> diag_pub_duration_;
  };
}
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = netft::NetftNode(argc, argv);
  rclcpp::shutdown();
  return 0;
}