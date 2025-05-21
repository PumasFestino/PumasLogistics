/*
This file is part of the long term autonomy software for robots
Copyright (c) 2016, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <filters/filter_chain.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include <tug_testing/PublisherTester.h>
#include <tug_yaml/ProcessYaml.h>
#include <vector>
#include <utility>
#include <boost/date_time/posix_time/posix_time.hpp>

class TestHelperLaser : public ::testing::Test
{
  ros::NodeHandle nh;
  PublisherTester<sensor_msgs::LaserScan> pub_tester_;
  ros::Publisher publisher_;
  ros::AsyncSpinner spinner_;
  unsigned int spin_time_;

protected:
  TestHelperLaser() : nh("~"), pub_tester_("/scan_filtered"), spinner_(1), spin_time_(500)
  {
    publisher_ = nh.advertise<sensor_msgs::LaserScan>("/scan_raw", 1);
    spinner_.start();

    required_laser_range_values_ = ProcessYaml::getValue<std::vector<float> >("required_laser_range_values", nh);
    send_laser_range_values_ = ProcessYaml::getValue<std::vector<float> >("send_laser_range_values", nh);
  }

  virtual ~TestHelperLaser()
  {
    spinner_.stop();
  }

  std::pair<sensor_msgs::LaserScan, bool> getMessage(boost::function<void()> function_to_call,
                                                     double time_to_wait)
  {
    return pub_tester_.getMessage(function_to_call, time_to_wait);
  }

public:
  std::vector<float> required_laser_range_values_;
  std::vector<float> send_laser_range_values_;

  void sendLaserTestHelperFunction(double time_to_wait)
  {
    ROS_DEBUG("wait for subscriber");
    boost::posix_time::ptime wake_up_time =
            boost::posix_time::microsec_clock::local_time() + boost::posix_time::milliseconds(time_to_wait * 1000.);

    while (publisher_.getNumSubscribers() < 1)
    {
      if (wake_up_time < boost::posix_time::microsec_clock::local_time())
        throw std::runtime_error("timeout while waiting for subscribe");

      boost::this_thread::sleep(boost::posix_time::millisec(spin_time_));
    }
    ROS_DEBUG("publish fake laser msg");

    sensor_msgs::LaserScan msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser";
    msg.angle_min = -.5;
    msg.angle_max = .5;
    msg.angle_increment = 0.1;
    msg.time_increment = 0.1;
    msg.scan_time = 0.1;
    msg.range_min = 0.5;
    msg.range_max = 1.5;
    msg.ranges = send_laser_range_values_;
    msg.intensities = send_laser_range_values_;

    publisher_.publish(msg);
  }
};

void test1HelperFunction()
{ }

TEST_F(TestHelperLaser, send_laser_test1)
{
  std::pair<sensor_msgs::LaserScan, bool> msg_pair = getMessage(&test1HelperFunction, 1);
  EXPECT_FALSE(msg_pair.second);
}


void expect_ranges_eq(const std::vector<float> &a, const std::vector<float> &b)
{
  for (int i = 0; i < 10; i++)
    if (isnan(a[i]))
    {
      EXPECT_TRUE(isnan(b[i]));
    }
    else
    {
      EXPECT_NEAR(a[i], b[i], 1e-6);
    }
}

TEST_F(TestHelperLaser, send_laser_test2)
{
  std::pair<sensor_msgs::LaserScan, bool> msg_pair = getMessage(
          boost::bind(&TestHelperLaser::sendLaserTestHelperFunction, this, 2.0), 1.0);

  EXPECT_TRUE(msg_pair.second);
  expect_ranges_eq(msg_pair.first.ranges, required_laser_range_values_);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_laser_angular_filter");
  return RUN_ALL_TESTS();
}
