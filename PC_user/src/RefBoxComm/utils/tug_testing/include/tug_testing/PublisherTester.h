/*
This file is part of the software provided by the tug ais group
Copyright (c) 2015, Clemens Muehlbacher
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_TESTING_PUBLISHERTESTER_H
#define TUG_TESTING_PUBLISHERTESTER_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <utility>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

template<class T>
class PublisherTester
{
    ros::NodeHandle nh_;
    ros::Subscriber the_sub_;
    ros::AsyncSpinner spinner_;

    boost::mutex the_mutex_;
    bool should_use_subscriber_content_;
    T buffered_content_;
    boost::condition got_message_condition_;
    unsigned int spin_time_;

public:
    explicit PublisherTester(std::string topic_name) : spinner_(2), should_use_subscriber_content_(false),
                                                       spin_time_(500)
    {
      the_sub_ = nh_.subscribe(topic_name, 1, &PublisherTester<T>::SubCB, this);
      spinner_.start();
    }

    virtual ~PublisherTester()
    {
      spinner_.stop();
    }

    void SubCB(const typename T::ConstPtr &msg)
    {
      ROS_DEBUG("got message");
      boost::mutex::scoped_lock the_lock(the_mutex_);
      if (should_use_subscriber_content_)
      {
        ROS_DEBUG("buffer mesage");
        buffered_content_ = *msg;
        got_message_condition_.notify_all();
      }
    }

    std::pair<T, bool> getMessage(boost::function<void()> function_to_call, double time_to_wait)
    {
      boost::mutex::scoped_lock the_lock(the_mutex_);
      should_use_subscriber_content_ = true;
      std::pair<T, bool> result;

      ROS_DEBUG("wait for publisher");
      boost::posix_time::ptime wake_up_time = boost::posix_time::microsec_clock::local_time() +
              boost::posix_time::milliseconds(time_to_wait * 1000.);
      while (the_sub_.getNumPublishers() < 1)
      {
        if (wake_up_time < boost::posix_time::microsec_clock::local_time())
        {
          ROS_DEBUG("timeout to wait for publisher");
          result.second = false;
          return result;
        }
        boost::this_thread::sleep(boost::posix_time::millisec(spin_time_));
      }

      ROS_DEBUG("call function");
      function_to_call();
      ROS_DEBUG("function called");

      boost::posix_time::time_duration timeout = boost::posix_time::milliseconds(time_to_wait * 1000.);
      if (!got_message_condition_.timed_wait(the_lock, timeout))
      {
        ROS_DEBUG("timeout");
        result.second = false;
      }
      else
      {
        ROS_DEBUG("got message - return the message");
        result.first = buffered_content_;
        result.second = true;
      }

      should_use_subscriber_content_ = false;

      return result;
    }
};

#endif  // TUG_TESTING_PUBLISHERTESTER_H
