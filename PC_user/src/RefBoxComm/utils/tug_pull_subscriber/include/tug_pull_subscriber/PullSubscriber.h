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
#ifndef TUG_PULL_SUBSCRIBER_PULLSUBSCRIBER_H
#define TUG_PULL_SUBSCRIBER_PULLSUBSCRIBER_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <string>

template <class T>
class PullSubscriber
{
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;

    ros::Subscriber the_subscriber_;

    boost::mutex subscriber_mutex_;
    boost::condition_variable subscriber_condition_;
    bool got_message_;
    bool needs_new_message_;
    T msg_;

    void theCB(const T& msg)
    {
      ROS_DEBUG_STREAM("got the call back");
      boost::mutex::scoped_lock the_lock(subscriber_mutex_);

      if (!got_message_ || needs_new_message_)
      {
        ROS_DEBUG_STREAM("saveing the message");
        msg_ = msg;
        got_message_ = true;
        needs_new_message_ = false;
        subscriber_condition_.notify_all();
      }
    }

public:
    PullSubscriber(ros::NodeHandle nh, std::string topic) : nh_(nh), got_message_(false), needs_new_message_(false),
                                                            spinner_(1)
    {
      boost::mutex::scoped_lock the_lock(subscriber_mutex_);
      the_subscriber_ = nh_.subscribe(topic, 1, &PullSubscriber::theCB, this);
      spinner_.start();
    }

    void waitForPublisher()
    {
      while (the_subscriber_.getNumPublishers() < 1)
        ros::Duration(1.).sleep();
    }

    T getLastMsg()
    {
      boost::mutex::scoped_lock the_lock(subscriber_mutex_);

      while (!got_message_)
        subscriber_condition_.wait(the_lock);

      return msg_;
    }

    T getNewMsg()
    {
      boost::mutex::scoped_lock the_lock(subscriber_mutex_);
      needs_new_message_ = true;

      while (needs_new_message_)
        subscriber_condition_.wait(the_lock);

      return msg_;
    }
};

#endif  // TUG_PULL_SUBSCRIBER_PULLSUBSCRIBER_H
