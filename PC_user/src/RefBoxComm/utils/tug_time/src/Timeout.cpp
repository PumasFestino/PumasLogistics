/*
This file is part of the software provided by the tug ais groupe
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tug_time/Timeout.h>
#include <boost/bind.hpp>
#include <ros/init.h>

Timeout::Timeout(const Callback& callback)
  : timeout_(Clock::duration::zero()), callback_(callback), running_(true), thread_(boost::bind(&Timeout::run, this))
{
}

Timeout::Timeout(const Clock::duration& timeout, const Callback& callback)
  : start_time_(Clock::now()), timeout_(timeout), callback_(callback), running_(true),
    thread_(boost::bind(&Timeout::run, this))
{
}

Timeout::Timeout(const boost::posix_time::time_duration& timeout, const Callback& callback)
  : start_time_(Clock::now()),
    timeout_(boost::chrono::duration_cast<Clock::duration>(boost::chrono::duration<double>(
      static_cast<double>(timeout.ticks()) / static_cast<double>(timeout.ticks_per_second())))),
    callback_(callback), running_(true), thread_(boost::bind(&Timeout::run, this))
{
}

Timeout::Timeout(double timeout, const Callback& callback)
  : start_time_(Clock::now()),
    timeout_(boost::chrono::duration_cast<Clock::duration>(boost::chrono::duration<double>(timeout))),
    callback_(callback), running_(true), thread_(boost::bind(&Timeout::run, this))
{
}

Timeout::~Timeout()
{
  Lock lock(mutex_);
  running_ = false;
  condition_.notify_one();
  lock.unlock();

  thread_.join();
}

void Timeout::run()
{
  Lock lock(mutex_);
  while (ros::ok() && running_)
  {
    if (start_time_ != Clock::time_point() && timeout_ != Clock::duration::zero())
    {
      const Clock::time_point expiration_time(start_time_ + timeout_);
      if (Clock::now() >= expiration_time)
      {
        // Stop timeout before calling the callback; this allows the callback to call set() or setTimeOut():
        start_time_ = Clock::time_point();
        if (callback_())
        {
          // If callback returned true, restart timeout:
          start_time_ = Clock::now();
        }
      }
      else
      {
        condition_.wait_until(lock, expiration_time);
      }
    }
    else
    {
      condition_.wait(lock);
    }
  }
}

void Timeout::set()
{
  Lock lock(mutex_);
  start_time_ = Clock::now();
  condition_.notify_one();
}

void Timeout::stop()
{
  Lock lock(mutex_);
  start_time_ = Clock::time_point();
  condition_.notify_one();
}

void Timeout::setTimeOut(const Clock::duration& timeout)
{
  Lock lock(mutex_);
  timeout_ = timeout;
  start_time_ = Clock::now();
  condition_.notify_one();
}

void Timeout::setTimeOut(const boost::posix_time::time_duration& timeout)
{
  setTimeOut(boost::chrono::duration_cast<Clock::duration>(boost::chrono::duration<double>(
    static_cast<double>(timeout.ticks()) / static_cast<double>(timeout.ticks_per_second()))));
}

void Timeout::setTimeOut(double timeout)
{
  setTimeOut(boost::chrono::duration_cast<Clock::duration>(boost::chrono::duration<double>(timeout)));
}
