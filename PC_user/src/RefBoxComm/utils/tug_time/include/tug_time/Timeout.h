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

#ifndef TUG_TIME_TIMEOUT_H
#define TUG_TIME_TIMEOUT_H

#include <boost/chrono.hpp>
#include <boost/date_time/time_duration.hpp>
#include <boost/function.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/thread.hpp>

class Timeout
{
public:
  // The clock used for time measurements.
  typedef boost::chrono::high_resolution_clock Clock;

  /// Callback called on expiration.
  ///
  /// If it returns true, the timeout is restarted; if it returns false, the timeout is stopped.
  typedef boost::function<bool()> Callback;

  /// Initializes the timeout without starting it.
  explicit Timeout(const Callback& callback);

  /// Initializes and starts the timeout.
  Timeout(const Clock::duration& timeout, const Callback& callback);

  /// Initializes and starts the timeout.
  Timeout(const boost::posix_time::time_duration& timeout, const Callback& callback);

  /// Initializes and starts the timeout.
  Timeout(double timeout, const Callback& callback);

  virtual ~Timeout();

  /// Sets (starts) or resets the timeout.
  void set();

  /// Stops (cancels) the timeout.
  void stop();

  /// Changes the timeout duration and (re)starts the timeout.
  void setTimeOut(const Clock::duration& timeout);

  /// Changes the timeout duration and (re)starts the timeout.
  void setTimeOut(const boost::posix_time::time_duration& timeout);

  /// Changes the timeout duration and (re)starts the timeout.
  void setTimeOut(double timeout);

protected:
  typedef boost::recursive_mutex Mutex;
  typedef boost::unique_lock<Mutex> Lock;

  /// Thread main function.
  void run();

  Mutex mutex_;
  boost::condition_variable_any condition_;
  Clock::time_point start_time_;
  Clock::duration timeout_;
  Callback callback_;
  bool running_;
  boost::thread thread_;
};

#endif  // TUG_TIME_TIMEOUT_H
