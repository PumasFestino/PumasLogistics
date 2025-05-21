/*
 * This file is part of the software provided by the Graz University of Technology AIS group.
 *
 * Copyright (c) 2017, Alexander Buchegger
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted  provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TUG_PROFILING_DURATION_MEASUREMENT_H
#define TUG_PROFILING_DURATION_MEASUREMENT_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <ostream>
#include <string>
#include <tug_profiling/profiler.h>

namespace tug_profiling
{
class DurationMeasurement
{
public:
  typedef std::chrono::steady_clock Clock;
  typedef std::function<void(std::ostream&, const Clock::duration::rep&)> Formatter;

  static const Clock::time_point NEVER;
  static const Formatter DEFAULT_FORMATTER;

  DurationMeasurement(Profiler& profiler, const std::string& name, const Clock::time_point& start_time = Clock::now());
  DurationMeasurement(Profiler& profiler, const std::string& name, const Formatter& formatter,
                      const Clock::time_point& start_time = Clock::now());
  ~DurationMeasurement();

  void start(const Clock::time_point& start_time = Clock::now());
  void stop(const Clock::time_point& stop_time = Clock::now());

protected:
  void commit(const Clock::duration& measurement);

  Profiler* profiler_ = nullptr;
  std::string name_;
  const Formatter* formatter_;
  Clock::time_point start_time_;
};

template <typename DurationType>
class SimpleDurationFormatter
{
public:
  SimpleDurationFormatter(int width = 0)
    : width_(width)
  {
  }

  void operator()(std::ostream& out, const DurationMeasurement::Clock::duration::rep& d) const
  {
    out << std::setw(width_)
        << std::chrono::duration_cast<DurationType>(DurationMeasurement::Clock::duration(d)).count()
        << getDurationAcronym();
  }

  static const char* getDurationAcronym();

protected:
  int width_;
};
}  // namespace tug_profiling

#endif  // TUG_PROFILING_DURATION_MEASUREMENT_H
