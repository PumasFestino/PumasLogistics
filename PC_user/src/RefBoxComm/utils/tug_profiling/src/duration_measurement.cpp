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
#include <tug_profiling/duration_measurement.h>
#include <tug_profiling/profile.h>
#include <ros/console.h>

namespace tug_profiling
{
const DurationMeasurement::Clock::time_point DurationMeasurement::NEVER;

const DurationMeasurement::Formatter DurationMeasurement::DEFAULT_FORMATTER(
  SimpleDurationFormatter<std::chrono::milliseconds>(5));

DurationMeasurement::DurationMeasurement(Profiler& profiler, const std::string& name,
                                         const Clock::time_point& start_time)
  : DurationMeasurement(profiler, name, DEFAULT_FORMATTER, start_time)
{
}

DurationMeasurement::DurationMeasurement(Profiler& profiler, const std::string& name, const Formatter& formatter,
                                         const Clock::time_point& start_time)
  : profiler_(&profiler), name_(name), formatter_(&formatter), start_time_(start_time)
{
}

DurationMeasurement::~DurationMeasurement()
{
  stop();
}

void DurationMeasurement::start(const Clock::time_point& start_time)
{
  start_time_ = start_time;
}

void DurationMeasurement::stop(const Clock::time_point& stop_time)
{
  if (start_time_ != NEVER)
  {
    commit(stop_time - start_time_);
    start_time_ = NEVER;  // Invalidate measurement
  }
}

void DurationMeasurement::commit(const Clock::duration& measurement)
{
  ProfilePtr& profile = profiler_->getProfile(name_);
  if (!profile)
  {
    profile = std::make_shared<Statistics<Clock::duration::rep>>(*formatter_);
  }

  typedef MeasurementAccumulator<Clock::duration::rep> DurationMA;
  std::shared_ptr<DurationMA> ma = std::dynamic_pointer_cast<DurationMA>(profile);
  if (ma)
  {
    ma->accumulate(measurement.count());
  }
  else
  {
    ROS_WARN_NAMED("tug_profiling.DurationMeasurement", "DurationMeasurement::stop: measurement types do not match");
  }
}

template<>
const char* SimpleDurationFormatter<std::chrono::hours>::getDurationAcronym()
{
  return "h";
}

template<>
const char* SimpleDurationFormatter<std::chrono::minutes>::getDurationAcronym()
{
  return "m";
}

template<>
const char* SimpleDurationFormatter<std::chrono::seconds>::getDurationAcronym()
{
  return "s";
}

template<>
const char* SimpleDurationFormatter<std::chrono::milliseconds>::getDurationAcronym()
{
  return "ms";
}

template<>
const char* SimpleDurationFormatter<std::chrono::microseconds>::getDurationAcronym()
{
  return "us";
}

template<>
const char* SimpleDurationFormatter<std::chrono::nanoseconds>::getDurationAcronym()
{
  return "ns";
}
}  // namespace tug_profiling
