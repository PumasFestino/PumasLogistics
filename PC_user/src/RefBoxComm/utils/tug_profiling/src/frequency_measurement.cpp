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
#include <tug_profiling/frequency_measurement.h>
#include <ros/console.h>
#include <tug_profiling/simple_formatter.h>

namespace tug_profiling
{
const FrequencyMeasurement::Formatter FrequencyMeasurement::DEFAULT_FORMATTER(SimpleFormatter<double>("Hz", 5, 1));

FrequencyMeasurement::FrequencyMeasurement(Profiler& profiler, const std::string& name, const Clock::time_point& time)
  : FrequencyMeasurement(profiler, name, DEFAULT_FORMATTER, time)
{
}

FrequencyMeasurement::FrequencyMeasurement(Profiler& profiler, const std::string& name, const Formatter& formatter,
                                           const Clock::time_point& time)
{
  ProfilePtr& profile = profiler.getProfile(name);
  if (!profile)
  {
    profile = std::make_shared<FrequencyStatistics>(formatter);
  }

  std::shared_ptr<FrequencyStatistics> fs = std::dynamic_pointer_cast<FrequencyStatistics>(profile);
  if (fs)
  {
    if (fs->last_time_ != Clock::time_point())
    {
      fs->accumulate(1.0 / std::chrono::duration_cast<std::chrono::duration<double>>(time - fs->last_time_).count());
    }
    fs->last_time_ = time;
  }
  else
  {
    ROS_WARN_NAMED("tug_profiling.FrequencyMeasurement", "FrequencyMeasurement: measurement types do not match");
  }
}

FrequencyStatistics::FrequencyStatistics(const Formatter& formatter)
  : Statistics(formatter)
{
}
}  // namespace tug_profiling
