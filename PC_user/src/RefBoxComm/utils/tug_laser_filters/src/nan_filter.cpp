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

#include "tug_laser_filters/nan_filter.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <limits>


namespace laser_filters
{
laser_filters::LaserScanNANFilter::LaserScanNANFilter()
  :
  up_and_running_(false)
{
}

bool LaserScanNANFilter::configure()
{
  up_and_running_ = true;
  return true;
}

bool LaserScanNANFilter::update(const sensor_msgs::LaserScan& input_scan,
                                sensor_msgs::LaserScan& output_scan)
{
  output_scan = input_scan;
  for (int index = 0; index < output_scan.ranges.size(); ++index)
  {
    if (std::isnan(output_scan.ranges[index]))
    {
      output_scan.ranges[index] = output_scan.range_max + 1.0;
    }
  }

  return true;
}

};  // namespace laser_filters
PLUGINLIB_EXPORT_CLASS(laser_filters::LaserScanNANFilter,
                       filters::FilterBase < sensor_msgs::LaserScan >)
