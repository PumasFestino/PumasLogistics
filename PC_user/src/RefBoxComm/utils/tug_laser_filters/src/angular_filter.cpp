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

#include "tug_laser_filters/angular_filter.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <limits>
#include <tug_yaml/ProcessYaml.h>

#define BETWEEN(x, x_min, x_max) ((x) > (x_min) && (x) < (x_max))
#define BETWEEN2(x, x0, x1) (BETWEEN((x), (x0), (x1)) || BETWEEN((x), (x1), (x0)))

namespace laser_filters
{
  laser_filters::LaserScanAngularFilter::LaserScanAngularFilter() :
          up_and_running_(false)
  {
  }

  bool LaserScanAngularFilter::configure()
  {
    up_and_running_ = true;

    XmlRpc::XmlRpcValue params;
    bool ignored_areas_set = getParam("angles_to_remove", params);
    if (!ignored_areas_set)
    {
      ROS_ERROR("ignored_areas are not given!");
      return ignored_areas_set;
    }

    for (int i = 0; i < params.size(); ++i)
    {
      AngularRange range;
      range.first = ProcessYaml::getValue<double>("lower_angle", params[i]);
      range.second = ProcessYaml::getValue<double>("upper_angle", params[i]);

      ignored_areas_.push_back(range);
    }
    return true;
  }

  bool LaserScanAngularFilter::update(const sensor_msgs::LaserScan &input_scan,
                                      sensor_msgs::LaserScan &output_scan)
  {
    output_scan = input_scan;
    float current_angle = input_scan.angle_min;
    for (int index = 0; index < input_scan.ranges.size(); ++index)
    {
      if (inForbiddenArea(current_angle))
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();

      current_angle += input_scan.angle_increment;
    }

    return true;
  }

  bool LaserScanAngularFilter::inForbiddenArea(float angle)
  {
    for (AngularRangeList::iterator it = ignored_areas_.begin();
         it != ignored_areas_.end(); ++it)
      if (BETWEEN2(angle, it->first, it->second))
        return true;

    return false;
  }
};  // namespace laser_filters
PLUGINLIB_EXPORT_CLASS(laser_filters::LaserScanAngularFilter,
                       filters::FilterBase<sensor_msgs::LaserScan>)
