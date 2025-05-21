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

#ifndef LTA_LASER_FILTER_ANGULAR_FILTER_H
#define LTA_LASER_FILTER_ANGULAR_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <utility>

namespace laser_filters
{
class LaserScanAngularFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  typedef std::pair<double, double> AngularRange;
  typedef std::vector<AngularRange> AngularRangeList;

public:
  LaserScanAngularFilter();

  bool configure();

  bool update(const sensor_msgs::LaserScan &input_scan,
              sensor_msgs::LaserScan &filtered_scan);

private:
  bool inForbiddenArea(float angle);

  bool up_and_running_;

  AngularRangeList ignored_areas_;
};
}  // namespace laser_filters

#endif  // LTA_LASER_FILTER_ANGULAR_FILTER_H
