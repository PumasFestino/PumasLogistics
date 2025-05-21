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
#ifndef TUG_PROFILING_STATISTICS_PRINTER_H
#define TUG_PROFILING_STATISTICS_PRINTER_H

#include <ros/node_handle.h>
#include <ros/timer.h>
#include <tug_cfg/ros_dynamic_config_server.h>
#include <tug_profiling/profiler.h>
#include <tug_profiling/StatisticsPrinterConfig.h>

namespace tug_profiling
{
class StatisticsPrinter
{
public:
  StatisticsPrinter(const ros::NodeHandle& nh = ros::NodeHandle("~"));
  StatisticsPrinter(Profiler& profiler, const ros::NodeHandle& nh = ros::NodeHandle("~"));
  virtual ~StatisticsPrinter();

protected:
  void printStatistics(const ros::TimerEvent&);
  void reconfigure();

  Profiler* profiler_;
  ros::NodeHandle node_handle_;
  ros::Timer timer_;
  StatisticsPrinterConfig config_;
  tug_cfg::RosDynamicConfigServer config_server_;
};
}  // namespace tug_profiling

#endif  // TUG_PROFILING_STATISTICS_PRINTER_H
