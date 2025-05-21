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
#include <tug_profiling/statistics_printer.h>
#include <tug_cfg/ros_param_reader.h>
#include <tug_cfg/configuration.h>
#include <functional>
#include <iostream>

namespace tug_profiling
{
StatisticsPrinter::StatisticsPrinter(const ros::NodeHandle& nh)
  : StatisticsPrinter(Profiler::getRootInstance(), nh)
{
}

StatisticsPrinter::StatisticsPrinter(Profiler& profiler, const ros::NodeHandle& nh)
  : profiler_(&profiler), node_handle_(nh, "profiling"), config_server_(config_)
{
  tug_cfg::RosParamReader reader(node_handle_);
  tug_cfg::load(config_, reader);
  reconfigure();
  config_server_.setCallback(std::bind(&StatisticsPrinter::reconfigure, this));
  config_server_.start(node_handle_);
}

StatisticsPrinter::~StatisticsPrinter()
{
}

void StatisticsPrinter::printStatistics(const ros::TimerEvent&)
{
  profiler_->printStatistics(std::cout);
  profiler_->clear();
}

void StatisticsPrinter::reconfigure()
{
  if (config_.print_statistics)
  {
    timer_ = node_handle_.createTimer(ros::Duration(config_.print_statistics_interval),
                                      &StatisticsPrinter::printStatistics, this);
  }
  else if (timer_)
  {
    timer_ = ros::Timer();
  }
}
}  // namespace tug_profiling
