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
#ifndef TUG_CFG_ROS_DYNAMIC_CONFIG_SERVER_H
#define TUG_CFG_ROS_DYNAMIC_CONFIG_SERVER_H

#include <dynamic_reconfigure/Reconfigure.h>
#include <functional>
#include <ros/node_handle.h>
#include <tug_cfg/struct.h>

namespace tug_cfg
{
class RosDynamicConfigServer
{
public:
  RosDynamicConfigServer();
  explicit RosDynamicConfigServer(AbstractStruct& configuration);
  virtual ~RosDynamicConfigServer();

  void setConfiguration(AbstractStruct& configuration);
  void setCallback(const std::function<void()>& callback);
  void setLock(const std::function<void()>& lock, const std::function<void()>& unlock);

  template <typename Lockable>
  void setLock(Lockable& lockable)
  {
    setLock(std::bind(&Lockable::lock, &lockable),
            std::bind(&Lockable::unlock, &lockable));
  }

  void start(ros::NodeHandle node_handle = ros::NodeHandle("~"));
  void stop();

  /**
   * Publishes current state of configuration. Used to notify e.g. GUI of changes.
   */
  void notify();

protected:
  class ScopedLock
  {
  public:
    explicit ScopedLock(const RosDynamicConfigServer* server);
    ~ScopedLock();

  protected:
    const RosDynamicConfigServer* server_;
  };
  friend class ScopedLock;

  RosDynamicConfigServer(const RosDynamicConfigServer&) = delete;
  RosDynamicConfigServer& operator=(const RosDynamicConfigServer&) = delete;

  bool reconfigure(dynamic_reconfigure::ReconfigureRequest& req,
                   dynamic_reconfigure::ReconfigureResponse& res);

  tug_cfg::AbstractStruct* configuration_;
  ros::ServiceServer srv_;
  ros::Publisher descriptions_pub_;
  ros::Publisher updates_pub_;
  std::function<void()> callback_;
  std::function<void()> lock_;
  std::function<void()> unlock_;
  bool reconfiguring_;
};
}  // namespace tug_cfg

#endif  // TUG_CFG_ROS_DYNAMIC_CONFIG_SERVER_H
