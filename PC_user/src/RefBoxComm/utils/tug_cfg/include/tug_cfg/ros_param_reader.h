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
#ifndef TUG_CFG_ROS_PARAM_READER_H
#define TUG_CFG_ROS_PARAM_READER_H

#include <ros/node_handle.h>
#include <set>
#include <string>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class RosParamReader : public Visitor
{
public:
  explicit RosParamReader(const ros::NodeHandle& node_handle, const std::string& key = std::string());
  explicit RosParamReader(const XmlRpc::XmlRpcValue& value);
  ~RosParamReader() override;

  void visit(Key* key, AbstractMap& value) override;
  void visit(Key* key, AbstractSequence& value) override;
  void visit(Key* key, AbstractStruct& value) override;
  void visit(Key* key, Scalar<bool>& value) override;
  void visit(Key* key, Scalar<double>& value) override;
  void visit(Key* key, Scalar<int>& value) override;
  void visit(Key* key, Scalar<std::string>& value) override;
  void visit(Key* key, Object& value) override;

  static const std::string& getXmlRpcTypeName(XmlRpc::XmlRpcValue::Type type);
protected:
  class Context
  {
  public:
    explicit Context(RosParamReader* reader_);
    ~Context();

    bool enter(Key* key);

    RosParamReader* reader;
    Context* parent;
    XmlRpc::XmlRpcValue* node;
    XmlRpc::XmlRpcValue::iterator it;
    std::set<std::string> remaining_keys;
  };

  friend class Context;

  Context* top_;
  XmlRpc::XmlRpcValue root_node_;
};
}  // namespace tug_cfg

#endif  // TUG_CFG_ROS_PARAM_READER_H
