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
#ifndef TUG_CFG_ROS_DYNAMIC_CONFIG_WRITER_H
#define TUG_CFG_ROS_DYNAMIC_CONFIG_WRITER_H

#include <dynamic_reconfigure/Config.h>
#include <string>
#include <tug_cfg/scalar.h>
#include <tug_cfg/visitor.h>
#include <vector>

namespace tug_cfg
{
class RosDynamicConfigWriter : public ConstVisitor
{
public:
  explicit RosDynamicConfigWriter(dynamic_reconfigure::Config* config);
  ~RosDynamicConfigWriter() override;

  void visit(const Key* key, const AbstractMap& value) override;
  void visit(const Key* key, const AbstractSequence& value) override;
  void visit(const Key* key, const AbstractStruct& value) override;
  void visit(const Key* key, const Scalar<bool>& value) override;
  void visit(const Key* key, const Scalar<double>& value) override;
  void visit(const Key* key, const Scalar<int>& value) override;
  void visit(const Key* key, const Scalar<std::string>& value) override;
  void visit(const Key* key, const Object& value) override;

protected:
  template <typename T, typename P>
  void visitScalar(const Key* key, const Scalar<T>& value,
                   std::vector<P>& values);

  dynamic_reconfigure::Config* config_;
};
}  // namespace tug_cfg

#endif  // TUG_CFG_ROS_DYNAMIC_CONFIG_WRITER_H
