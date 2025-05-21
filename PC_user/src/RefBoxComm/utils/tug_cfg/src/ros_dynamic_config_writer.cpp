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
#include <tug_cfg/ros_dynamic_config_writer.h>
#include <string>
#include <tug_cfg/collection.h>
#include <tug_cfg/error_handler.h>
#include <tug_cfg/struct.h>
#include <vector>

namespace tug_cfg
{
RosDynamicConfigWriter::RosDynamicConfigWriter(dynamic_reconfigure::Config* config)
  : config_(config)
{
}

RosDynamicConfigWriter::~RosDynamicConfigWriter()
{
}

void RosDynamicConfigWriter::visit(const Key* key, const AbstractMap& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ErrorHandler::get()->handleUnsupportedType(key, value, "dynamic maps are not supported");
  }
}

void RosDynamicConfigWriter::visit(const Key* key, const AbstractSequence& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ErrorHandler::get()->handleUnsupportedType(key, value, "dynamic sequences are not supported");
  }
}

void RosDynamicConfigWriter::visit(const Key* key, const AbstractStruct& value)
{
  value.acceptElements(key, *this);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<bool>& value)
{
  visitScalar(key, value, config_->bools);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<double>& value)
{
  visitScalar(key, value, config_->doubles);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<int>& value)
{
  visitScalar(key, value, config_->ints);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<std::string>& value)
{
  visitScalar(key, value, config_->strs);
}

void RosDynamicConfigWriter::visit(const Key* key, const Object& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ErrorHandler::get()->handleUnsupportedType(key, value, "");
  }
}

template <typename T, typename P>
void RosDynamicConfigWriter::visitScalar(const Key* key, const Scalar<T>& value,
                                         std::vector<P>& values)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    std::ostringstream name;
    name << key;
    P p;
    p.name = name.str();
    p.value = static_cast<const typename P::_value_type&>(
          static_cast<const T&>(value));
    values.push_back(p);
  }
}
}  // namespace tug_cfg
