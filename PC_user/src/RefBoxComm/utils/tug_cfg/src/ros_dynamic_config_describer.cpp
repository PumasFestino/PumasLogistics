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
#include <tug_cfg/ros_dynamic_config_describer.h>
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <string>
#include <tug_cfg/error_handler.h>
#include <tug_cfg/key.h>
#include <tug_cfg/scalar.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace tug_cfg
{
RosDynamicConfigDescriber::RosDynamicConfigDescriber(dynamic_reconfigure::ConfigDescription* description)
  : description_(description)
{
  dynamic_reconfigure::Group group;
  group.name = "Default";
  group.type = "";
  group.parent = 0;
  group.id = 0;
  description_->groups.push_back(group);
  top_group_ = &description_->groups.back();
}

RosDynamicConfigDescriber::~RosDynamicConfigDescriber()
{
}

void RosDynamicConfigDescriber::visit(const Key* key, const AbstractMap& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ErrorHandler::get()->handleUnsupportedType(key, value, "dynamic maps are not supported");
  }
}

void RosDynamicConfigDescriber::visit(const Key* key, const AbstractSequence& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ErrorHandler::get()->handleUnsupportedType(key, value, "dynamic sequences are not supported");
  }
}

void RosDynamicConfigDescriber::visit(const Key* key, const AbstractStruct& value)
{
  if (key == nullptr)
  {
    top_group_ = &description_->groups.at(0);
  }
  else
  {
    dynamic_reconfigure::Group group;
    std::ostringstream name;
    name << key;
    group.name = name.str();
    group.type = "collapse";
    group.parent = top_group_->id;
    group.id = description_->groups.size();
    description_->groups.push_back(group);
    top_group_ = &description_->groups.back();
  }

  value.acceptElements(key, *this);

  top_group_ = &description_->groups.at(top_group_->parent);
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<bool>& /*value*/)
{
  addParam<bool>(key, "bool", &dynamic_reconfigure::Config::bools);
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<double>& /*value*/)
{
  addParam<double>(key, "double", &dynamic_reconfigure::Config::doubles);
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<int>& /*value*/)
{
  addParam<int>(key, "int", &dynamic_reconfigure::Config::ints);
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<std::string>& /*value*/)
{
  addParam<std::string>(key, "str", &dynamic_reconfigure::Config::strs);
}

void RosDynamicConfigDescriber::visit(const Key* key, const Object& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ErrorHandler::get()->handleUnsupportedType(key, value, "");
  }
}

dynamic_reconfigure::ParamDescription* RosDynamicConfigDescriber::createParam(
    const Key* key, const AbstractStruct::Field* field,
    const std::string& type_name)
{
  dynamic_reconfigure::ParamDescription param_desc;
  std::ostringstream name;
  name << key;
  param_desc.name = name.str();
  param_desc.type = type_name;
  param_desc.level = field->level;
  param_desc.description = field->description;
  if (!field->unit.empty())
  {
    param_desc.description += " (in " + field->unit + ")";
  }
  top_group_->parameters.push_back(param_desc);
  return &top_group_->parameters.back();
}

template <typename T, typename P>
void RosDynamicConfigDescriber::addParam(
    const Key* key, const std::string& type_name,
    std::vector<P> dynamic_reconfigure::Config::* p)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    const AbstractStruct::ScalarField<T>& scalar_field(
          dynamic_cast<const AbstractStruct::ScalarField<T>&>(*field));
    dynamic_reconfigure::ParamDescription* param_desc = createParam(key, field,
                                                                    type_name);
    P param;
    param.name = param_desc->name;
    param.value = scalar_field.default_value;
    (description_->dflt.*p).push_back(param);
    param.value = scalar_field.min;
    (description_->min.*p).push_back(param);
    param.value = scalar_field.max;
    (description_->max.*p).push_back(param);
    if (!scalar_field.choices.empty())
    {
      YAML::Emitter e;
      e.SetSeqFormat(YAML::Flow);
      e.SetMapFormat(YAML::Flow);
      e.SetStringFormat(YAML::SingleQuoted);
      e << YAML::BeginMap;
      e << YAML::Key << "enum_description" << YAML::Value << "";
      e << YAML::Key << "enum" << YAML::Value << YAML::BeginSeq;
      for (const T& choice : scalar_field.choices)
      {
        e << YAML::BeginMap;
        e << YAML::Key << "name" << YAML::Value << boost::lexical_cast<std::string>(choice);
        e << YAML::Key << "value" << YAML::Value << choice;
        e << YAML::Key << "description" << YAML::Value << "";
        e << YAML::Key << "type" << YAML::Value << type_name;
        e << YAML::EndMap;
      }
      e << YAML::EndSeq;
      e << YAML::EndMap;
      param_desc->edit_method = e.c_str();
    }
  }
}
}  // namespace tug_cfg
