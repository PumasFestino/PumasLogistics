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
#include <tug_cfg/yaml_reader.h>
#include <exception>
#include <map>
#include <set>
#include <string>
#include <tug_cfg/collection.h>
#include <tug_cfg/error_handler.h>
#include <tug_cfg/key.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>
#include <yaml-cpp/yaml.h>

namespace tug_cfg
{
class YamlReader::Context
{
public:
  explicit Context(YamlReader* reader);
  ~Context();

  bool enter(Key* key);

  YAML::Node node;
  YAML::const_iterator it;
  std::set<std::string> remaining_keys;

protected:
  YamlReader* reader_;
  Context* parent_;
};



class YamlReader::RootNode
{
public:
  explicit RootNode(const YAML::Node& node_);

  YAML::Node node;
};



const std::string& getYamlNodeTypeName(YAML::NodeType::value type)
{
  typedef std::map<YAML::NodeType::value, std::string> Names;

  static std::string unknown_type_name = "Unknown";
  static Names names;

  if (names.empty())
  {
#define X(x) names.insert(std::make_pair(YAML::NodeType::x, #x))
    X(Undefined);
    X(Null);
    X(Scalar);
    X(Sequence);
    X(Map);
#undef X
  }

  Names::iterator it = names.find(type);
  if (it != names.end())
  {
    return it->second;
  }
  return unknown_type_name;
}



YamlReader::YamlReader(const std::string& file_path)
  : top_(nullptr)
{
  for (const YAML::Node& document : YAML::LoadAllFromFile(file_path))
  {
    root_nodes_.push(std::make_shared<RootNode>(document));
  }
}

void YamlReader::visit(Key* key, AbstractMap& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsMap())
    {
      for (; context.it != context.node.end(); ++context.it)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getYamlNodeTypeName(context.node.Type()), "");
    }
  }
}

void YamlReader::visit(Key* key, AbstractSequence& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsSequence())
    {
      for (; context.it != context.node.end(); ++context.it)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getYamlNodeTypeName(context.node.Type()), "");
    }
  }
}

void YamlReader::visit(Key* key, AbstractStruct& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsMap())
    {
      for (; context.it != context.node.end(); ++context.it)
      {
        context.remaining_keys.insert(context.it->first.as<std::string>());
      }
      value.acceptElements(key, *this);
      for (const std::string& remaining_key : context.remaining_keys)
      {
        ErrorHandler::get()->handleSuperfluousValue(key, value, remaining_key, "");
      }
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getYamlNodeTypeName(context.node.Type()), "");
    }
  }
}

void YamlReader::visit(Key* key, Scalar<bool>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<bool>();
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getYamlNodeTypeName(context.node.Type()), "");
    }
  }
}

void YamlReader::visit(Key* key, Scalar<double>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<double>();
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getYamlNodeTypeName(context.node.Type()), "");
    }
  }
}

void YamlReader::visit(Key* key, Scalar<int>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<int>();
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getYamlNodeTypeName(context.node.Type()), "");
    }
  }
}

void YamlReader::visit(Key* key, Scalar<std::string>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node.IsScalar())
    {
      value = context.node.as<std::string>();
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getYamlNodeTypeName(context.node.Type()), "");
    }
  }
}

void YamlReader::visit(Key* key, Object& value)
{
  ErrorHandler::get()->handleUnsupportedType(key, value, "");
}



YamlReader::Context::Context(YamlReader* reader)
  : reader_(reader), parent_(reader->top_)
{
  reader_->top_ = this;
}

YamlReader::Context::~Context()
{
  reader_->top_ = parent_;
}

bool YamlReader::Context::enter(Key* key)
{
  // std::cerr << "Entering " << key << std::endl;
  if (key == nullptr)
  {
    if (parent_ != nullptr)
    {
      throw std::logic_error("Null key");
    }
    if (!reader_->root_nodes_.empty())
    {
      node = reader_->root_nodes_.front()->node;
      reader_->root_nodes_.pop();
    }
  }
  else if (parent_ == nullptr)
  {
    throw std::logic_error("First key must be null");
  }
  else if (parent_->node.IsSequence())
  {
    node = static_cast<const YAML::Node&>(*parent_->it);
  }
  else if (parent_->node.IsMap())
  {
    const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
    if (field != nullptr)
    {
      node = parent_->node[field->name];
      parent_->remaining_keys.erase(field->name);
    }
    else
    {
      std::string* key_name = key->asPtr<std::string>();
      if (key_name != nullptr)
      {
        *key_name = parent_->it->first.as<std::string>();
      }
      else
      {
        int* key_number = key->asPtr<int>();
        if (key_number != nullptr)
        {
          *key_number = parent_->it->first.as<int>();
        }
        else
        {
          ErrorHandler::get()->handleUnsupportedKey(key, "map key is neither field, nor string, nor int");
        }
      }
      node = parent_->it->second;
    }
  }
  it = node.begin();
  return node;
}

YamlReader::RootNode::RootNode(const YAML::Node& node_)
  : node(node_)
{
}
}  // namespace tug_cfg
