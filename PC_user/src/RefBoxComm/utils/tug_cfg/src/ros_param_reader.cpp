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
#include <tug_cfg/ros_param_reader.h>
#include <boost/lexical_cast.hpp>
#include <map>
#include <sstream>
#include <string>
#include <tug_cfg/collection.h>
#include <tug_cfg/error_handler.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>
#include <utility>

namespace tug_cfg
{
RosParamReader::RosParamReader(const ros::NodeHandle& node_handle, const std::string& key)
  : top_(nullptr)
{
  if (!node_handle.getParam(key, root_node_))
  {
    ErrorHandler::get()->handleError("ROS parameter server has no parameter named " + node_handle.getNamespace() + "/"
                                     + key);
  }
}

RosParamReader::RosParamReader(const XmlRpc::XmlRpcValue& value)
  : top_(nullptr), root_node_(value)
{
}

RosParamReader::~RosParamReader()
{
}

void RosParamReader::visit(Key* key, AbstractMap& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      for (; context.it != context.node->end(); ++context.it)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getXmlRpcTypeName(context.node->getType()), "");
    }
  }
}

void RosParamReader::visit(Key* key, AbstractSequence& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < context.node->size(); ++i)
      {
        value.acceptNewElement(key, *this);
      }
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getXmlRpcTypeName(context.node->getType()), "");
    }
  }
}

void RosParamReader::visit(Key* key, AbstractStruct& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      for (const XmlRpc::XmlRpcValue::ValueStruct::value_type& v : *context.node)
      {
        context.remaining_keys.insert(v.first);
      }
      value.acceptElements(key, *this);
      for (const std::string& remaining_key : context.remaining_keys)
      {
        ErrorHandler::get()->handleSuperfluousValue(key, value, remaining_key, "");
      }
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getXmlRpcTypeName(context.node->getType()), "");
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<bool>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeBoolean)
    {
      value = *context.node;
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getXmlRpcTypeName(context.node->getType()), "");
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<double>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      value = *context.node;
    }
    else if (context.node->getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = static_cast<int>(*context.node);
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getXmlRpcTypeName(context.node->getType()), "");
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<int>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = *context.node;
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getXmlRpcTypeName(context.node->getType()), "");
    }
  }
}

void RosParamReader::visit(Key* key, Scalar<std::string>& value)
{
  Context context(this);
  if (context.enter(key))
  {
    if (context.node->getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      value = *context.node;
    }
    else
    {
      ErrorHandler::get()->handleTypeMismatch(key, value, getXmlRpcTypeName(context.node->getType()), "");
    }
  }
}

void RosParamReader::visit(Key* key, Object& value)
{
  ErrorHandler::get()->handleUnsupportedType(key, value, "");
}

const std::string& RosParamReader::getXmlRpcTypeName(XmlRpc::XmlRpcValue::Type type)
{
  typedef std::map<XmlRpc::XmlRpcValue::Type, std::string> Names;

  static std::string unknown_type_name = "Unknown";
  static Names names;

  if (names.empty())
  {
#define X(x) names.insert(std::make_pair(XmlRpc::XmlRpcValue::Type ## x, #x))
    X(Invalid);
    X(Boolean);
    X(Int);
    X(Double);
    X(String);
    X(DateTime);
    X(Base64);
    X(Array);
    X(Struct);
#undef X
  }

  Names::iterator it = names.find(type);
  if (it != names.end())
  {
    return it->second;
  }
  return unknown_type_name;
}




RosParamReader::Context::Context(RosParamReader* reader_)
  : reader(reader_), parent(reader_->top_), node(nullptr)
{
  reader->top_ = this;
}

RosParamReader::Context::~Context()
{
  reader->top_ = parent;
}

bool RosParamReader::Context::enter(Key* key)
{
  // std::cerr << "Entering " << key << std::endl;
  if (key == nullptr)
  {
    if (parent != nullptr)
    {
      throw std::logic_error("Null key");
    }
    node = &reader->root_node_;
  }
  else if (parent == nullptr)
  {
    throw std::logic_error("First key must be null");
  }
  else if (parent->node->getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    int* key_number = key->asPtr<int>();
    if (key_number != nullptr)
    {
      if (0 <= *key_number && *key_number < parent->node->size())
      {
        node = &(*parent->node)[*key_number];
      }
    }
    else
    {
      ErrorHandler::get()->handleUnsupportedKey(key, "sequence key is not int");
    }
  }
  else if (parent->node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
    if (field != nullptr)
    {
      if (parent->node->hasMember(field->name))
      {
        node = &(*parent->node)[field->name];
        parent->remaining_keys.erase(field->name);
      }
    }
    else
    {
      std::string* key_name = key->asPtr<std::string>();
      if (key_name != nullptr)
      {
        *key_name = parent->it->first;
      }
      else
      {
        int* key_number = key->asPtr<int>();
        if (key_number != nullptr)
        {
          *key_number = boost::lexical_cast<int>(parent->it->first);
        }
        else
        {
          ErrorHandler::get()->handleUnsupportedKey(key, "map key is neither field, nor string, nor int");
        }
      }
      node = &parent->it->second;
    }
  }

  if (node != nullptr)
  {
    if (node->getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      it = node->begin();
    }
    return node->valid();
  }
  return false;
}
}  // namespace tug_cfg
