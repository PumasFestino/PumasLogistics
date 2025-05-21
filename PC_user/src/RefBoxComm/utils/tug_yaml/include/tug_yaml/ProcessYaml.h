/*
This file is part of the software provided by the tug ais groupe
Copyright (c) 2015, Clemens Muehlbacher
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_YAML_PROCESSYAML_H
#define TUG_YAML_PROCESSYAML_H

#include <ros/param.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>
#include <set>
#include <list>
#include <queue>

template<typename _T>
class TypeHelper
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInvalid;
};

template<>
class TypeHelper<bool>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeBoolean;
};

template<>
class TypeHelper<unsigned short>
{
public:
  static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<short>
{
public:
  static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<unsigned int>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<int>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<unsigned long>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<long>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<unsigned long long>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<long long>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeInt;
};

template<>
class TypeHelper<std::string>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeString;
};

template<>
class TypeHelper<float>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeDouble;
};

template<>
class TypeHelper<double>
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeDouble;
};

template<typename _T>
class TypeHelper<std::vector<_T> >
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeArray;
};

template<typename _T>
class TypeHelper<std::set<_T> >
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeArray;
};

template<typename _T>
class TypeHelper<std::list<_T> >
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeArray;
};

template<typename _T>
class TypeHelper<std::queue<_T> >
{
public:
    static const XmlRpc::XmlRpcValue::Type Type = XmlRpc::XmlRpcValue::TypeArray;
};

template<typename _T, typename _R>
class CastHelper
{
public:
    static _T performCast(XmlRpc::XmlRpcValue xml_value)
    {
      return static_cast<_T>(static_cast<_R>(xml_value));
    }
};

template<typename _T>
class CastHelper<_T, unsigned short>
{
public:
  static _T performCast(XmlRpc::XmlRpcValue xml_value)
  {
    return static_cast<_T>(static_cast<int>(xml_value));
  }
};

template<typename _T>
class CastHelper<_T, short>
{
public:
  static _T performCast(XmlRpc::XmlRpcValue xml_value)
  {
    return static_cast<_T>(static_cast<int>(xml_value));
  }
};

template<typename _T>
class CastHelper<_T, unsigned int>
{
public:
    static _T performCast(XmlRpc::XmlRpcValue xml_value)
    {
      return static_cast<_T>(static_cast<int>(xml_value));
    }
};

template<typename _T>
class CastHelper<_T, unsigned long>
{
public:
    static _T performCast(XmlRpc::XmlRpcValue xml_value)
    {
      return static_cast<_T>(static_cast<int>(xml_value));
    }
};

template<typename _T>
class CastHelper<_T, long>
{
public:
    static _T performCast(XmlRpc::XmlRpcValue xml_value)
    {
      return static_cast<_T>(static_cast<int>(xml_value));
    }
};

template<typename _T>
class CastHelper<_T, unsigned long long>
{
public:
    static _T performCast(XmlRpc::XmlRpcValue xml_value)
    {
      return static_cast<_T>(static_cast<int>(xml_value));
    }
};

template<typename _T>
class CastHelper<_T, long long>
{
public:
    static _T performCast(XmlRpc::XmlRpcValue xml_value)
    {
      return static_cast<_T>(static_cast<int>(xml_value));
    }
};

template<typename _T>
class CastHelper<_T, float>
{
public:
    static _T performCast(XmlRpc::XmlRpcValue xml_value)
    {
      return static_cast<_T>(static_cast<double>(xml_value));
    }
};

template<typename _T, typename _R>
class GetValueHelper
{
    static std::string getTypeName(XmlRpc::XmlRpcValue::Type type)
    {
      switch (type)
      {
        case XmlRpc::XmlRpcValue::TypeInvalid:
          return "TypeInvalid";
        case XmlRpc::XmlRpcValue::TypeBoolean:
          return "TypeBoolean";
        case XmlRpc::XmlRpcValue::TypeInt:
          return "TypeInt";
        case XmlRpc::XmlRpcValue::TypeDouble:
          return "TypeDouble";
        case XmlRpc::XmlRpcValue::TypeString:
          return "TypeString";
        case XmlRpc::XmlRpcValue::TypeDateTime:
          return "TypeDateTime";
        case XmlRpc::XmlRpcValue::TypeBase64:
          return "TypeBase64";
        case XmlRpc::XmlRpcValue::TypeArray:
          return "TypeArray";
        case XmlRpc::XmlRpcValue::TypeStruct:
          return "TypeStruct";
        default:
          return "TypeInvalid";
      }
    }

public:
    static _T getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
      if (xml_value.getType() != type)
      {
        ROS_FATAL_STREAM("" << xml_value << " has wrong type should have " << getTypeName(type) << " and has " <<
                         getTypeName(xml_value.getType()));
        std::stringstream error_stream;
        error_stream << "" << xml_value << " has wrong type should have " << getTypeName(type) << " and has " <<
        getTypeName(xml_value.getType());
        throw std::invalid_argument(error_stream.str());
      }

      return CastHelper<_T, _R>::performCast(xml_value);
    }
};

template<typename _T, typename _R>
class GetValueHelper<std::vector<_T>, std::vector<_R> >
{
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
      return GetValueHelper<_T, _R>::getValue(TypeHelper<_T>::Type, xml_value);
    }

public:
    static std::vector<_T> getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
      if (xml_value.getType() != type)
      {
        ROS_FATAL_STREAM(" has wrong type should have " << type << " and has " << xml_value.getType());
        std::stringstream error_stream;
        error_stream << " has wrong type should have " << type << " and has " << xml_value.getType();
        throw std::invalid_argument(error_stream.str());
      }

      std::vector<_T> result;
      for (int i = 0; i < xml_value.size(); ++i)
        result.push_back(getValue(xml_value[i]));

      return result;
    }
};

template<typename _T, typename _R>
class GetValueHelper<std::set<_T>, std::set<_R> >
{
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T, _R>::getValue(TypeHelper<_T>::Type, xml_value);
    }

public:
    static std::set<_T> getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if (xml_value.getType() != type)
        {
            ROS_FATAL_STREAM(" has wrong type should have " << type << " and has " << xml_value.getType());
            std::stringstream error_stream;
            error_stream << " has wrong type should have " << type << " and has " << xml_value.getType();
            throw std::invalid_argument(error_stream.str());
        }

        std::set<_T> result;
        for (int i = 0; i < xml_value.size(); ++i)
            result.insert(getValue(xml_value[i]));

        return result;
    }
};

template<typename _T, typename _R>
class GetValueHelper<std::list<_T>, std::list<_R> >
{
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T, _R>::getValue(TypeHelper<_T>::Type, xml_value);
    }

public:
    static std::list<_T> getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if (xml_value.getType() != type)
        {
            ROS_FATAL_STREAM(" has wrong type should have " << type << " and has " << xml_value.getType());
            std::stringstream error_stream;
            error_stream << " has wrong type should have " << type << " and has " << xml_value.getType();
            throw std::invalid_argument(error_stream.str());
        }

        std::list<_T> result;
        for (int i = 0; i < xml_value.size(); ++i)
            result.push_back(getValue(xml_value[i]));

        return result;
    }
};

template<typename _T, typename _R>
class GetValueHelper<std::queue<_T>, std::queue<_R> >
{
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
        return GetValueHelper<_T, _R>::getValue(TypeHelper<_T>::Type, xml_value);
    }

public:
    static std::queue<_T> getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
        if (xml_value.getType() != type)
        {
            ROS_FATAL_STREAM(" has wrong type should have " << type << " and has " << xml_value.getType());
            std::stringstream error_stream;
            error_stream << " has wrong type should have " << type << " and has " << xml_value.getType();
            throw std::invalid_argument(error_stream.str());
        }

        std::queue<_T> result;
        for (int i = 0; i < xml_value.size(); ++i)
            result.push_back(getValue(xml_value[i]));

        return result;
    }
};

template<>
class GetValueHelper<XmlRpc::XmlRpcValue, XmlRpc::XmlRpcValue>
{
public:
    static XmlRpc::XmlRpcValue getValue(XmlRpc::XmlRpcValue::Type, XmlRpc::XmlRpcValue xml_value)
    {
      return xml_value;
    }
};

class ProcessYaml
{
public:
    static bool hasValue(std::string name, XmlRpc::XmlRpcValue xml_value)
    {
      return xml_value.hasMember(name);
    }

    template<typename _T>
    static _T getValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
      return GetValueHelper<_T, _T>::getValue(type, xml_value);
    }

    template<typename _T>
    static _T getValue(XmlRpc::XmlRpcValue xml_value)
    {
      return getValue(TypeHelper<_T>::Type, xml_value);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
      if (!xml_value.hasMember(name))
      {
        ROS_FATAL_STREAM(name << " is not a member of " << xml_value);
        std::stringstream error_stream;
        error_stream << name << " is not a member of " << xml_value;
        throw std::invalid_argument(error_stream.str());
      }

      return getValue<_T>(type, xml_value[name]);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue xml_value)
    {
      return getValue<_T>(name, TypeHelper<_T>::Type, xml_value);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value,
                       _T default_value)
    {
      if (!xml_value.hasMember(name))
        return default_value;

      return getValue<_T>(name, type, xml_value);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue xml_value, _T default_value)
    {
      return getValue<_T>(name, TypeHelper<_T>::Type, xml_value, default_value);
    }

    static bool hasValue(std::string name, ros::NodeHandle nh)
    {
      return nh.hasParam(name);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue::Type type, ros::NodeHandle nh)
    {
      if (!nh.hasParam(name))
      {
        ROS_FATAL_STREAM(name << " is not a paramter");
        std::stringstream error_stream;
        error_stream << name << " is not a paramter";
        throw std::invalid_argument(error_stream.str());
      }

      XmlRpc::XmlRpcValue value;
      nh.getParam(name, value);

      return getValue<_T>(type, value);
    }

    template<typename _T>
    static _T getValue(std::string name, ros::NodeHandle nh)
    {
      return getValue<_T>(name, TypeHelper<_T>::Type, nh);
    }

    template<typename _T>
    static _T getValue(std::string name, XmlRpc::XmlRpcValue::Type type, ros::NodeHandle nh,
                       _T default_value)
    {
      if (!nh.hasParam(name))
        return default_value;

      return getValue<_T>(name, type, nh);
    }

    template<typename _T>
    static _T getValue(std::string name, ros::NodeHandle nh, _T default_value)
    {
      return getValue<_T>(name, TypeHelper<_T>::Type, nh, default_value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
      return GetValueHelper<_T, _R>::getValue(type, xml_value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(XmlRpc::XmlRpcValue xml_value)
    {
      return getCastedValue<_T, _R>(TypeHelper<_T>::Type, xml_value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value)
    {
      if (!xml_value.hasMember(name))
      {
        ROS_FATAL_STREAM(name << " is not a member of " << xml_value);
        std::stringstream error_stream;
        error_stream << name << " is not a member of " << xml_value;
        throw std::invalid_argument(error_stream.str());
      }

      return getCastedValue<_T, _R>(type, xml_value[name]);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue xml_value)
    {
      return getCastedValue<_T, _R>(name, TypeHelper<_T>::Type, xml_value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue::Type type, XmlRpc::XmlRpcValue xml_value,
                             _T default_value)
    {
      if (!xml_value.hasMember(name))
        return default_value;

      return getCastedValue<_T, _R>(name, type, xml_value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue xml_value, _T default_value)
    {
      return getCastedValue<_T, _R>(name, TypeHelper<_T>::Type, xml_value, default_value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue::Type type, ros::NodeHandle nh)
    {
      if (!nh.hasParam(name))
      {
        ROS_FATAL_STREAM(name << " is not a paramter");
        std::stringstream error_stream;
        error_stream << name << " is not a paramter";
        throw std::invalid_argument(error_stream.str());
      }

      XmlRpc::XmlRpcValue value;
      nh.getParam(name, value);

      return getCastedValue<_T, _R>(type, value);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, ros::NodeHandle nh)
    {
      return getCastedValue<_T, _R>(name, TypeHelper<_T>::Type, nh);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, XmlRpc::XmlRpcValue::Type type, ros::NodeHandle nh,
                             _T default_value)
    {
      if (!nh.hasParam(name))
        return default_value;

      return getCastedValue<_T, _R>(name, type, nh);
    }

    template<typename _T, typename _R>
    static _T getCastedValue(std::string name, ros::NodeHandle nh, _T default_value)
    {
      return getCastedValue<_T, _R>(name, TypeHelper<_T>::Type, nh, default_value);
    }
};

#endif  // TUG_YAML_PROCESSYAML_H
