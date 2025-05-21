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
#ifndef TUG_CFG_COLLECTION_H
#define TUG_CFG_COLLECTION_H

#include <map>
#include <string>
#include <tug_cfg/key.h>
#include <tug_cfg/object.h>
#include <tug_cfg/type.h>
#include <typeinfo>
#include <vector>

namespace tug_cfg
{
class Collection : public Object
{
public:
  virtual void acceptElements(const Key* parent_key, Visitor& visitor) = 0;
  virtual void acceptElements(const Key* parent_key, ConstVisitor& visitor) const = 0;
};



class ExtensibleCollection : public Collection
{
public:
  virtual void acceptNewElement(const Key* parent_key, Visitor& visitor) = 0;
};



class AbstractSequence : public ExtensibleCollection
{
public:
  void accept(Key* key, Visitor& visitor) override;
  void accept(const Key* key, ConstVisitor& visitor) const override;
};



template <typename ElementMetaT>
class Sequence : public AbstractSequence
{
public:
  typedef std::vector<typename ElementMetaT::Value> Value;

  class Type : public tug_cfg::Type
  {
    std::string getName() const override
    {
      typename ElementMetaT::Value value;
      return ElementMetaT(value).getType().getName() + "[]";
    }
  };

  explicit Sequence(Value& elements)
    : elements_(elements)
  {
  }

  const tug_cfg::Type& getType() const override
  {
    static Type type;
    return type;
  }

  void acceptElements(const Key* parent_key, Visitor& visitor) override
  {
    for (std::size_t i = 0; i < elements_.size(); ++i)
    {
      ScalarKey<int> key(parent_key, static_cast<int>(i));
      ElementMetaT(elements_[i]).accept(&key, visitor);
    }
  }

  void acceptElements(const Key* parent_key, ConstVisitor& visitor) const override
  {
    for (std::size_t i = 0; i < elements_.size(); ++i)
    {
      ScalarKey<int> key(parent_key, static_cast<int>(i));
      ElementMetaT(elements_[i]).accept(&key, visitor);
    }
  }

  void acceptNewElement(const Key* parent_key, Visitor& visitor) override
  {
    ScalarKey<int> key(parent_key, static_cast<int>(elements_.size()));
    typename Value::value_type value;
    ElementMetaT(value).accept(&key, visitor);
    elements_.insert(elements_.begin() + static_cast<int>(key), value);
  }

protected:
  Value& elements_;
};



class AbstractMap : public ExtensibleCollection
{
public:
  void accept(Key* key, Visitor& visitor) override;
  void accept(const Key* key, ConstVisitor& visitor) const override;
};



template <typename K, typename ElementMetaT>
class Map : public AbstractMap
{
public:
  typedef std::map<K, typename ElementMetaT::Value> Value;

  class Type : public tug_cfg::Type
  {
  public:
    std::string getName() const override
    {
      typename ElementMetaT::Value value;
      return ElementMetaT(value).getType().getName() + "[" + typeid(K).name()
          + "]";
    }
  };

  explicit Map(Value& elements)
    : elements_(elements)
  {
  }

  const tug_cfg::Type& getType() const override
  {
    static Type type;
    return type;
  }

  void acceptElements(const Key* parent_key, Visitor& visitor) override
  {
    for (typename Value::value_type& element : elements_)
    {
      ScalarKey<K> key(parent_key, element.first);
      ElementMetaT(element.second).accept(&key, visitor);
    }
  }

  void acceptElements(const Key* parent_key, ConstVisitor& visitor) const override
  {
    for (typename Value::value_type& element : elements_)
    {
      ScalarKey<K> key(parent_key, element.first);
      ElementMetaT(element.second).accept(&key, visitor);
    }
  }

  void acceptNewElement(const Key* parent_key, Visitor& visitor) override
  {
    ScalarKey<K> key(parent_key, K());
    typename Value::mapped_type value;
    ElementMetaT(value).accept(&key, visitor);
    elements_[key] = value;
  }

protected:
  Value& elements_;
};
}  // namespace tug_cfg

#endif  // TUG_CFG_COLLECTION_H
