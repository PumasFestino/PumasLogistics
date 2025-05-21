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
#ifndef TUG_CFG_STRUCT_H
#define TUG_CFG_STRUCT_H

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <tug_cfg/collection.h>
#include <tug_cfg/key.h>
#include <tug_cfg/type.h>
#include <vector>

namespace tug_cfg
{
class AbstractStruct : public Collection
{
public:
  class Field
  {
  public:
    Field(const std::string& name_, const std::string& unit_,
          const std::string& description_, bool dynamic_, int level_,
          bool ignored_);
    virtual ~Field() = default;

    std::string name;
    std::string unit;
    std::string description;
    bool dynamic;
    int level;
    bool ignored;
  };



  template <typename T>
  class ScalarField : public Field
  {
  public:
    ScalarField(const std::string& name_, const std::string& unit_,
                const std::string& description_, bool dynamic_, int level_,
                bool ignored_, const T& default_value_, const T& min_,
                const T& max_, const std::set<T>& choices_,
                const std::set<T>& suggestions_)
      : Field(name_, unit_, description_, dynamic_, level_, ignored_),
        default_value(default_value_), min(min_), max(max_), choices(choices_),
        suggestions(suggestions_)
    {
    }

    T default_value;
    T min;
    T max;
    const std::set<T> choices;
    const std::set<T> suggestions;
  };



  template <typename MetaT>
  struct FieldTypes
  {
    typedef Field Type;
  };

  template <typename T>
  struct FieldTypes<Scalar<T>>
  {
    typedef ScalarField<T> Type;
  };



  void accept(Key* key, Visitor& visitor) override;
  void accept(const Key* key, ConstVisitor& visitor) const override;
};



template <>
void ScalarKey<const AbstractStruct::Field*>::format(std::ostream& s) const;



template <typename C>
class Struct : public AbstractStruct
{
public:
  typedef C Value;



  class FieldImplBase
  {
  public:
    virtual ~FieldImplBase() = default;

    virtual void accept(C* instance, const Key* parent_key, Visitor& visitor) const = 0;
    virtual void accept(C* instance, const Key* parent_key, ConstVisitor& visitor) const = 0;
  };



  typedef const std::vector<const FieldImplBase*> Fields;



  template <typename MetaT, typename FieldT = typename FieldTypes<MetaT>::Type>
  class FieldImpl : public FieldImplBase
  {
  public:
    FieldImpl(typename MetaT::Value C::* var, const FieldT& field)
      : var_(var), field_(field)
    {
    }

    void accept(C* instance, const Key* parent_key, Visitor& visitor) const override
    {
      ScalarKey<const Field*> key(parent_key, &field_);
      MetaT(instance->*var_).accept(&key, visitor);
    }

    void accept(C* instance, const Key* parent_key, ConstVisitor& visitor) const override
    {
      ScalarKey<const Field*> key(parent_key, &field_);
      MetaT(instance->*var_).accept(&key, visitor);
    }

  protected:
    typename MetaT::Value C::* var_;
    FieldT field_;
  };



  class Type : public tug_cfg::Type
  {
  public:
    Type(const std::string& name, const Fields& fields)
      : name_(name), fields_(fields)
    {
    }

    std::string getName() const override
    {
      return name_;
    }

  protected:
    friend class Struct<C>;

    const std::string name_;
    const Fields fields_;
  };



  explicit Struct(Value& value)
    : value_(&value)
  {
  }

  const Type& getType() const override
  {
    return Value::getDefinition();
  }

  void acceptElements(const Key* parent_key, Visitor& visitor) override
  {
    for (const FieldImplBase* field : Value::getDefinition().fields_)
    {
      field->accept(value_, parent_key, visitor);
    }
  }

  void acceptElements(const Key* parent_key, ConstVisitor& visitor) const override
  {
    for (const FieldImplBase* field : Value::getDefinition().fields_)
    {
      field->accept(value_, parent_key, visitor);
    }
  }

protected:
  Value* value_;
};
}  // namespace tug_cfg

#endif  // TUG_CFG_STRUCT_H
