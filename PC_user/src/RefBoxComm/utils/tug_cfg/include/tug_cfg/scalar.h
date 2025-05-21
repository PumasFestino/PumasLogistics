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
#ifndef TUG_CFG_SCALAR_H
#define TUG_CFG_SCALAR_H

#include <string>
#include <tug_cfg/object.h>
#include <tug_cfg/type.h>
#include <tug_cfg/visitor.h>
#include <typeinfo>

namespace tug_cfg
{
class AbstractScalar : public Object
{
};

template <typename T>
class Scalar : public AbstractScalar
{
public:
  typedef T Value;

  class Type : public tug_cfg::Type
  {
  public:
    std::string getName() const override
    {
      return typeid(Value).name();
    }
  };

  inline explicit Scalar(Value& value)
    : value_(value)
  {
  }

  inline operator Value&()
  {
    return value_;
  }

  inline operator const Value&() const
  {
    return value_;
  }

  inline Scalar<T>& operator=(const T& value)
  {
    value_ = value;
    return *this;
  }

  void accept(Key* key, Visitor& visitor) override
  {
    visitor.visit(key, *this);
  }

  void accept(const Key* key, ConstVisitor& visitor) const override
  {
    visitor.visit(key, *this);
  }

  const Type& getType() const override
  {
    static Type type;
    return type;
  }

protected:
  Value& value_;
};
}  // namespace tug_cfg

#endif  // TUG_CFG_SCALAR_H
