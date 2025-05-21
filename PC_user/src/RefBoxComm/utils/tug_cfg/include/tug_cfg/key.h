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
#ifndef TUG_CFG_KEY_H
#define TUG_CFG_KEY_H

#include <ostream>

namespace tug_cfg
{
template <typename T> class ScalarKey;

class Key
{
public:
  explicit Key(const Key* parent);
  virtual ~Key() = default;

  const Key* getParent() const;

  virtual void format(std::ostream& s) const = 0;
  void formatPath(std::ostream& s) const;

  /**
   * \throw std::bad_cast
   */
  template <typename T>
  inline T& as()
  {
    return dynamic_cast<ScalarKey<T>&>(*this);
  }

  /**
   * \throw std::bad_cast
   */
  template <typename T>
  inline const T& as() const
  {
    return dynamic_cast<const ScalarKey<T>&>(*this);
  }

  template <typename T>
  T* asPtr()
  {
    ScalarKey<T>* result = dynamic_cast<ScalarKey<T>*>(this);
    if (result != nullptr)
    {
      return &result->get();
    }
    ScalarKey<T*>* result2 = dynamic_cast<ScalarKey<T*>*>(this);
    if (result2 != nullptr)
    {
      return *result2;
    }
    return nullptr;
  }

  template <typename T>
  const T* asPtr() const
  {
    const ScalarKey<T>* result = dynamic_cast<const ScalarKey<T>*>(this);
    if (result != nullptr)
    {
      return &result->get();
    }
    const ScalarKey<T*>* result2 = dynamic_cast<const ScalarKey<T*>*>(this);
    if (result2 != nullptr)
    {
      return *result2;
    }
    return nullptr;
  }

  template <typename T>
  inline bool is() const
  {
    return dynamic_cast<const ScalarKey<T>*>(this) != nullptr;
  }

protected:
  const Key* const parent_;
};



template <typename T>
class ScalarKey : public Key
{
public:
  inline ScalarKey(const Key* parent, const T& key)
    : Key(parent), key_(key)
  {
  }

  void format(std::ostream& s) const override
  {
    s << '[' << key_ << ']';
  }

  T& get()
  {
    return key_;
  }

  const T& get() const
  {
    return key_;
  }

  inline operator T&()
  {
    return key_;
  }

  inline operator const T&() const
  {
    return key_;
  }

protected:
  T key_;
};



std::ostream& operator<<(std::ostream& s, const Key& key);
std::ostream& operator<<(std::ostream& s, const Key* key);
}  // namespace tug_cfg

#endif  // TUG_CFG_KEY_H
