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
#include <tug_cfg/default_constrainer.h>
#include <set>
#include <sstream>
#include <string>
#include <tug_cfg/collection.h>
#include <tug_cfg/error_handler.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/struct.h>

namespace tug_cfg
{
void DefaultConstrainer::visit(Key* key, AbstractMap& value)
{
  value.acceptElements(key, *this);
}

void DefaultConstrainer::visit(Key* key, Scalar<bool>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, Scalar<double>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, Scalar<int>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, Scalar<std::string>& value)
{
  constrainScalar(key, value);
}

void DefaultConstrainer::visit(Key* key, AbstractStruct& value)
{
  value.acceptElements(key, *this);
}

void DefaultConstrainer::visit(Key* key, AbstractSequence& value)
{
  value.acceptElements(key, *this);
}

void DefaultConstrainer::visit(Key* key, Object& value)
{
  ErrorHandler::get()->handleUnsupportedType(key, value, "");
}

template <typename T>
void DefaultConstrainer::constrainScalar(Key* key, Scalar<T>& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr)
  {
    const AbstractStruct::ScalarField<T>& scalar_field = dynamic_cast<const AbstractStruct::ScalarField<T>&>(*field);
    enforceMin<T>(key, value, scalar_field.min);
    enforceMax<T>(key, value, scalar_field.max);
    enforceChoices<T>(key, value, scalar_field.choices, scalar_field.default_value);
  }
}

template <typename T>
void DefaultConstrainer::enforceMin(Key* key, T& value, const T& min_value)
{
  if (value < min_value)
  {
    std::ostringstream s;
    s << "value is below specified minimum (" << value << " < " << min_value << ")";
    ErrorHandler::get()->handleViolatedConstraint(key, s.str());
    value = min_value;
  }
}

template <>
void DefaultConstrainer::enforceMin<std::string>(Key* /*key*/, std::string& /*value*/, const std::string& /*min_value*/)
{
  // String has no meaningful minimum, so we just ignore this.
}

template <typename T>
void DefaultConstrainer::enforceMax(Key* key, T& value, const T& max_value)
{
  if (value > max_value)
  {
    std::ostringstream s;
    s << "value is above specified maximum (" << value << " > " << max_value << ")";
    ErrorHandler::get()->handleViolatedConstraint(key, s.str());
    value = max_value;
  }
}

template <>
void DefaultConstrainer::enforceMax<std::string>(Key* /*key*/, std::string& /*value*/, const std::string& /*max_value*/)
{
  // String has no meaningful maximum, so we just ignore this.
}

template <typename T>
void DefaultConstrainer::enforceChoices(Key* key, T& value, const std::set<T>& choices, const T& default_value)
{
  if (!choices.empty() && choices.find(value) == choices.end())
  {
    std::ostringstream s;
    s << "value is not in set of valid choices (" << value << " not in (";
    typename std::set<T>::const_iterator it = choices.begin();
    if (it != choices.end())
    {
      s << *it;
      while (++it != choices.end())
      {
        s << ", " << *it;
      }
    }
    s << "))";
    ErrorHandler::get()->handleViolatedConstraint(key, s.str());
    value = default_value;
  }
}
}  // namespace tug_cfg
