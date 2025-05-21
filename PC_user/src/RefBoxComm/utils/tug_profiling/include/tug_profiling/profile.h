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
#ifndef TUG_PROFILING_PROFILE_H
#define TUG_PROFILING_PROFILE_H

#include <boost/format.hpp>
#include <functional>
#include <iomanip>
#include <ostream>
#include <string>

namespace tug_profiling
{
class Profile
{
public:
  Profile() = default;
  virtual ~Profile() = default;

  virtual void print(std::ostream& out) const = 0;
};

template <typename T>
class MeasurementAccumulator : public Profile
{
public:
  virtual void accumulate(const T& value) = 0;
};

template <typename T>
class Statistics : public MeasurementAccumulator<T>
{
public:
  typedef std::function<void(std::ostream&, const T& value)> Formatter;

  Statistics(const Formatter& formatter)
    : formatter_(formatter)
  {
  }

  void print(std::ostream& out) const override
  {
    if (count_ <= 0)
    {
      out << "no calculations performed" << std::endl;
    }
    else
    {
      out << "performed " << std::setw(6) << count_ << "x, min: ";
      formatter_(out, min_);
      out << ", avg: ";
      formatter_(out, getAverage());
      out << ", max: ";
      formatter_(out, max_);
      out << std::endl;
    }
  }

  void accumulate(const T& value) override
  {
    ++count_;
    sum_ += value;
    if (max_ < value)
    {
      max_ = value;
    }
    if (min_ > value)
    {
      min_ = value;
    }
  }

  T getAverage() const
  {
    return sum_ / static_cast<T>(count_);
  }

protected:
  Formatter formatter_;
  std::size_t count_ = 0;
  T sum_ = 0;
  T min_ = std::numeric_limits<T>::max();
  T max_ = std::numeric_limits<T>::lowest();
};
}  // namespace tug_profiling

#endif  // TUG_PROFILING_PROFILE_H
