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
#ifndef TUG_PROFILING_SIMPLE_FORMATTER_H
#define TUG_PROFILING_SIMPLE_FORMATTER_H

#include <iomanip>
#include <iostream>
#include <ratio>
#include <string>

namespace tug_profiling
{
template <typename Ratio>
const char* getRatioAcronym();

template <typename T, typename Ratio = std::ratio<1>>
class SimpleFormatter
{
public:
  typedef T ValueType;
  typedef Ratio RatioType;

  SimpleFormatter(const std::string& unit, const int width, const int precision = 0)
    : unit_(unit), width_(width), precision_(precision)
  {
  }

  void operator()(std::ostream& out, const T& value) const
  {
    out << std::fixed;  // Fixed-point notation for floating-point values
    out << std::setw(width_) << std::setprecision(precision_) << value << getRatioAcronym<Ratio>() << unit_;
  }

protected:
  std::string unit_;
  int width_;
  int precision_;
};
}  // namespace tug_profiling

#endif  // TUG_PROFILING_SIMPLE_FORMATTER_H
