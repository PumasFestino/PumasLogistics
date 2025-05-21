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
#ifndef TUG_PROFILING_PROFILER_H
#define TUG_PROFILING_PROFILER_H

#include <map>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>

namespace tug_profiling
{
class Profile;
typedef std::shared_ptr<Profile> ProfilePtr;

class Profiler
{
public:
  Profiler(const std::string& name);
  Profiler(Profiler& parent, const std::string& name);
  virtual ~Profiler();

  static Profiler& getRootInstance();

  void printStatistics(std::ostream& out, const unsigned indent = 0);
  ProfilePtr& getProfile(const std::string& name);
  void clear();
  bool hasChildren();

protected:
  typedef std::mutex Mutex;
  typedef std::unique_lock<Mutex> Lock;

  Profiler();
  Profiler(const Profiler&) = delete;
  Profiler& operator=(const Profiler&) = delete;

  void addChild(Profiler* child);
  void removeChild(Profiler* child);

  Mutex mutex_;
  Profiler* parent_ = nullptr;
  std::string name_;
  std::vector<Profiler*> children_;
  std::map<std::string, ProfilePtr> profiles_;
};
}  // namespace tug_profiling

#endif  // TUG_PROFILING_PROFILER_H
