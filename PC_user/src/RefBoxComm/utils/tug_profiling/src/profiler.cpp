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
#include <tug_profiling/profiler.h>
#include <algorithm>
#include <iomanip>
#include <ros/this_node.h>
#include <tug_profiling/profile.h>

static const std::string HR(79, '-');

namespace tug_profiling
{
Profiler::Profiler()
{
}

Profiler::Profiler(const std::string& name)
  : Profiler(getRootInstance(), name)
{
}

Profiler::Profiler(Profiler& parent, const std::string& name)
  : parent_(&parent), name_(name)
{
  parent.addChild(this);
}

Profiler::~Profiler()
{
  Lock lock(mutex_);
  if (parent_ != nullptr)
  {
    parent_->removeChild(this);
  }
  for (Profiler* child : children_)
  {
    Lock child_lock(child->mutex_);
    child->parent_ = nullptr;
  }
}

Profiler& Profiler::getRootInstance()
{
  // This is thread-safe according to paragraph 6.7 [stmt.dcl] p4:
  static Profiler instance;
  return instance;
}

void Profiler::printStatistics(std::ostream& out, const unsigned indent)
{
  Lock lock(mutex_);
  if (parent_ == nullptr && indent <= 0)
  {
    out << HR << std::endl;
    out << "Profiling statistics of " << ros::this_node::getName() << " node:" << std::endl;
  }
  else
  {
    out << std::setw(indent + 2) << std::right << "- " << name_ << ":" << std::endl;
  }
  for (const auto& profile : profiles_)
  {
    out << std::setw(indent + 2 + 2) << std::right << "- " << profile.first
        << std::setw(30 - std::min(30, static_cast<int>(profile.first.size())) + 2) << std::left << ": " << std::right;
    profile.second->print(out);
  }
  for (Profiler* child : children_)
  {
    child->printStatistics(out, indent + 2);
  }
  if (parent_ == nullptr)
  {
    out << HR << std::endl;
  }
}

void Profiler::clear()
{
  Lock lock(mutex_);
  profiles_.clear();
  for (Profiler* child : children_)
  {
    child->clear();
  }
}

bool Profiler::hasChildren()
{
  Lock lock(mutex_);
  return !children_.empty();
}

ProfilePtr& Profiler::getProfile(const std::string& name)
{
  Lock lock(mutex_);
  return profiles_[name];
}

void Profiler::addChild(Profiler* child)
{
  Lock lock(mutex_);
  children_.push_back(child);
}

void Profiler::removeChild(Profiler* child)
{
  Lock lock(mutex_);
  children_.erase(std::remove(children_.begin(), children_.end(), child), children_.end());
}
}  // namespace tug_profiling
