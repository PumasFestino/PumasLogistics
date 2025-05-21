/*
This file is part of the software provided by the tug ais groupe
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge, Alexander Buchegger
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_PLUGIN_MANAGER_PLUGIN_SPEC_H
#define TUG_PLUGIN_MANAGER_PLUGIN_SPEC_H

#include <string>
#include <boost/shared_ptr.hpp>

namespace tug_plugin_manager
{

template <class T>
class PluginSpec
{
public:
  PluginSpec(const std::string& _name, const std::string& _type, boost::shared_ptr<T> _instance)
    : name(_name), type(_type), instance(_instance)
  {
  }

  std::string name;
  std::string type;
  boost::shared_ptr<T> instance;
};

}  // namespace tug_plugin_manager

#endif  // TUG_PLUGIN_MANAGER_PLUGIN_SPEC_H
