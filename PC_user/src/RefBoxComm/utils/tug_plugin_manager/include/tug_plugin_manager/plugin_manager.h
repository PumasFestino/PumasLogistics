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

#ifndef TUG_PLUGIN_MANAGER_PLUGIN_MANAGER_H
#define TUG_PLUGIN_MANAGER_PLUGIN_MANAGER_H

#include <vector>
#include <pluginlib/class_loader.h>
#include <ros/console.h>

#include <tug_plugin_manager/plugin_spec.h>
#include <tug_plugin_manager/plugin_manager_exceptions.h>
#include <string>

namespace tug_plugin_manager
{
template <class T>
class PluginManager
{
public:
  typedef boost::shared_ptr<T> PluginPtr;
  typedef PluginSpec<T> PluginSpecT;
  typedef std::vector<PluginSpecT> PluginList;
  typedef typename PluginList::iterator PluginListIterator;

  PluginManager(const std::string& package, const std::string& base_class)
    : plugin_loader_(package, base_class)
  {
  }

  virtual ~PluginManager()
  {
  }

  PluginPtr loadPlugin(const std::string& name, const std::string& type)
  {
    ROS_INFO("Will load plugin '%s' of type '%s'", name.c_str(), type.c_str());

    PluginPtr plugin;

    if (getPluginInstanceByName(name))
      throw PluginAlreadyInListException(
          "The plugin '" + name + "' was already loaded inside the plugin manager");

    plugin = plugin_loader_.createInstance(type);

    if (!plugin)
      throw PluginCannotBeCreatedException("Could not create object of plugin '" + name + "'");

    plugin_list_.push_back(PluginSpecT(name, type, plugin));

    //  ROS_INFO("Now %d plugin(s) is/are loaded!", (int)plugin_list_.size() );

    return plugin;
  }

  /**
   * Removes the named plugin from the list.
   *
   * Note that this will only unload the plugin if there are no other shared
   * pointers to the plugin.
   *
   * Does nothing if there is no plugin with the given name.
   *
   * \return true if the plugin existed, false otherwise.
   */
  bool removePlugin(const std::string& name)
  {
    for (PluginListIterator it = plugin_list_.begin(); it != plugin_list_.end(); ++it)
    {
      if (it->name == name)
      {
        plugin_list_.erase(it);
        return true;
      }
    }

    return false;
  }

  PluginPtr getPluginInstanceByName(const std::string& name) const
  {
    for (size_t i = 0; i < plugin_list_.size(); ++i)
      if (plugin_list_[i].name == name)
        return plugin_list_[i].instance;

    return PluginPtr();
  }

  std::vector<std::string> getPluginNames() const
  {
    std::vector<std::string> names_list;
    for (size_t i = 0; i < plugin_list_.size(); ++i)
      names_list.push_back(plugin_list_[i].name);

    return names_list;
  }

  const PluginList& getPluginList() const
  {
    return plugin_list_;
  }

protected:
  pluginlib::ClassLoader<T> plugin_loader_;
  PluginList plugin_list_;
};

}  // namespace tug_plugin_manager

#endif  // TUG_PLUGIN_MANAGER_PLUGIN_MANAGER_H
