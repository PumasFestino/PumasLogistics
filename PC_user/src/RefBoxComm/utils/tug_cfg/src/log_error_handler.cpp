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
#include <tug_cfg/log_error_handler.h>
#include <functional>
#include <memory>
#include <ostream>
#include <ros/console.h>
#include <string>
#include <tug_cfg/key.h>
#include <tug_cfg/object.h>
#include <tug_cfg/type.h>

namespace tug_cfg
{
LogErrorHandler::LogErrorHandler(const LogFunction& log)
  : log_(log)
{
}

void LogErrorHandler::handleUnsupportedType(const Key* key, const Object& value, const std::string& description)
{
  std::ostringstream s;
  s << key << ": unsupported type " << value.getType().getName() << (description.empty() ? "" : ": ") << description;
  log_(s.str());
}

void LogErrorHandler::handleUnsupportedKey(const Key* key, const std::string& description)
{
  std::ostringstream s;
  s << key << ": unsupported key" << (description.empty() ? "" : ": ") << description;
  log_(s.str());
}

void LogErrorHandler::handleTypeMismatch(const Key* key, const Object& value, const std::string& other_type,
                                         const std::string& description)
{
  std::ostringstream s;
  s << key << ": incompatible types (" << value.getType().getName() << " vs. " << other_type << ")";
  s << (description.empty() ? "" : ": ") << description;
  log_(s.str());
}

void LogErrorHandler::handleViolatedConstraint(const Key* key, const std::string& description)
{
  std::ostringstream s;
  s << key << ": constraint violated" << (description.empty() ? "" : ": ") << description;
  log_(s.str());
}

void LogErrorHandler::handleSuperfluousValue(const Key* key, const Object& /*value*/, const std::string& child_key,
                                             const std::string& description)
{
  std::ostringstream s;
  s << key << ": superfluous value '" << child_key << "'" << (description.empty() ? "" : ": ") << description;
  log_(s.str());
}

void LogErrorHandler::handleError(const std::string& description)
{
  log_(description);
}

ErrorHandlerPtr LogErrorHandler::createRosWarnHandler(const std::string& name)
{
  return std::make_shared<LogErrorHandler>(std::bind(tug_cfg::LogErrorHandler::logToRosWarn, name,
                                                     std::placeholders::_1));
}

ErrorHandlerPtr LogErrorHandler::createStreamHandler(std::ostream& out, const std::string& prefix)
{
  return std::make_shared<LogErrorHandler>(std::bind(tug_cfg::LogErrorHandler::logToStream, std::ref(out), prefix,
                                                     std::placeholders::_1));
}

void LogErrorHandler::logToRosWarn(const std::string& name, const std::string& message)
{
  ROS_WARN_STREAM_NAMED(name, message);
}

void LogErrorHandler::logToStream(std::ostream& out, const std::string& prefix, const std::string& message)
{
  out << prefix << message << std::endl;
}
}  // namespace tug_cfg
