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
#ifndef TUG_CFG_LOG_ERROR_HANDLER_H
#define TUG_CFG_LOG_ERROR_HANDLER_H

#include <functional>
#include <ostream>
#include <string>
#include <tug_cfg/error_handler.h>

namespace tug_cfg
{
class LogErrorHandler : public ErrorHandler
{
public:
  typedef std::function<void(const std::string&)> LogFunction;

  explicit LogErrorHandler(const LogFunction& log);

  void handleUnsupportedType(const Key* key, const Object& value, const std::string& description) override;
  void handleUnsupportedKey(const Key* key, const std::string& description) override;
  void handleTypeMismatch(const Key* key, const Object& value, const std::string& other_type,
                          const std::string& description) override;
  void handleViolatedConstraint(const Key* key, const std::string& description) override;
  void handleSuperfluousValue(const Key* key, const Object& value, const std::string& child_key,
                              const std::string& description) override;
  void handleError(const std::string& description) override;

  static ErrorHandlerPtr createRosWarnHandler(const std::string& name);
  static ErrorHandlerPtr createStreamHandler(std::ostream& out, const std::string& prefix);

protected:
  static void logToRosWarn(const std::string& name, const std::string& message);
  static void logToStream(std::ostream& out, const std::string& prefix, const std::string& message);

  LogFunction log_;
};
}  // namespace tug_cfg

#endif  // TUG_CFG_LOG_ERROR_HANDLER_H
