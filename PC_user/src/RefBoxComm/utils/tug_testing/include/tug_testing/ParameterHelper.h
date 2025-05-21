/*
This file is part of the software provided by the tug ais group
Copyright (c) 2015, Clemens Muehlbacher
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_TESTING_PARAMETERHELPER_H
#define TUG_TESTING_PARAMETERHELPER_H

#include <set>
#include <string>
#include <XmlRpcValue.h>
#include <vector>
#include <map>

class ParameterHelper
{
    std::set<std::string> used_parameters_;
    std::string tmp_param_prefix_;

    std::string createTmpParamName(const std::string &key);

public:
    ParameterHelper();

    explicit ParameterHelper(std::string tmp_param_prefix);

    virtual ~ParameterHelper();

    void setTmpParameter(const std::string &key, const XmlRpc::XmlRpcValue &v);

    void setTmpParameter(const std::string &key, const std::string &s);

    void setTmpParameter(const std::string &key, const char *s);

    void setTmpParameter(const std::string &key, double d);

    void setTmpParameter(const std::string &key, int i);

    void setTmpParameter(const std::string &key, bool b);

    void setTmpParameter(const std::string &key, const std::vector<std::string> &vec);

    void setTmpParameter(const std::string &key, const std::vector<double> &vec);

    void setTmpParameter(const std::string &key, const std::vector<float> &vec);

    void setTmpParameter(const std::string &key, const std::vector<int> &vec);

    void setTmpParameter(const std::string &key, const std::vector<bool> &vec);

    void setTmpParameter(const std::string &key, const std::map<std::string, std::string> &map);

    void setTmpParameter(const std::string &key, const std::map<std::string, double> &map);

    void setTmpParameter(const std::string &key, const std::map<std::string, float> &map);

    void setTmpParameter(const std::string &key, const std::map<std::string, int> &map);

    void setTmpParameter(const std::string &key, const std::map<std::string, bool> &map);

    XmlRpc::XmlRpcValue getTmpParams();

    void setParameter(const std::string &key, const XmlRpc::XmlRpcValue &v);

    void setParameter(const std::string &key, const std::string &s);

    void setParameter(const std::string &key, const char *s);

    void setParameter(const std::string &key, double d);

    void setParameter(const std::string &key, int i);

    void setParameter(const std::string &key, bool b);

    void setParameter(const std::string &key, const std::vector<std::string> &vec);

    void setParameter(const std::string &key, const std::vector<double> &vec);

    void setParameter(const std::string &key, const std::vector<float> &vec);

    void setParameter(const std::string &key, const std::vector<int> &vec);

    void setParameter(const std::string &key, const std::vector<bool> &vec);

    void setParameter(const std::string &key, const std::map<std::string, std::string> &map);

    void setParameter(const std::string &key, const std::map<std::string, double> &map);

    void setParameter(const std::string &key, const std::map<std::string, float> &map);

    void setParameter(const std::string &key, const std::map<std::string, int> &map);

    void setParameter(const std::string &key, const std::map<std::string, bool> &map);
};

#endif  // TUG_TESTING_PARAMETERHELPER_H
