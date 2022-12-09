/*
 * Copyright (c) 2018 Smart Mapping Tech Co., Ltd. All rights reserved.
 *
 * Created by Shuang Song on 5/21/18.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ETHZASL_MSF_NOROS_MSF_IMUHANDLER_NOROS_H
#define ETHZASL_MSF_NOROS_MSF_IMUHANDLER_NOROS_H
#include <msf_core/msf_IMUHandler.h>

namespace msf_core {

template<typename EKFState_T>
class IMUHandler_NOROS : public IMUHandler<EKFState_T> {
 public:
  IMUHandler_NOROS(MSF_SensorManager<EKFState_T>& mng,
                 const std::string& topic_namespace,
                 const std::string& parameternamespace)
      : IMUHandler<EKFState_T>(mng, topic_namespace, parameternamespace) {

  }

  virtual ~IMUHandler_NOROS() { }

  virtual bool Initialize() {
    return true;
  }
};
}

#endif //ETHZASL_MSF_NOROS_MSF_IMUHANDLER_NOROS_H
