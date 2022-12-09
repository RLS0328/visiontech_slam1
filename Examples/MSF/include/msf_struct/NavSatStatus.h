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
#ifndef ETHZASL_MSF_NOROS_NAVSATSTATUS_H
#define ETHZASL_MSF_NOROS_NAVSATSTATUS_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>
namespace sensor_msgs {
template<class ContainerAllocator>
struct NavSatStatus_ {
  typedef NavSatStatus_<ContainerAllocator> Type;

  NavSatStatus_()
      : status(0), service(0) {
  }
  NavSatStatus_(const ContainerAllocator &_alloc)
      : status(0), service(0) {
    (void) _alloc;
  }

  typedef int8_t _status_type;
  _status_type status;

  typedef uint16_t _service_type;
  _service_type service;

  enum {
    STATUS_NO_FIX = -1,
    STATUS_FIX = 0,
    STATUS_SBAS_FIX = 1,
    STATUS_GBAS_FIX = 2,
    SERVICE_GPS = 1u,
    SERVICE_GLONASS = 2u,
    SERVICE_COMPASS = 4u,
    SERVICE_GALILEO = 8u,
  };

  typedef boost::shared_ptr<::sensor_msgs::NavSatStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_msgs::NavSatStatus_<ContainerAllocator> const> ConstPtr;

}; // struct NavSatStatus_

typedef ::sensor_msgs::NavSatStatus_<std::allocator<void> > NavSatStatus;

typedef boost::shared_ptr<::sensor_msgs::NavSatStatus> NavSatStatusPtr;
typedef boost::shared_ptr<::sensor_msgs::NavSatStatus const> NavSatStatusConstPtr;
}
#endif //ETHZASL_MSF_NOROS_NAVSATSTATUS_H
