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
#ifndef ETHZASL_MSF_NOROS_NAVSATFIX_H
#define ETHZASL_MSF_NOROS_NAVSATFIX_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/NavSatStatus.h>

namespace sensor_msgs {
template<class ContainerAllocator>
struct NavSatFix_ {
  typedef NavSatFix_<ContainerAllocator> Type;

  NavSatFix_()
      : header(),
        status(),
        latitude(0.0),
        longitude(0.0),
        altitude(0.0),
        position_covariance(),
        position_covariance_type(0) {
    position_covariance.assign(0.0);
  }
  NavSatFix_(const ContainerAllocator &_alloc)
      : header(_alloc),
        status(_alloc),
        latitude(0.0),
        longitude(0.0),
        altitude(0.0),
        position_covariance(),
        position_covariance_type(0) {
    (void) _alloc;
    position_covariance.assign(0.0);
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::sensor_msgs::NavSatStatus_<ContainerAllocator> _status_type;
  _status_type status;

  typedef double _latitude_type;
  _latitude_type latitude;

  typedef double _longitude_type;
  _longitude_type longitude;

  typedef double _altitude_type;
  _altitude_type altitude;

  typedef boost::array<double, 9> _position_covariance_type;
  _position_covariance_type position_covariance;

  typedef uint8_t _position_covariance_type_type;
  _position_covariance_type_type position_covariance_type;

  enum {
    COVARIANCE_TYPE_UNKNOWN = 0u,
    COVARIANCE_TYPE_APPROXIMATED = 1u,
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2u,
    COVARIANCE_TYPE_KNOWN = 3u,
  };

  typedef boost::shared_ptr<::sensor_msgs::NavSatFix_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_msgs::NavSatFix_<ContainerAllocator> const> ConstPtr;

}; // struct NavSatFix_

typedef ::sensor_msgs::NavSatFix_<std::allocator<void> > NavSatFix;

typedef boost::shared_ptr<::sensor_msgs::NavSatFix> NavSatFixPtr;
typedef boost::shared_ptr<::sensor_msgs::NavSatFix const> NavSatFixConstPtr;
}
#endif //ETHZASL_MSF_NOROS_NAVSATFIX_H
