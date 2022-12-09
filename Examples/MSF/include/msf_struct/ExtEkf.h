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
#ifndef ETHZASL_MSF_NOROS_EXTEKF_H
#define ETHZASL_MSF_NOROS_EXTEKF_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/Vector3.h>

namespace sensor_fusion_comm {
template<class ContainerAllocator>
struct ExtEkf_ {
  typedef ExtEkf_<ContainerAllocator> Type;

  ExtEkf_()
      : header(), angular_velocity(), linear_acceleration(), state(), flag(0) {
  }
  ExtEkf_(const ContainerAllocator &_alloc)
      : header(_alloc), angular_velocity(_alloc), linear_acceleration(_alloc), state(_alloc), flag(0) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _angular_velocity_type;
  _angular_velocity_type angular_velocity;

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other> _state_type;
  _state_type state;

  typedef int32_t _flag_type;
  _flag_type flag;

  enum {
    ignore_state = 0u,
    current_state = 1u,
    initialization = 2u,
    state_correction = 3u,
  };

  typedef boost::shared_ptr<::sensor_fusion_comm::ExtEkf_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_fusion_comm::ExtEkf_<ContainerAllocator> const> ConstPtr;

}; // struct ExtEkf_

typedef ::sensor_fusion_comm::ExtEkf_<std::allocator<void> > ExtEkf;

typedef boost::shared_ptr<::sensor_fusion_comm::ExtEkf> ExtEkfPtr;
typedef boost::shared_ptr<::sensor_fusion_comm::ExtEkf const> ExtEkfConstPtr;
}
#endif //ETHZASL_MSF_NOROS_EXTEKF_H
