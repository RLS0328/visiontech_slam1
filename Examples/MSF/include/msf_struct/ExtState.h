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
#ifndef ETHZASL_MSF_NOROS_EXTSTATE_H
#define ETHZASL_MSF_NOROS_EXTSTATE_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

namespace sensor_fusion_comm {
template<class ContainerAllocator>
struct ExtState_ {
  typedef ExtState_<ContainerAllocator> Type;

  ExtState_()
      : header(), pose(), velocity() {
  }
  ExtState_(const ContainerAllocator &_alloc)
      : header(_alloc), pose(_alloc), velocity(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::geometry_msgs::Pose_<ContainerAllocator> _pose_type;
  _pose_type pose;

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _velocity_type;
  _velocity_type velocity;

  typedef boost::shared_ptr<::sensor_fusion_comm::ExtState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_fusion_comm::ExtState_<ContainerAllocator> const> ConstPtr;

}; // struct ExtState_

typedef ::sensor_fusion_comm::ExtState_<std::allocator<void> > ExtState;

typedef boost::shared_ptr<::sensor_fusion_comm::ExtState> ExtStatePtr;
typedef boost::shared_ptr<::sensor_fusion_comm::ExtState const> ExtStateConstPtr;

}
#endif //ETHZASL_MSF_NOROS_EXTSTATE_H
