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
#ifndef ETHZASL_MSF_NOROS_POSE_H
#define ETHZASL_MSF_NOROS_POSE_H

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Point.h>
#include <msf_struct/Quaternion.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct Pose_ {
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
      : position(), orientation() {
  }
  Pose_(const ContainerAllocator &_alloc)
      : position(_alloc), orientation(_alloc) {
    (void) _alloc;
  }

  typedef ::geometry_msgs::Point_<ContainerAllocator> _position_type;
  _position_type position;

  typedef ::geometry_msgs::Quaternion_<ContainerAllocator> _orientation_type;
  _orientation_type orientation;

  typedef boost::shared_ptr<::geometry_msgs::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef ::geometry_msgs::Pose_<std::allocator<void>> Pose;

typedef boost::shared_ptr<::geometry_msgs::Pose> PosePtr;
typedef boost::shared_ptr<::geometry_msgs::Pose const> PoseConstPtr;
}
#endif //ETHZASL_MSF_NOROS_POSE_H
