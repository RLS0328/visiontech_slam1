/*
 * Copyright (c) 2018 Smart Mapping Tech Co., Ltd. All rights reserved.
 *
 * Created by Shuang Song on 5/25/18.
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
#ifndef ETHZASL_MSF_NOROS_POSESTAMPED_H
#define ETHZASL_MSF_NOROS_POSESTAMPED_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/Pose.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct PoseStamped_ {
  typedef PoseStamped_<ContainerAllocator> Type;

  PoseStamped_()
      : header(), pose() {
  }
  PoseStamped_(const ContainerAllocator &_alloc)
      : header(_alloc), pose(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::geometry_msgs::Pose_<ContainerAllocator> _pose_type;
  _pose_type pose;

  typedef boost::shared_ptr<::geometry_msgs::PoseStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::PoseStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PoseStamped_

typedef ::geometry_msgs::PoseStamped_<std::allocator<void> > PoseStamped;

typedef boost::shared_ptr<::geometry_msgs::PoseStamped> PPoseStampedPtr;
typedef boost::shared_ptr<::geometry_msgs::PoseStamped const> PoseStampedConstPtr;
}
#endif //ETHZASL_MSF_NOROS_POSESTAMPED_H
