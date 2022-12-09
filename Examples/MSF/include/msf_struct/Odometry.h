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
#ifndef ETHZASL_MSF_NOROS_ODOMETRY_H
#define ETHZASL_MSF_NOROS_ODOMETRY_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/PoseWithCovariance.h>
#include <msf_struct/TwistWithCovariance.h>

namespace nav_msgs {
template<class ContainerAllocator>
struct Odometry_ {
  typedef Odometry_<ContainerAllocator> Type;

  Odometry_()
      : header(), child_frame_id(), pose(), twist() {
  }
  Odometry_(const ContainerAllocator &_alloc)
      : header(_alloc), child_frame_id(_alloc), pose(_alloc), twist(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
      _child_frame_id_type;
  _child_frame_id_type child_frame_id;

  typedef ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> _pose_type;
  _pose_type pose;

  typedef ::geometry_msgs::TwistWithCovariance_<ContainerAllocator> _twist_type;
  _twist_type twist;

  typedef boost::shared_ptr<::nav_msgs::Odometry_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::nav_msgs::Odometry_<ContainerAllocator> const> ConstPtr;

}; // struct Odometry_

typedef ::nav_msgs::Odometry_<std::allocator<void> > Odometry;

typedef boost::shared_ptr<::nav_msgs::Odometry> OdometryPtr;
typedef boost::shared_ptr<::nav_msgs::Odometry const> OdometryConstPtr;
}
#endif //ETHZASL_MSF_NOROS_ODOMETRY_H
