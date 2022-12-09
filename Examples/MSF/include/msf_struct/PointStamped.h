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
#ifndef ETHZASL_MSF_NOROS_POINTSTAMPED_H
#define ETHZASL_MSF_NOROS_POINTSTAMPED_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/Point.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct PointStamped_ {
  typedef PointStamped_<ContainerAllocator> Type;

  PointStamped_()
      : header(), point() {
  }
  PointStamped_(const ContainerAllocator &_alloc)
      : header(_alloc), point(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::geometry_msgs::Point_<ContainerAllocator> _point_type;
  _point_type point;

  typedef boost::shared_ptr<::geometry_msgs::PointStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::PointStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PointStamped_

typedef ::geometry_msgs::PointStamped_<std::allocator<void> > PointStamped;

typedef boost::shared_ptr<::geometry_msgs::PointStamped> PointStampedPtr;
typedef boost::shared_ptr<::geometry_msgs::PointStamped const> PointStampedConstPtr;
}
#endif //ETHZASL_MSF_NOROS_POINTSTAMPED_H
