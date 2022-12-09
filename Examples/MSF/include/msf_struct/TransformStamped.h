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
#ifndef ETHZASL_MSF_NOROS_TRANSFORMSTAMPED_H
#define ETHZASL_MSF_NOROS_TRANSFORMSTAMPED_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/Transform.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct TransformStamped_ {
  typedef TransformStamped_<ContainerAllocator> Type;

  TransformStamped_()
      : header(), child_frame_id(), transform() {
  }
  TransformStamped_(const ContainerAllocator &_alloc)
      : header(_alloc), child_frame_id(_alloc), transform(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
      _child_frame_id_type;
  _child_frame_id_type child_frame_id;

  typedef ::geometry_msgs::Transform_<ContainerAllocator> _transform_type;
  _transform_type transform;

  typedef boost::shared_ptr<::geometry_msgs::TransformStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::TransformStamped_<ContainerAllocator> const> ConstPtr;

}; // struct TransformStamped_

typedef ::geometry_msgs::TransformStamped_<std::allocator<void> > TransformStamped;

typedef boost::shared_ptr<::geometry_msgs::TransformStamped> TransformStampedPtr;
typedef boost::shared_ptr<::geometry_msgs::TransformStamped const> TransformStampedConstPtr;
}
#endif //ETHZASL_MSF_NOROS_TRANSFORMSTAMPED_H
