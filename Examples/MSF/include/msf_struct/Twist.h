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
#ifndef ETHZASL_MSF_NOROS_TWIST_H
#define ETHZASL_MSF_NOROS_TWIST_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Vector3.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct Twist_ {
  typedef Twist_<ContainerAllocator> Type;

  Twist_()
      : linear(), angular() {
  }
  Twist_(const ContainerAllocator &_alloc)
      : linear(_alloc), angular(_alloc) {
    (void) _alloc;
  }

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _linear_type;
  _linear_type linear;

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _angular_type;
  _angular_type angular;

  typedef boost::shared_ptr<::geometry_msgs::Twist_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::Twist_<ContainerAllocator> const> ConstPtr;

}; // struct Twist_

typedef ::geometry_msgs::Twist_<std::allocator<void> > Twist;

typedef boost::shared_ptr<::geometry_msgs::Twist> TwistPtr;
typedef boost::shared_ptr<::geometry_msgs::Twist const> TwistConstPtr;

}
#endif //ETHZASL_MSF_NOROS_TWIST_H
