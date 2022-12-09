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
#ifndef ETHZASL_MSF_NOROS_TRANSFORM_H
#define ETHZASL_MSF_NOROS_TRANSFORM_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Vector3.h>
#include <msf_struct/Quaternion.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct Transform_ {
  typedef Transform_<ContainerAllocator> Type;

  Transform_()
      : translation(), rotation() {
  }
  Transform_(const ContainerAllocator &_alloc)
      : translation(_alloc), rotation(_alloc) {
    (void) _alloc;
  }

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _translation_type;
  _translation_type translation;

  typedef ::geometry_msgs::Quaternion_<ContainerAllocator> _rotation_type;
  _rotation_type rotation;

  typedef boost::shared_ptr<::geometry_msgs::Transform_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::Transform_<ContainerAllocator> const> ConstPtr;

}; // struct Transform_

typedef ::geometry_msgs::Transform_<std::allocator<void> > Transform;

typedef boost::shared_ptr<::geometry_msgs::Transform> TransformPtr;
typedef boost::shared_ptr<::geometry_msgs::Transform const> TransformConstPtr;
}
#endif //ETHZASL_MSF_NOROS_TRANSFORM_H
