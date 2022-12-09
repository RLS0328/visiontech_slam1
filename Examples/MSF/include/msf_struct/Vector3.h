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
#ifndef ETHZASL_MSF_NOROS_VECTOR3_H
#define ETHZASL_MSF_NOROS_VECTOR3_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

namespace geometry_msgs {
template<class ContainerAllocator>
struct Vector3_ {
  typedef Vector3_<ContainerAllocator> Type;

  Vector3_()
      : x(0.0), y(0.0), z(0.0) {
  }
  Vector3_(const ContainerAllocator &_alloc)
      : x(0.0), y(0.0), z(0.0) {
    (void) _alloc;
  }

  typedef double _x_type;
  _x_type x;

  typedef double _y_type;
  _y_type y;

  typedef double _z_type;
  _z_type z;

  typedef boost::shared_ptr<::geometry_msgs::Vector3_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::Vector3_<ContainerAllocator> const> ConstPtr;

}; // struct Vector3_

typedef ::geometry_msgs::Vector3_<std::allocator<void> > Vector3;

typedef boost::shared_ptr<::geometry_msgs::Vector3> Vector3Ptr;
typedef boost::shared_ptr<::geometry_msgs::Vector3 const> Vector3ConstPtr;
}
#endif //ETHZASL_MSF_NOROS_VECTOR3_H
