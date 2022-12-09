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
#ifndef ETHZASL_MSF_NOROS_POINT_H
#define ETHZASL_MSF_NOROS_POINT_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

namespace geometry_msgs {
template<class ContainerAllocator>
struct Point_ {
  typedef Point_<ContainerAllocator> Type;

  Point_()
      : x(0.0), y(0.0), z(0.0) {
  }
  Point_(const ContainerAllocator &_alloc)
      : x(0.0), y(0.0), z(0.0) {
    (void) _alloc;
  }

  typedef double _x_type;
  _x_type x;

  typedef double _y_type;
  _y_type y;

  typedef double _z_type;
  _z_type z;

  typedef boost::shared_ptr<::geometry_msgs::Point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::Point_<ContainerAllocator> const> ConstPtr;

}; // struct Point_

typedef ::geometry_msgs::Point_<std::allocator<void> > Point;

typedef boost::shared_ptr<::geometry_msgs::Point> PointPtr;
typedef boost::shared_ptr<::geometry_msgs::Point const> PointConstPtr;
}
#endif //ETHZASL_MSF_NOROS_POINT_H
