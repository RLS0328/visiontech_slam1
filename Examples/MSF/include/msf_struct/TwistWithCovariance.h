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
#ifndef ETHZASL_MSF_NOROS_TWISTWITHCOVARIANCE_H
#define ETHZASL_MSF_NOROS_TWISTWITHCOVARIANCE_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Twist.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct TwistWithCovariance_ {
  typedef TwistWithCovariance_<ContainerAllocator> Type;

  TwistWithCovariance_()
      : twist(), covariance() {
    covariance.assign(0.0);
  }
  TwistWithCovariance_(const ContainerAllocator &_alloc)
      : twist(_alloc), covariance() {
    (void) _alloc;
    covariance.assign(0.0);
  }

  typedef ::geometry_msgs::Twist_<ContainerAllocator> _twist_type;
  _twist_type twist;

  typedef boost::array<double, 36> _covariance_type;
  _covariance_type covariance;

  typedef boost::shared_ptr<::geometry_msgs::TwistWithCovariance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::TwistWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct TwistWithCovariance_

typedef ::geometry_msgs::TwistWithCovariance_<std::allocator<void> > TwistWithCovariance;

typedef boost::shared_ptr<::geometry_msgs::TwistWithCovariance> TwistWithCovariancePtr;
typedef boost::shared_ptr<::geometry_msgs::TwistWithCovariance const> TwistWithCovarianceConstPtr;

}
#endif //ETHZASL_MSF_NOROS_TWISTWITHCOVARIANCE_H
