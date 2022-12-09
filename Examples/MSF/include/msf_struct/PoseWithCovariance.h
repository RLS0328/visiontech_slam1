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
#ifndef ETHZASL_MSF_NOROS_POSEWITHCOVARIANCE_H
#define ETHZASL_MSF_NOROS_POSEWITHCOVARIANCE_H
#include <string>
#include <vector>
#include <map>

#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <msf_struct/Pose.h>
namespace geometry_msgs {
template<class ContainerAllocator>
struct PoseWithCovariance_ {
  typedef PoseWithCovariance_<ContainerAllocator> Type;

  PoseWithCovariance_()
      : pose(), covariance() {
    covariance.assign(0.0);
  }
  PoseWithCovariance_(const ContainerAllocator &_alloc)
      : pose(_alloc), covariance() {
    (void) _alloc;
    covariance.assign(0.0);
  }

  typedef ::geometry_msgs::Pose_<ContainerAllocator> _pose_type;
  _pose_type pose;

  typedef boost::array<double, 36> _covariance_type;
  _covariance_type covariance;

  typedef boost::shared_ptr<::geometry_msgs::PoseWithCovariance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::PoseWithCovariance_<ContainerAllocator> const> ConstPtr;

}; // struct PoseWithCovariance_

typedef ::geometry_msgs::PoseWithCovariance_<std::allocator<void>> PoseWithCovariance;

typedef boost::shared_ptr<::geometry_msgs::PoseWithCovariance> PoseWithCovariancePtr;
typedef boost::shared_ptr<::geometry_msgs::PoseWithCovariance const> PoseWithCovarianceConstPtr;
}
#endif //ETHZASL_MSF_NOROS_POSEWITHCOVARIANCE_H
