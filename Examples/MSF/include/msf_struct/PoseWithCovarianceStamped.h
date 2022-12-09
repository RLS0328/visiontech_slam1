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
#ifndef ETHZASL_MSF_NOROS_POSEWITHCOVARIANCESTAMPED_H
#define ETHZASL_MSF_NOROS_POSEWITHCOVARIANCESTAMPED_H

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/PoseWithCovariance.h>

namespace geometry_msgs {
template<class ContainerAllocator>
struct PoseWithCovarianceStamped_ {
  typedef PoseWithCovarianceStamped_<ContainerAllocator> Type;

  PoseWithCovarianceStamped_()
      : header(), pose() {
  }
  PoseWithCovarianceStamped_(const ContainerAllocator &_alloc)
      : header(_alloc), pose(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> _pose_type;
  _pose_type pose;

  typedef boost::shared_ptr<::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::geometry_msgs::PoseWithCovarianceStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PoseWithCovarianceStamped_

typedef ::geometry_msgs::PoseWithCovarianceStamped_<std::allocator<void> > PoseWithCovarianceStamped;

typedef boost::shared_ptr<::geometry_msgs::PoseWithCovarianceStamped> PoseWithCovarianceStampedPtr;
typedef boost::shared_ptr<::geometry_msgs::PoseWithCovarianceStamped const> PoseWithCovarianceStampedConstPtr;
}

#endif //ETHZASL_MSF_NOROS_POSEWITHCOVARIANCESTAMPED_H
