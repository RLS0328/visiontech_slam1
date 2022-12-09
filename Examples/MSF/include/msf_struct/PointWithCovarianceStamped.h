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
#ifndef ETHZASL_MSF_NOROS_POINTWITHCOVARIANCESTAMPED_H
#define ETHZASL_MSF_NOROS_POINTWITHCOVARIANCESTAMPED_H
#include <string>
#include <vector>
#include <map>

#include <msf_struct/Header.h>
#include <msf_struct/Point.h>

namespace sensor_fusion_comm {
template<class ContainerAllocator>
struct PointWithCovarianceStamped_ {
  typedef PointWithCovarianceStamped_<ContainerAllocator> Type;

  PointWithCovarianceStamped_()
      : header(), point(), covariance() {
    covariance.assign(0.0);
  }
  PointWithCovarianceStamped_(const ContainerAllocator &_alloc)
      : header(_alloc), point(_alloc), covariance() {
    (void) _alloc;
    covariance.assign(0.0);
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::geometry_msgs::Point_<ContainerAllocator> _point_type;
  _point_type point;

  typedef boost::array<double, 9> _covariance_type;
  _covariance_type covariance;

  typedef boost::shared_ptr<::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_fusion_comm::PointWithCovarianceStamped_<ContainerAllocator> const> ConstPtr;

}; // struct PointWithCovarianceStamped_

typedef ::sensor_fusion_comm::PointWithCovarianceStamped_<std::allocator<void> > PointWithCovarianceStamped;

typedef boost::shared_ptr<::sensor_fusion_comm::PointWithCovarianceStamped> PointWithCovarianceStampedPtr;
typedef boost::shared_ptr<::sensor_fusion_comm::PointWithCovarianceStamped const> PointWithCovarianceStampedConstPtr;
}

#endif //ETHZASL_MSF_NOROS_POINTWITHCOVARIANCESTAMPED_H
