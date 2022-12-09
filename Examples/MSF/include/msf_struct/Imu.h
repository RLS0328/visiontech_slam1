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
#ifndef ETHZASL_MSF_NOROS_IMU_H
#define ETHZASL_MSF_NOROS_IMU_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>
#include <msf_struct/Vector3.h>
#include <msf_struct/Quaternion.h>

namespace sensor_msgs {
template<class ContainerAllocator>
struct Imu_ {
  typedef Imu_<ContainerAllocator> Type;

  Imu_()
      : header(),
        orientation(),
        orientation_covariance(),
        angular_velocity(),
        angular_velocity_covariance(),
        linear_acceleration(),
        linear_acceleration_covariance() {
    orientation_covariance.assign(0.0);

    angular_velocity_covariance.assign(0.0);

    linear_acceleration_covariance.assign(0.0);
  }
  Imu_(const ContainerAllocator &_alloc)
      : header(_alloc),
        orientation(_alloc),
        orientation_covariance(),
        angular_velocity(_alloc),
        angular_velocity_covariance(),
        linear_acceleration(_alloc),
        linear_acceleration_covariance() {
    (void) _alloc;
    orientation_covariance.assign(0.0);

    angular_velocity_covariance.assign(0.0);

    linear_acceleration_covariance.assign(0.0);
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef ::geometry_msgs::Quaternion_<ContainerAllocator> _orientation_type;
  _orientation_type orientation;

  typedef boost::array<double, 9> _orientation_covariance_type;
  _orientation_covariance_type orientation_covariance;

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _angular_velocity_type;
  _angular_velocity_type angular_velocity;

  typedef boost::array<double, 9> _angular_velocity_covariance_type;
  _angular_velocity_covariance_type angular_velocity_covariance;

  typedef ::geometry_msgs::Vector3_<ContainerAllocator> _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

  typedef boost::array<double, 9> _linear_acceleration_covariance_type;
  _linear_acceleration_covariance_type linear_acceleration_covariance;

  typedef boost::shared_ptr<::sensor_msgs::Imu_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_msgs::Imu_<ContainerAllocator> const> ConstPtr;

}; // struct Imu_

typedef ::sensor_msgs::Imu_<std::allocator<void> > Imu;

typedef boost::shared_ptr<::sensor_msgs::Imu> ImuPtr;
typedef boost::shared_ptr<::sensor_msgs::Imu const> ImuConstPtr;
}
#endif //ETHZASL_MSF_NOROS_IMU_H
