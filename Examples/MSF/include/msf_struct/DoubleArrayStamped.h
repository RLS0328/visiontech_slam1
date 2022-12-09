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
#ifndef ETHZASL_MSF_NOROS_DOUBLEARRAYSTAMPED_H
#define ETHZASL_MSF_NOROS_DOUBLEARRAYSTAMPED_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>

namespace sensor_fusion_comm {
template<class ContainerAllocator>
struct DoubleArrayStamped_ {
  typedef DoubleArrayStamped_<ContainerAllocator> Type;

  DoubleArrayStamped_()
      : header(), data() {
  }
  DoubleArrayStamped_(const ContainerAllocator &_alloc)
      : header(_alloc), data(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other> _data_type;
  _data_type data;

  typedef boost::shared_ptr<::sensor_fusion_comm::DoubleArrayStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_fusion_comm::DoubleArrayStamped_<ContainerAllocator> const> ConstPtr;

}; // struct DoubleArrayStamped_

typedef ::sensor_fusion_comm::DoubleArrayStamped_<std::allocator<void> > DoubleArrayStamped;

typedef boost::shared_ptr<::sensor_fusion_comm::DoubleArrayStamped> DoubleArrayStampedPtr;
typedef boost::shared_ptr<::sensor_fusion_comm::DoubleArrayStamped const> DoubleArrayStampedConstPtr;

}
#endif //ETHZASL_MSF_NOROS_DOUBLEARRAYSTAMPED_H
