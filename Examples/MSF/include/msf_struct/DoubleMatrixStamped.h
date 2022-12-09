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
#ifndef ETHZASL_MSF_NOROS_DOUBLEMATRIXSTAMPED_H
#define ETHZASL_MSF_NOROS_DOUBLEMATRIXSTAMPED_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <msf_struct/Header.h>

namespace sensor_fusion_comm {
template<class ContainerAllocator>
struct DoubleMatrixStamped_ {
  typedef DoubleMatrixStamped_<ContainerAllocator> Type;

  DoubleMatrixStamped_()
      : header(), rows(0), cols(0), data() {
  }
  DoubleMatrixStamped_(const ContainerAllocator &_alloc)
      : header(_alloc), rows(0), cols(0), data(_alloc) {
    (void) _alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef int32_t _rows_type;
  _rows_type rows;

  typedef int32_t _cols_type;
  _cols_type cols;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other> _data_type;
  _data_type data;

  typedef boost::shared_ptr<::sensor_fusion_comm::DoubleMatrixStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::sensor_fusion_comm::DoubleMatrixStamped_<ContainerAllocator> const> ConstPtr;

}; // struct DoubleMatrixStamped_

typedef ::sensor_fusion_comm::DoubleMatrixStamped_<std::allocator<void> > DoubleMatrixStamped;

typedef boost::shared_ptr<::sensor_fusion_comm::DoubleMatrixStamped> DoubleMatrixStampedPtr;
typedef boost::shared_ptr<::sensor_fusion_comm::DoubleMatrixStamped const> DoubleMatrixStampedConstPtr;
}
#endif //ETHZASL_MSF_NOROS_DOUBLEMATRIXSTAMPED_H
