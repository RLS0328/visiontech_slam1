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
#ifndef ETHZASL_MSF_NOROS_HEADER_H
#define ETHZASL_MSF_NOROS_HEADER_H
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

namespace std_msgs {

struct TimeStamp {
  uint64_t timestamp;
  double toSec() const {
    return 1.0e-9 * timestamp;
  }
};

template<class ContainerAllocator>
struct Header_ {
  typedef Header_<ContainerAllocator> Type;

  Header_()
      : seq(0), stamp(), frame_id() {
  }
  Header_(const ContainerAllocator &_alloc)
      : seq(0), stamp(), frame_id(_alloc) {
    (void) _alloc;
  }

  typedef uint32_t _seq_type;
  _seq_type seq;

  typedef TimeStamp _stamp_type;
  _stamp_type stamp;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>
      _frame_id_type;
  _frame_id_type frame_id;

  typedef boost::shared_ptr<::std_msgs::Header_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr<::std_msgs::Header_<ContainerAllocator> const> ConstPtr;

}; // struct Header_

typedef ::std_msgs::Header_<std::allocator<void> > Header;

typedef boost::shared_ptr<::std_msgs::Header> HeaderPtr;
typedef boost::shared_ptr<::std_msgs::Header const> HeaderConstPtr;
}
#endif //ETHZASL_MSF_NOROS_HEADER_H
