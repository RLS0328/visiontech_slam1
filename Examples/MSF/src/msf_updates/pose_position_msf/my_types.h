/*
 * Copyright (c) 2018 Smart Mapping Tech Co., Ltd. All rights reserved.
 *
 * Created by Shuang Song on 5/28/18.
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
#ifndef ETHZASL_MSF_NOROS_MY_TYPES_H
#define ETHZASL_MSF_NOROS_MY_TYPES_H
#include <queue>
#include <mutex>
#include <condition_variable>
#include <Eigen/Dense>
#include <memory>






typedef struct Meas_ {
  typedef std::shared_ptr<Meas_> Ptr_mea;
  uint64_t timestamp;
  uint32_t seq;
  enum { IMU, VICON };

  virtual int type() const = 0;
} Meas;

typedef struct IMUMeas_ : public Meas_ {
  typedef std::shared_ptr<IMUMeas_> Ptr_mea;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual int type() const { return IMU; }
} IMUMeas;

typedef struct VICONMeas_ : public Meas_ {
  typedef std::shared_ptr<VICONMeas_> Ptr_mea;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual int type() const { return VICON; }
} VICONMeas;


template <typename T> class BlockQueue_t
{
public:
    //类的构造函数，初始化对象使用（可定义长度）
    BlockQueue_t(int maxSize)
    {
        t_maxSize = maxSize;
    }
    BlockQueue_t(){t_maxSize = 3;}

    ~BlockQueue_t(){;}
    //向队列中加数据，如果满则阻塞
    void Put_t(const T& x)
    {
        std::unique_lock<std::mutex> lock(t_mutexqueue);

        while (t_queue.size() == t_maxSize)
        {
            //cout << "the blocking queue is full,waiting..." << endl;
            t_notFull.wait(t_mutexqueue);
        }
        t_queue.push(x);
        t_notEmpty.notify_one();
    }
    //从队列中取数据，如果空则阻塞
    void Take_t( T& x)
    {
        std::unique_lock<std::mutex> lock(t_mutexqueue);

        while (t_queue.empty())
        {
            //cout << "the blocking queue is empty,wating..." << endl;
            t_notEmpty.wait(t_mutexqueue);
        }

        x = t_queue.front();
        t_queue.pop();
        t_notFull.notify_one();
    }

    bool no_data_t(){
        if(t_queue.empty()){
            return 1;
        }else{
            return 0;
        }
    }



    int GetSizeQueue_t()
    {
        return t_queue.size();
    }

private:
    std::queue<T> t_queue;                  //缓冲区
    std::mutex t_mutexqueue;                    //互斥量和条件变量结合起来使用
    std::condition_variable_any t_notEmpty;//不为空的条件变量
    std::condition_variable_any t_notFull; //没有满的条件变量
    unsigned int t_maxSize;                         //同步队列最大的size
};




#endif //ETHZASL_MSF_NOROS_MY_TYPES_H
