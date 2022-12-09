/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <msf_core/config.h>
#include "my_types.h"
#include "pose_sensormanager_noros.h"

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

#include <pcl/visualization/pcl_visualizer.h>

#include <boost/circular_buffer.hpp>

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("MSF"));
typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> Pose;
boost::circular_buffer<Pose> rawPose(1);
boost::circular_buffer<Pose> ekfPose(1);
pcl::PointCloud<pcl::PointXYZ> rawTraj;
pcl::PointCloud<pcl::PointXYZ> ekfTraj;

void viewerUpdate() {
  for (int i = 0; i < rawPose.size(); ++i) {
    auto &it = rawPose[i];
    Eigen::Affine3f af(it.second.toRotationMatrix().cast<float>());
    af.translation() = it.first.cast<float>();
    viewer->removeCoordinateSystem("rawpose" + to_string(i));
    viewer->addCoordinateSystem(0.3, af, "rawpose" + to_string(i));
  }

  for (int i = 0; i < ekfPose.size(); ++i) {
    auto &it = ekfPose[i];
    viewer->removeCoordinateSystem("ekfpose" + to_string(i));
    Eigen::Affine3f af(it.second.toRotationMatrix().cast<float>());
    af.translation() = it.first.cast<float>();

    viewer->addCoordinateSystem(1, af, "ekfpose" + to_string(i));
  }
  viewer->updatePointCloud(rawTraj.makeShared(), "rawTraj");
  viewer->updatePointCloud(ekfTraj.makeShared(), "ekfTraj");
  viewer->spinOnce(Config::get<int>("spin_duration"));
}

void stateAfterPropagationCallback(const boost::shared_ptr<msf_pose_sensor::PoseSensorManager::EKFState_T> &state) {
//  std::cout << "P" << state->ToEigenVector().transpose() << std::endl;
/*  auto evstate = state->ToEigenVector();


  Eigen::Vector3d p = evstate.block<3, 1>(0, 0);

  Eigen::Vector4d q_vec = evstate.block<4, 1>(6, 0);
  Eigen::Quaterniond q;
  q.x() = q_vec[1];
  q.y() = q_vec[2];
  q.z() = q_vec[3];
  q.w() = q_vec[0];*/

  //yyh

  //q_i_w----q_i_v
/*  double s_ = 2.0;
  Eigen::Quaterniond q_vw;
    q_vw.x() = 0.00771;
    q_vw.y() = -0.0105;
    q_vw.z() = -0.702;
    q_vw.w() = 0.712;
    Eigen::Matrix3d C_Q = q.toRotationMatrix();
    Eigen::Matrix3d C_q_v_w = q_vw.toRotationMatrix();
    C_Q =  C_q_v_w  * C_Q;
    Eigen::Quaterniond Q_p(C_Q);
    Eigen::Vector3d p_vw ;
    p_vw << 0.02164, 0.06468, -0.009811;
    q = Q_p;
    p =  C_q_v_w *s_ * p + p_vw;*/

  /*  Eigen::Quaterniond q_vw;
    q_vw.x() = -0.9999875;
    q_vw.y() = -0.0021644;
    q_vw.z() = -0.0034724;
    q_vw.w() = 0.0028832;
    Eigen::Matrix3d C_Q = q.toRotationMatrix();
    Eigen::Matrix3d C_q_v_w = q_vw.toRotationMatrix();
    C_Q =  C_q_v_w  * C_Q;
    Eigen::Quaterniond Q_p(C_Q);
    Eigen::Vector3d p_vw ;
    p_vw << 0.04777362000000000108, 0.00223730999999999991 , 0.00160071000000000008;
    q = Q_p;
    p =  C_q_v_w * p + p_vw;*/


    //std::cout << "111111" << msf_core::timehuman(state->time) << " " << p[0] << " " << p[1] << " " << p[2] << " "
     //         << q.x() << " " << q.y() << " " << q.z() << " " << q.w()  <<std::endl;
  //140363 std::cout <<  msf_core::timehuman(state->time) << " " <<p[0] <<" " <<  p[1] <<" " <<p[2] <<" " <<q.x() <<" " <<q.y() <<" " <<q.z() <<" " <<q.w() <<" " << msf_core::Auxiliary <<std::endl;

}

void stateAfterUpdateCallback(const boost::shared_ptr<msf_pose_sensor::PoseSensorManager::EKFState_T> &state) {
//  std::cout << "U " << state->ToEigenVector().transpose() << std::endl;
//  cout << state->Print() << endl;
  auto evstate = state->ToEigenVector();


  Eigen::Vector3d p = evstate.block<3, 1>(0, 0);

  Eigen::Vector4d q_vec = evstate.block<4, 1>(6, 0);
  Eigen::Quaterniond q;
  q.x() = q_vec[1];
  q.y() = q_vec[2];
  q.z() = q_vec[3];
  q.w() = q_vec[0];
//q_i_w--->q_i_v
  Eigen::Quaterniond q_w;
  Eigen::Vector3d p_w;
  p_w[0] = Config::get<double>("msf_app/init/p_ic/x", 0.0);
  p_w[1] = Config::get<double>("msf_app/init/p_ic/y", 0.0);
  p_w[2] = Config::get<double>("msf_app/init/p_ic/z", 0.0);

  q_w.w() = Config::get<double>("msf_app/init/q_ic/w", 1.0);
  q_w.x() = Config::get<double>("msf_app/init/q_ic/x", 0.0);
  q_w.y() = Config::get<double>("msf_app/init/q_ic/y", 0.0);
  q_w.z() = Config::get<double>("msf_app/init/q_ic/z", 0.0);
  //std::cout<< "ok"<<" " << p_w[0] << " " << p_w[1] << " " << p_w[2] << " "
   //        << q_w.x() << " " << q_w.y() << " " << q_w.z() << " " << q_w.w()  <<std::endl;
  Eigen::Matrix3d C_Q = q.toRotationMatrix();
  Eigen::Matrix3d C_q_v_w = q_w.conjugate().toRotationMatrix();
  Eigen::Matrix3d C_q_w_v = q_w.toRotationMatrix();
  C_Q = C_q_v_w * C_Q;
  //q_i_v--->q_c_v
  C_Q = C_Q * C_q_w_v;
  Eigen::Quaterniond Q_p(C_Q);
  q = Q_p;
  p = C_q_v_w * p - p_w;
  p = p + q * p_w;

  Pose pp = std::make_pair(p, q);
  ekfPose.push_back(pp);

  pcl::PointXYZ pt;
  pt.getVector3fMap() = p.cast<float>();
  ekfTraj.push_back(pt);

  viewerUpdate();
//插值需要
  std::cout << "111111" << msf_core::timehuman(state->time)  << " "
                                                             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
          << " " << p[0] << " " << p[1] << " " << p[2]<<std::endl;

}

int main(int argc, char **argv) {

  if (argc < 2)
    return -1;
  Config::setParameterFile(argv[1]);
  rawPose.resize(Config::get<int>("rawbuffer"));
  ekfPose.resize(Config::get<int>("ekfbuffer"));

  viewer->addText("imu: ", 100, 0, "imu");
  viewer->addText("vicon: ", 200, 0, "vicon");
  viewer->addPointCloud(rawTraj.makeShared(), "rawTraj");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "rawTraj");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "rawTraj");
  viewer->addPointCloud(ekfTraj.makeShared(), "ekfTraj");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "ekfTraj");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ekfTraj");

  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(0.0);
  coeffs.values.push_back(1.0);
  coeffs.values.push_back(0.0);
  viewer->addPlane(coeffs, "plane");

  msf_pose_sensor::PoseSensorManager manager;
  manager.pfPublishStateAfterPropagationCallback = stateAfterPropagationCallback;
  manager.pfPublishStateAfterUpdateCallback = stateAfterUpdateCallback;
  auto &imu_handler_ = manager.imu_handler_;
  auto &pose_handler_ = manager.pose_handler_;
  auto &position_handler_ = manager.position_handler_;

  vector<Meas::Ptr> measurements;

  uint32_t seq_cnt = 0;
  string imuTxt = Config::get<string>("imu_data_txt");
  string viconTxt = Config::get<string>("vicon_data_txt");
  ifstream imuFS(imuTxt.c_str());
   //yyh  1403636618153550000  0.5017361 0.1238355 0.464439 0.1835151 0.0186067 -0.0404292 0.9818663   i木 9.447072833, -0.473988083, -2.574245625
   /* Eigen::Quaterniond C_Q;
    C_Q.x() = 0.1835151;
    C_Q.y() = 0.0186067;
    C_Q.z() = -0.0404292;
    C_Q.w() = 0.9818663;
    Eigen::Matrix3d C_Q_w_v = C_Q.conjugate().toRotationMatrix();
    Eigen::Vector3d P_wv ;
    P_wv << 0.5017361, 0.1238355, 0.464439;*/
   // double s = 3.79;
   // double s_ = 1.00 / 3.79;
  //1403636618163550000  9.46341725, -0.506676917, -2.582417833 imu
  /* Eigen::Quaterniond C_Q;
    C_Q.x() = 0.204237;
    C_Q.y() = 0.00776071;
    C_Q.z() = -0.0364049;
    C_Q.w() = 0.978214;
    Eigen::Matrix3d C_Q_w_v = C_Q.conjugate().toRotationMatrix();


    Eigen::Vector3d P_wv ;
    P_wv << 0.452646, 0.332618, 0.391493;*/
  //1111111121042960000 	-0.115142	0.262767	0.209259	-0.0708026	-0.295412	0.031713	0.952215
//1111110453773960000 	0.0215313	-0.0685612	-0.0897613	0.048704	0.029095	0.00749247	0.998361
    Eigen::Quaterniond C_Q;
    C_Q.x() = -0.0708026;
    C_Q.y() = -0.295412;
    C_Q.z() = 0.0317137;
    C_Q.w() = 0.952215;
    Eigen::Matrix3d C_Q_w_v = C_Q.conjugate().toRotationMatrix();


    Eigen::Vector3d P_wv ;
    P_wv << -0.115142, 0.262767, 0.209259;

  while (imuFS.peek() != EOF) {
    IMUMeas::Ptr mea(new IMUMeas);
    mea->seq = seq_cnt++;
    //1349442751013921022 -4.37526 4.39488 9.35874 -2.10120188648 0.643241095823 -0.307614280664
    imuFS >> mea->timestamp >> mea-> linear_acceleration[0] >> mea->linear_acceleration[1] >> mea->linear_acceleration[2]
          >> mea->angular_velocity[0] >> mea->angular_velocity[1] >> mea->angular_velocity[2];
    measurements.push_back(mea);
  }
  imuFS.close();

  seq_cnt = 0;
  ifstream viconFS(viconTxt.c_str());
  while (viconFS.peek() != EOF) {
    VICONMeas::Ptr mea(new VICONMeas);
    mea->seq = seq_cnt++;
    //1349442751009001180 0.193471895296 0.0169222941425 1.19366082671 0.259788004921 0.220592089619 -0.0481239682821 0.93890010447
    viconFS >> mea->timestamp >> mea->p[0] >> mea->p[1] >> mea->p[2] >> mea->q.coeffs()[0] >> mea->q.coeffs()[1]
            >> mea->q.coeffs()[2] >> mea->q.coeffs()[3];

//yyh
      Eigen::Matrix3d C_Q_p = mea->q.toRotationMatrix();
      C_Q_p =  C_Q_w_v * C_Q_p;
      Eigen::Quaterniond Q_p(C_Q_p);
      mea->q = Q_p;
      mea->p =  C_Q_w_v * mea->p - P_wv;

    measurements.push_back(mea);
  }
  viconFS.close();

  std::sort(measurements.begin(), measurements.end(), [](const Meas::Ptr &lhs, const Meas::Ptr &rhs) {
    return lhs->timestamp < rhs->timestamp;
  });

  manager.Init(1.0, measurements.front()->timestamp * 1.0e-9);//1.0yyh

  for (auto &it: measurements) {
    if (it->type() == Meas::IMU) {
      IMUMeas::Ptr imuMsg = static_pointer_cast<IMUMeas>(it);
      imu_handler_->ProcessIMU(imuMsg->linear_acceleration,
                               imuMsg->angular_velocity,
                               1.0e-9 * imuMsg->timestamp,
                               imuMsg->seq);

      viewer->updateText("imu: " + to_string(imuMsg->seq), 100, 0, "imu");

    } else if (it->type() == Meas::VICON) {
      VICONMeas::Ptr viconMsg = static_pointer_cast<VICONMeas>(it);
      geometry_msgs::PoseWithCovarianceStampedPtr pose(
          new geometry_msgs::PoseWithCovarianceStamped());

      // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
      Pose pp = std::make_pair(viconMsg->p, viconMsg->q);
      rawPose.push_back(pp);

      pcl::PointXYZ pt;
      pt.getVector3fMap() = viconMsg->p.cast<float>();
      rawTraj.push_back(pt);

      pose->header.stamp.timestamp = viconMsg->timestamp;

      pose->pose.pose.position.x = viconMsg->p[0];
      pose->pose.pose.position.y = viconMsg->p[1];
      pose->pose.pose.position.z = viconMsg->p[2];

      pose->pose.pose.orientation.w = viconMsg->q.w();
      pose->pose.pose.orientation.x = viconMsg->q.x();
      pose->pose.pose.orientation.y = viconMsg->q.y();
      pose->pose.pose.orientation.z = viconMsg->q.z();

      sensor_fusion_comm::PointWithCovarianceStampedPtr point(
          new sensor_fusion_comm::PointWithCovarianceStamped());
      point->header.stamp.timestamp = viconMsg->timestamp;
      point->point.x = viconMsg->p[0];
      point->point.y = viconMsg->p[1];
      point->point.z = viconMsg->p[2];

      if (Config::get<bool>("use_position_update")) {
        position_handler_->ProcessPositionMeasurement(point);
      } else {
        pose_handler_->ProcessPoseMeasurement(pose);
      }
      viewer->updateText("vicon: " + to_string(viconMsg->seq), 200, 0, "vicon");
    }
  }

  viewer->spin();

  return 0;
}
