/*
 * Copyright (c) 2018 Smart Mapping Tech Co., Ltd. All rights reserved.
 *
 * Created by Shuang Song on 5/25/18.
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
#ifndef ETHZASL_MSF_NOROS_POSE_SENSORMANAGER_NOROS_H
#define ETHZASL_MSF_NOROS_POSE_SENSORMANAGER_NOROS_H
#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanager.h>
#include <msf_core/msf_IMUHandler_NOROS.h>
#include <msf_core/config.h>

#include "msf_statedef.hpp"

#include <msf_updates/pose_sensor_handler/pose_sensorhandler.h>
#include <msf_updates/pose_sensor_handler/pose_measurement.h>
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>

namespace msf_pose_sensor {
class PoseSensorManager : public msf_core::MSF_SensorManager<
    msf_updates::EKFState> {
  typedef PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement<>,
                            PoseSensorManager> PoseSensorHandler_T;

  friend class PoseSensorHandler<msf_updates::pose_measurement::PoseMeasurement<>, PoseSensorManager>;

  typedef msf_position_sensor::PositionSensorHandler<msf_updates::position_measurement::PositionMeasurement,
                                                     PoseSensorManager> PositionSensorHandler_T;

  friend class msf_position_sensor::PositionSensorHandler<msf_updates::position_measurement::PositionMeasurement,
                                                          PoseSensorManager>;
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  typedef void (*Callback_T)(const shared_ptr<EKFState_T> &);
  PoseSensorManager()
      : pfPublishStateInitialCallback(nullptr),
        pfPublishStateAfterPropagationCallback(nullptr),
        pfPublishStateAfterUpdateCallback(nullptr) {
    bool distortmeas = false;  ///< Distort the pose measurements.

    imu_handler_.reset(
        new msf_core::IMUHandler_NOROS<msf_updates::EKFState>(*this, "msf_core",
                                                              "imu_handler"));
    pose_handler_.reset(
        new PoseSensorHandler_T(*this, "", "pose_sensor", false));

    double pose_noise_meas_p = Config::get<double>("pose_sensor/pose_noise_meas_p", 0.005);
    double pose_noise_meas_q = Config::get<double>("pose_sensor/pose_noise_meas_q", 0.005);
    double pose_delay = Config::get<double>("pose_sensor/pose_delay", 0.0);
    pose_handler_->SetNoises(pose_noise_meas_p,
                             pose_noise_meas_q);
    pose_handler_->SetDelay(pose_delay);

    position_handler_.reset(
        new PositionSensorHandler_T(*this, "", "position_sensor"));
    double position_noise_meas_p = Config::get<double>("position_sensor/position_noise_meas_p", 0.005);
    double position_delay = Config::get<double>("position_sensor/pose_delay", 0.0);

    position_handler_->SetNoises(position_noise_meas_p);
    position_handler_->SetDelay(position_delay);

    AddHandler(pose_handler_);
    AddHandler(position_handler_);
  }

  virtual ~PoseSensorManager() {}

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T &state) const {
    //set scale to 1
    Eigen::Matrix<double, 1, 1> scale;
    scale << 1.0;//1.0
    state.Set<StateDefinition_T::L>(scale);
  }

  shared_ptr<msf_core::IMUHandler_NOROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<PoseSensorHandler_T> pose_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;

  virtual void Init(double scale, double time) const {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_ip, p_vc, p_wv;
    Eigen::Quaternion<double> q, q_wv, q_ic, q_cv;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g <<  -0.224304199, 9.829907227, -0.247631836;            /// Gravity.0, 0, 9.81  1413393224705760000
    b_w << 0, 0, 0;        /// Bias gyroscopes.
    b_a << 0, 0, 0;        /// Bias accelerometer.

    v << 0, 0, 0;            /// Robot velocity (IMU centered).
    w_m << 0, 0, 0;        /// Initial angular velocity.

    //q_wv.setIdentity();  // Vision-world rotation drift.
    //p_wv.setZero();  // Vision-world position drift.
   /* p_wv << -0.0216401454975, -0.064676986768, 0.00981073058949;//pic
    q_wv.x() = 0.00770718; //qic
    q_wv.y() = -0.0104993;
    q_wv.z() = -0.701753;
    q_wv.w() = 0.71230;*/

      p_wv << -0.04777362000000000108, -0.00223730999999999991, -0.00160071000000000008;//pic
      q_wv.x() = -0.9999875; //qic
      q_wv.y() = -0.0021644;
      q_wv.z() = -0.0034724;
      q_wv.w() = 0.0028832;

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

    p_vc = pose_handler_->GetPositionMeasurement();
    q_cv = pose_handler_->GetAttitudeMeasurement();

    MSF_INFO_STREAM(
        "initial measurement pos:[" << p_vc.transpose() << "] orientation: " << STREAMQUAT(q_cv));

    // Check if we have already input from the measurement sensor.
    if (p_vc.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position - using [0 0 0]");
    if (q_cv.w() == 1)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize attitude - using [1 0 0 0]");

    p_ic[0] = Config::get<double>("msf_app/init/p_ic/x", 0.0);
    p_ic[1] = Config::get<double>("msf_app/init/p_ic/y", 0.0);
    p_ic[2] = Config::get<double>("msf_app/init/p_ic/z", 0.0);

    q_ic.w() = Config::get<double>("msf_app/init/q_ic/w", 1.0);
    q_ic.x() = Config::get<double>("msf_app/init/q_ic/x", 0.0);
    q_ic.y() = Config::get<double>("msf_app/init/q_ic/y", 0.0);
    q_ic.z() = Config::get<double>("msf_app/init/q_ic/z", 0.0);

    q_ic.normalize();

    // Calculate initial attitude and position based on sensor measurements.
    if (!pose_handler_->ReceivedFirstMeasurement()) {  // If there is no pose measurement, only apply q_wv.
      q = q_wv;


    } else {  // If there is a pose measurement, apply q_ic and q_wv to get initial attitude.
      q = (q_ic * q_cv.conjugate() * q_wv).conjugate();
    }

    q.normalize();

    p_ip[0] = Config::get<double>("msf_app/init/p_ip/x", 0.0);
    p_ip[1] = Config::get<double>("msf_app/init/p_ip/y", 0.0);
    p_ip[2] = Config::get<double>("msf_app/init/p_ip/z", 0.0);

    //TODO: init with p_ic and p_ip
    p = p_wv + q_wv.conjugate().toRotationMatrix() * p_vc / scale
        - q.toRotationMatrix() * p_ic;

    a_m = q.inverse() * g;            /// Initial acceleration.

    // Prepare init "measurement"
    // True means that this message contains initial sensor readings.
    shared_ptr<msf_core::MSF_InitMeasurement<EKFState_T>
    > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue<StateDefinition_T::p>(p);
    meas->SetStateInitValue<StateDefinition_T::v>(v);
    meas->SetStateInitValue<StateDefinition_T::q>(q);
    meas->SetStateInitValue<StateDefinition_T::b_w>(b_w);
    meas->SetStateInitValue<StateDefinition_T::b_a>(b_a);
    meas->SetStateInitValue<StateDefinition_T::L
    >(Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->SetStateInitValue<StateDefinition_T::q_wv>(q_wv);
    meas->SetStateInitValue<StateDefinition_T::p_wv>(p_wv);
    meas->SetStateInitValue<StateDefinition_T::q_ic>(q_ic);
    meas->SetStateInitValue<StateDefinition_T::p_ic>(p_ic);
    meas->SetStateInitValue<StateDefinition_T::p_ip>(p_ip);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = time;

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  void InitAlign(double scale, double time, Eigen::Vector3d position, Eigen::Quaterniond orientation) const {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ic, p_ip, p_vc, p_wv;
    Eigen::Quaternion<double> q, q_wv, q_ic, q_cv;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
      g << 0.607137447, 9.721384279, 1.154998165;            /// Gravity.0, 0, 9.81
      b_w << 0, 0, 0;        /// Bias gyroscopes.
      b_a << 0, 0, 0;        /// Bias accelerometer.

      v << 0, 0, 0;            /// Robot velocity (IMU centered).
      w_m << 0, 0, 0;        /// Initial angular velocity.

      //q_wv.setIdentity();  // Vision-world rotation drift.
      //p_wv.setZero();  // Vision-world position drift.
      p_wv << -0.0216401454975, -0.064676986768, 0.00981073058949;//pic
      q_wv.x() = -0.00770718; //qic
      q_wv.y() = 0.0104993;
      q_wv.z() = 0.701753;
      q_wv.w() = 0.71230;
    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used

    p_vc = pose_handler_->GetPositionMeasurement();
    q_cv = pose_handler_->GetAttitudeMeasurement();

    MSF_INFO_STREAM(
        "initial measurement pos:[" << p_vc.transpose() << "] orientation: " << STREAMQUAT(q_cv));

    // Check if we have already input from the measurement sensor.
    if (p_vc.norm() == 0)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position - using [0 0 0]");
    if (q_cv.w() == 1)
      MSF_WARN_STREAM(
          "No measurements received yet to initialize attitude - using [1 0 0 0]");

    p_ic[0] = Config::get<double>("msf_app/init/p_ic/x", 0.0);
    p_ic[1] = Config::get<double>("msf_app/init/p_ic/y", 0.0);
    p_ic[2] = Config::get<double>("msf_app/init/p_ic/z", 0.0);

    q_ic.w() = Config::get<double>("msf_app/init/q_ic/w", 1.0);
    q_ic.x() = Config::get<double>("msf_app/init/q_ic/x", 0.0);
    q_ic.y() = Config::get<double>("msf_app/init/q_ic/y", 0.0);
    q_ic.z() = Config::get<double>("msf_app/init/q_ic/z", 0.0);

    q_ic.normalize();

//     Calculate initial attitude and position based on sensor measurements.
//    if (!pose_handler_->ReceivedFirstMeasurement()) {  // If there is no pose measurement, only apply q_wv.
//      q = q_wv;
//    } else {  // If there is a pose measurement, apply q_ic and q_wv to get initial attitude.
//      q = (q_ic * q_cv.conjugate() * q_wv).conjugate();
//    }

//    q.normalize();

    p_ip[0] = Config::get<double>("msf_app/init/p_ip/x", 0.0);
    p_ip[1] = Config::get<double>("msf_app/init/p_ip/y", 0.0);
    p_ip[2] = Config::get<double>("msf_app/init/p_ip/z", 0.0);

    //TODO: init with p_ic and p_ip
//    p = p_wv + q_wv.conjugate().toRotationMatrix() * p_vc / scale
//        - q.toRotationMatrix() * p_ic;
    p = position;
    q = orientation;

    a_m = q.inverse() * g;            /// Initial acceleration.

    // Prepare init "measurement"
    // True means that this message contains initial sensor readings.
    shared_ptr<msf_core::MSF_InitMeasurement<EKFState_T>
    > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue<StateDefinition_T::p>(p);
    meas->SetStateInitValue<StateDefinition_T::v>(v);
    meas->SetStateInitValue<StateDefinition_T::q>(q);
    meas->SetStateInitValue<StateDefinition_T::b_w>(b_w);
    meas->SetStateInitValue<StateDefinition_T::b_a>(b_a);
    meas->SetStateInitValue<StateDefinition_T::L
    >(Eigen::Matrix<double, 1, 1>::Constant(scale));
    meas->SetStateInitValue<StateDefinition_T::q_wv>(q_wv);
    meas->SetStateInitValue<StateDefinition_T::p_wv>(p_wv);
    meas->SetStateInitValue<StateDefinition_T::q_ic>(q_ic);
    meas->SetStateInitValue<StateDefinition_T::p_ic>(p_ic);
    meas->SetStateInitValue<StateDefinition_T::p_ip>(p_ip);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = time;

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  virtual void InitState(EKFState_T &state) const {
    UNUSED(state);
  }

  virtual void CalculateQAuxiliaryStates(EKFState_T &state, double dt) const {
    static double pose_noise_q_wv = Config::get<double>("msf_app/pose_sensor/pose_noise_q_wv", 0.0);
    const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(
        pose_noise_q_wv);
    static double pose_noise_p_wv = Config::get<double>("msf_app/pose_sensor/pose_noise_p_wv", 0.0);
    const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(
        pose_noise_p_wv);
    static double pose_noise_q_ic = Config::get<double>("msf_app/pose_sensor/pose_noise_q_ic", 0.0);
    const msf_core::Vector3 nqicv = msf_core::Vector3::Constant(
        pose_noise_q_ic);
    static double pose_noise_p_ic = Config::get<double>("msf_app/pose_sensor/pose_noise_p_ic", 0.0);
    const msf_core::Vector3 npicv = msf_core::Vector3::Constant(
        pose_noise_p_ic);
    static double pose_noise_scale = Config::get<double>("msf_app/pose_sensor/pose_noise_scale", 0.0);
    const msf_core::Vector1 n_L = msf_core::Vector1::Constant(
        pose_noise_scale);
    static double position_noise_p_ip = Config::get<double>("msf_app/pose_sensor/position_noise_p_ip", 0.0);
    const msf_core::Vector3 npipv = msf_core::Vector3::Constant(
        position_noise_p_ip);

    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    state.GetQBlock<StateDefinition_T::L>() = (dt * n_L.cwiseProduct(n_L))
        .asDiagonal();
    state.GetQBlock<StateDefinition_T::q_wv>() =
        (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_wv>() =
        (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_ic>() =
        (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ic>() =
        (dt * npicv.cwiseProduct(npicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ip>() =
        (dt * npipv.cwiseProduct(npipv)).asDiagonal();
  }

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
                    EKFState_T::nErrorStatesAtCompileTime> &P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1> &correction) const {
    UNUSED(correction);
  }

  virtual void SanityCheckCorrection(
      EKFState_T &delaystate,
      const EKFState_T &buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1> &correction) const {
    UNUSED(buffstate);
    UNUSED(correction);

    const EKFState_T &state = delaystate;
    if (state.Get<StateDefinition_T::L>()(0) < 0) {
      MSF_WARN_STREAM_THROTTLE(
          1,
          "Negative scale detected: " << state.Get<StateDefinition_T::L>()(0) << ". Correcting to 0.1");
      Eigen::Matrix<double, 1, 1> L_;
      L_ << 0.1;
      delaystate.Set<StateDefinition_T::L>(L_);
    }
  }

/***
 * Provide a getter for these parameters, this is implemented for a given
 * middleware or param file parser.
 */
  virtual bool GetParamFixedBias() const {
    return Config::get<bool>("core/core_fixed_bias", false);
  }
  virtual double GetParamNoiseAcc() const {
    return Config::get<double>("core/core_noise_acc", 0.01);
  }
  virtual double GetParamNoiseAccbias() const {

    return Config::get<double>("core/core_noise_accbias", 0.01);
  }
  virtual double GetParamNoiseGyr() const {
    return Config::get<double>("core/core_noise_gyr", 0.01);
  }
  virtual double GetParamNoiseGyrbias() const {
    return Config::get<double>("core/core_noise_gyrbias", 0.01);
  }
  virtual double GetParamFuzzyTrackingThreshold() const {
    return Config::get<double>("core/fuzzy_tracking_threshold", 0.1);
  }

  virtual void PublishStateInitial(
      const shared_ptr<EKFState_T> &state) const {
    if (pfPublishStateInitialCallback != nullptr) {
      pfPublishStateInitialCallback(state);
    }
  }
  virtual void PublishStateAfterPropagation(
      const shared_ptr<EKFState_T> &state) const {
    if (pfPublishStateAfterPropagationCallback != nullptr) {
      pfPublishStateAfterPropagationCallback(state);
    }
  }
  virtual void PublishStateAfterUpdate(
      const shared_ptr<EKFState_T> &state) const {
    if (pfPublishStateAfterUpdateCallback != nullptr) {
      pfPublishStateAfterUpdateCallback(state);
    }
  }
  Callback_T pfPublishStateInitialCallback;
  Callback_T pfPublishStateAfterPropagationCallback;
  Callback_T pfPublishStateAfterUpdateCallback;

};
}
#endif //ETHZASL_MSF_NOROS_POSE_SENSORMANAGER_NOROS_H
