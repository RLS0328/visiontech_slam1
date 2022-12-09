/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <msf_core/config.h>
#include "my_types.h"
#include "pose_sensormanager_noros.h"

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

#include <pcl/visualization/pcl_visualizer.h>

#include <boost/circular_buffer.hpp>


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <pthread.h>
#include <sys/time.h>
#include <semaphore.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

#include "../../../include/System.h"
#include <mynteye/AR.h>
#include "../../../include/Viewer.h"
#include "../../../include/ORBextractor.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include <iomanip>

#include <sched.h>

#include <opencv2/highgui/highgui.hpp>

#include <mynteye/camera.h>
#include <mynteye/utils.h>
#include <mynteye/rate.h>

#include <mynteye/cam_utils.h>
#include <mynteye/counter.h>
#include <mynteye/cv_painter.h>


#define BUFFERSIZE1 2

using namespace std;
using namespace ORB_SLAM2;

//static ORB_SLAM2::ViewerAR viewerAR;
static AR ar;
static thread tViewer, t_ar, t_cam, t_tracking, t_extractor;
mynteye::Camera cam;
mynteye::DeviceInfo dev_info;

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

}

void stateAfterUpdateCallback(const boost::shared_ptr<msf_pose_sensor::PoseSensorManager::EKFState_T> &state) {

}


run() {

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




void campose(cv::Mat &Tcw, mpData *DataStruct) {
    cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);    //rotation matrix
    cv::Mat tcw = Tcw.rowRange(0, 3).col(3);        //translation matrix
    Eigen::Matrix3f rotation_matrix;

    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc * tcw;
    DataStruct->t0 = twc.at<float>(0);
    DataStruct->t1 = twc.at<float>(1);
    DataStruct->t2 = -twc.at<float>(2);

    rotation_matrix << Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
            Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
            Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2);

    Eigen::Quaternionf quat(rotation_matrix);
    DataStruct->x = -quat.x();
    DataStruct->y = -quat.y();
    DataStruct->z = quat.z();
    DataStruct->w = quat.w();
}

struct extractORB {
    std::vector<cv::KeyPoint> Keypoints;
    cv::Mat Descriptors;
    cv::Mat ImageArray[BUFFERSIZE1];
    int idx_image = 0;
    //cv::Mat frame;
};

struct timeval time0, time1, time2, time3, time4, time5;
float ms1 = 0, ms2 = 0, ms3 = 0;

//提取特征点和描述子线程
class Extractor {
private:
    cv::Mat im;
    int mpstate;
    std::vector<cv::KeyPoint> mvKeys;
    cv::Mat mDescriptors;
    int mbRGB;
    //int nFeatures,nLevels,fIniThFAST,fMinThFAST,mbRGB;
    //float fScaleFactor;

public:

    ORBextractor *mpORBextractor, *mpORBextractorIni;
    BlockQueue<extractORB> *mpORBbuffer;
    volatile bool IsExtractor;
    int framesize = 0;

    Extractor(const string &strSettingPath) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];
        mbRGB = fSettings["Camera.RGB"];
        IsExtractor = false;
        mpORBextractorIni = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractor = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBbuffer = new BlockQueue<extractORB>(BUFFERSIZE1);
    }

    ~Extractor() { ; }

    void run() {
        extractORB ORBData;
        while (1) {

            if (IsExtractor) {

                if (framesize == 0) {
                    gettimeofday(&time0, NULL);
                    cout << "time0 = " << (time0.tv_sec * 1000000 + time0.tv_usec) << endl;
                }
                framesize++;
                IsExtractor = false;
                cv::Mat mImGray;
                im.copyTo(mImGray);
                ORBData.ImageArray[ORBData.idx_image] = im.clone();
                ORBData.idx_image++;
                ORBData.idx_image = (ORBData.idx_image) % BUFFERSIZE1;
                if (mImGray.channels() == 3) {
                    if (mbRGB)
                        cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                    else
                        cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                } else if (mImGray.channels() == 4) {
                    if (mbRGB)
                        cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                    else
                        cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                }
                gettimeofday(&time2,NULL);

                if ((mpstate == 0) || (mpstate == 1)) {
                    (*mpORBextractorIni)(mImGray, cv::Mat(), mvKeys, mDescriptors);
                } else {
                    (*mpORBextractor)(mImGray, cv::Mat(), mvKeys, mDescriptors);
                }
                gettimeofday(&time3,NULL);

                ORBData.Keypoints = mvKeys;
                ORBData.Descriptors = mDescriptors;
                mpORBbuffer->Put(ORBData);

                ms2 = (time3.tv_sec*1000000+time3.tv_usec) - (time2.tv_sec*1000000+time2.tv_usec);
                //cout<<"ORB提取时间 = "<<ms2<<endl;
            } else {
                usleep(1000);
            }
        }
    }

    void setExtractor(cv::Mat im_, int state_) {
        im = im_.clone();
        mpstate = state_;
    }
};



//跟踪tracking类
class Track {
private:
    cv::Mat im_;
    int state;
    volatile bool mapped_;
    ORB_SLAM2::System *mSystem;
    AR *mAr;
    Extractor *mextractor;

public:


    Track(bool mapped, ORB_SLAM2::System *system, AR *ar, Extractor *extractor) {
        mSystem = system;
        mAr = ar;
        mextractor = extractor;
        mapped_ = mapped;
    }

    ~Track() { ; }

    void run() {
        mpData mpDataStruct;
        extractORB mpORBData;
        int timestamp = 0;
        while (1) {
            mextractor->mpORBbuffer->Take(mpORBData);

            cv::Mat frame;

            if (mpORBData.idx_image == 0) {
                frame = (mpORBData.ImageArray[BUFFERSIZE1 - 1]).clone();
            } else {
                frame = (mpORBData.ImageArray[mpORBData.idx_image - 1]).clone();
            }

            mpDataStruct.ImageArray2[mpDataStruct.idx_image] = frame.clone();
            mpDataStruct.idx_image++;
            mpDataStruct.idx_image = mpDataStruct.idx_image % BUFFERSIZE;

            gettimeofday(&time4,NULL);
            cv::Mat Tcw = mSystem->TrackMonocular(frame, mpORBData.Keypoints, mpORBData.Descriptors, timestamp++);
            gettimeofday(&time5,NULL);

            if (mapped_) {
                if (!Tcw.empty()) {
                    campose(Tcw, &mpDataStruct);//更换位姿

                }
                mAr->mpDatabuffer->Put(mpDataStruct);

                state = mSystem->GetTrackingState();
                if (state == 3) state = 0;
                else if (state == 2) state = 1;
                mAr->mploader->objs[0]->visible = true;
                vector<cv::KeyPoint> vKeys = mSystem->GetTrackedKeyPointsUn();
                vector<ORB_SLAM2::MapPoint *> vMPs = mSystem->GetTrackedMapPoints();
                mSystem->mpViewer->SetImagePose(frame,Tcw,state,vKeys,vMPs);
            }
            ms3 = (time5.tv_sec*1000000+time5.tv_usec) - (time4.tv_sec*1000000+time4.tv_usec);
            //cout<<"-------------------------track时间 = "<<ms3<<endl;
        }
    }
};

//主函数
int main(int argc, char **argv) {

    if (argc != 5) {
        cout << endl << "Usage: monoAR path_to_vocabulary path_to_settings is_map_ready path_to_map" << endl;
        //ros::shutdown();
        return 1;
    }

    bool mapped = (argv[3][0] != '0');
    if (mapped) {
        cout << "Locating Mode" << endl;
    } else {
        cout << "Mapping Mode" << endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true, mapped, argv[4]);
    cout << "Create SLAM System" << endl;

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    cout << "Loading Setting File" << endl;

    //set VideoCapture
    int w = fSettings["Camera.width"];
    int h = fSettings["Camera.height"];
    float fps = fSettings["Camera.fps"];
    cv::Mat frame;

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float k1 = fSettings["Camera.k1"];
    float k2 = fSettings["Camera.k2"];
    float p1 = fSettings["Camera.p1"];
    float p2 = fSettings["Camera.p2"];
    //设置AR对象的参数
    if (mapped) {
        string yaml_path = fSettings["YAML_PATH"];
        float near = fSettings["near"];
        float far = fSettings["far"];
        int back_color[3] = {0, 0, 0};
        back_color[0] = fSettings["back_red"];
        back_color[1] = fSettings["back_green"];
        back_color[2] = fSettings["back_blue"];



        ar = AR(w, h, yaml_path, fps);
        ar.setProject(fx, fy, cx, cy, near, far);
        ar.back_color[0] = back_color[0];
        ar.back_color[1] = back_color[1];
        ar.back_color[2] = back_color[2];

        SLAM.mpViewer->ar = &ar;
        t_ar = thread(&AR::run, &ar);    //cout<<"ar\t"<<t_ar.get_id()<<endl;
        //K = (cv::Mat_<float>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    }

    //初始化cam,extractor,tracking对象
    if (!mynteye::util::select(cam, &dev_info)) {
        return 1;
    }
    mynteye::util::print_stream_infos(cam, dev_info.index);

    std::cout << "Open device: " << dev_info.index << ", "
              << dev_info.name << std::endl << std::endl;

    mynteye::OpenParams params(dev_info.index);
    {
        // Framerate: 10(default), [0,60], [0,30](STREAM_2560x720)
        params.framerate = 30;

        // Color mode: raw(default), rectified
        // params.color_mode = ColorMode::COLOR_RECTIFIED;

        // Depth mode: colorful(default), gray, raw
        // params.depth_mode = DepthMode::DEPTH_GRAY;

        // Stream mode: left color only
        // params.stream_mode = StreamMode::STREAM_640x480;  // vga
        params.stream_mode = mynteye::StreamMode::STREAM_1280x720;  // hd
        // Stream mode: left+right color
        // params.stream_mode = StreamMode::STREAM_1280x480;  // vga
        //params.stream_mode = StreamMode::STREAM_2560x720;  // hd

        // Auto-exposure: true(default), false
        // params.state_ae = false;

        // Auto-white balance: true(default), false
        // params.state_awb = false;

        // Infrared intensity: 0(default), [0,6]
        params.ir_intensity = 0;
    }

    // Enable what process logics
    // cam.EnableProcessMode(ProcessMode::PROC_IMU_ALL);

    // Enable image infos

    cam.EnableImageInfo(false);

    // Enable what stream datas: left_color, right_color, depth
    /* if (mynteye::util::is_right_color_supported(params.stream_mode)) {
         cam.EnableStreamData(mynteye::ImageType::IMAGE_ALL);
     } else {
         cam.EnableStreamData(mynteye::ImageType::IMAGE_LEFT_COLOR);
         cam.EnableStreamData(mynteye::ImageType::IMAGE_DEPTH);
     }*/
    cam.EnableStreamData(mynteye::ImageType::IMAGE_LEFT_COLOR);
    bool is_imu_ok = cam.IsMotionDatasSupported();
    // Enable motion datas until you get them
    if (is_imu_ok)
    {
        cam.EnableMotionDatas();
    } else{

            std::cerr << "Error: IMU is not supported on your device." << std::endl;

    }

    {
        // Set image info callback
        cam.SetImgInfoCallback([](const std::shared_ptr<mynteye::ImgInfo>& info) {
            std::cout << "  [img_info] fid: " << info->frame_id
                      << ", stamp: " << info->timestamp
                      << ", expos: " << info->exposure_time << std::endl
                      << std::flush;
        });

        std::vector<mynteye::ImageType> types{
                mynteye::ImageType::IMAGE_LEFT_COLOR,
                //    mynteye::ImageType::IMAGE_RIGHT_COLOR,
                //   mynteye::ImageType::IMAGE_DEPTH,
        };
        for (auto&& type : types) {
            // Set stream data callback
            cam.SetStreamCallback(type, [](const mynteye::StreamData& data) {
                std::cout << "  [" << data.img->type() << "] fid: "
                          << data.img->frame_id() << std::endl
                          << std::flush;
            });
        }

        // Set motion data callback
        cam.SetMotionCallback([](const mynteye::MotionData& data) {
            if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
                std::cout << "[accel] stamp: " << data.imu->timestamp
                          << ", x: " << data.imu->accel[0]
                          << ", y: " << data.imu->accel[1]
                          << ", z: " << data.imu->accel[2]
                          << ", temp: " << data.imu->temperature
                          << std::endl;
            } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
                std::cout << "[gyro] stamp: " << data.imu->timestamp
                          << ", x: " << data.imu->gyro[0]
                          << ", y: " << data.imu->gyro[1]
                          << ", z: " << data.imu->gyro[2]
                          << ", temp: " << data.imu->temperature
                          << std::endl;
            }
            std::cout << std::flush;
        });
    }

    cam.Open(params);
    std::cout << std::endl;
    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "Open device success" << std::endl << std::endl;
    //cv::namedWindow("left color");
    // cv::namedWindow("depth");
    //Cam cam(w, h, fps);
    Extractor extractor(argv[2]);
    Track track(mapped, &SLAM, &ar, &extractor);

    //开启cam,extractor,track线程
    t_cam = thread(&mynteye::Camera::run, &cam);
    t_extractor = thread(&Extractor::run, &extractor);
    t_tracking = thread(&Track::run, &track);
    //pthread_create(&extractor, NULL, pthread, (void*)b)


    //改变线程优先级
    //得到当前线程的调度策略和优先级最大值与最小值
    sched_param sch,sch1;
    int policy;
    pthread_getschedparam(t_extractor.native_handle(), &policy, &sch);
    cout<<"now"<<sch.__sched_priority<<endl;
    cout<<"max"<<sched_get_priority_max(policy)<<endl;
    cout<<"min"<<sched_get_priority_min(policy)<<endl;
    switch(policy)
    {
        case SCHED_FIFO:
            cout<<"policy = FIFO"<<endl;
            break;
        case SCHED_RR:
            cout<<"policy = RR"<<endl;
            break;
        case SCHED_OTHER:
            cout<<"policy = OTHER"<<endl;
            break;
    }


    //设置调度策略，观察优先级最大值最小值是否改变，从而确定调度策略的改变是否成功。

    sch.sched_priority = 99;
    sch1.sched_priority = 40;
    policy = SCHED_RR;
    pthread_setschedparam(t_extractor.native_handle(), policy, &sch);
    pthread_setschedparam(t_tracking.native_handle(), policy, &sch1);
    cout<<"now"<<sch.__sched_priority<<endl;
    cout<<"max"<<sched_get_priority_max(policy)<<endl;
    cout<<"min"<<sched_get_priority_min(policy)<<endl;
    switch(policy)
    {
        case SCHED_FIFO:
            cout<<"policy = FIFO"<<endl;
            break;
        case SCHED_RR:
            cout<<"policy = RR"<<endl;
            break;
        case SCHED_OTHER:
            cout<<"policy = OTHER"<<endl;
            break;
    }

/*
    if (pthread_setschedparam(t_extractor.native_handle(), policy, &sch)) {
    std::cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }
    */



    Size imageSize(w,h);
    // Size newimageSize(w,h)
    cv::Mat distCoeffs,cameraMatrix,newcameraMatrix,map1,map2,newimage;
    distCoeffs = (cv::Mat_<double>(5,1)<<k1,k2,p1,p2,0);
    cameraMatrix = (cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    // newcameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, newimageSize, 0);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(),
                            imageSize, CV_16SC2, map1, map2);
    // cout<<"111"<<endl;
    while (cam.getFrame().empty()) {;}
    mpData mpDataStruct1;
    // cout<<"222"<<endl;
    while (!SLAM.isShutdown()) {
        // cout<<"333"<<endl;
        if (cam.cam_rdy) {
            // cout<<"444"<<endl;
            //ar.flag2 = true;
            cam.cam_rdy = false;
            frame = cam.getFrame();
            remap(frame,newimage,map1,map2,INTER_LINEAR);
            extractor.setExtractor(newimage, SLAM.GetTrackingState());//imu+图像打包输入

            // extractor.setExtractor(frame, SLAM.GetTrackingState());
            extractor.IsExtractor = true;
        } else {
            usleep(1000);
        }
    }
    //cout<<""<<newimage.size();
    cout << "extract size = " << extractor.framesize << endl;
    //cout << "capturesize = " << cam.counter << endl;
    gettimeofday(&time1, NULL);
    cout << "time1 = " << (time1.tv_sec * 1000000 + time1.tv_usec) << endl;
    ms1 = (time1.tv_sec * 1000000 + time1.tv_usec) - (time0.tv_sec * 1000000 + time0.tv_usec);
    cout << "time totle" << (int) ms1 << endl;
    cout << "per frame's average time = " << (float) ms1 / (float) extractor.framesize << endl;


    if (mapped) {
        tViewer.join();
        t_ar.join();
    }

    // cam.loop = false;
    cam.Close();
    t_cam.join();

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
