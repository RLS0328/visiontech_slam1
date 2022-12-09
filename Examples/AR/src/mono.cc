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

//#define _GLIBCXX_USE_CXX11_ABI 0
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <sys/time.h>
#include <semaphore.h>
#include <sys/types.h>
#include <unistd.h>
#include <my_types.h>
#include "System.h"
#include "AR.h"
#include "Viewer.h"
#include <ORBextractor.h>
#include "to_msf.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include <iomanip>
#define BUFFERSIZE1 2


using namespace std;
using namespace ORB_SLAM2;

struct Imu7{
    double time_imu;
    float acc_imu0,acc_imu1,acc_imu2,gyr_imu0,gyr_imu1,gyr_imu2;
};

struct imu_data {
//public:
    uint64_t imu_timestamp;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d gyr_velocity;
    //cv::Mat frame;

    //Tcw_pose pose_result;
    cv::Mat Tcw_ = Mat::eye(4, 4, CV_32F);
    //std::vector<cv::Mat> Tcw_msf;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //yyh

};

Vec3f rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    //#if 1
    x = x*180.0f/3.141592653589793f;
    y = y*180.0f/3.141592653589793f;
    z = z*180.0f/3.141592653589793f;
    //#endif
    return Vec3f(x, y, z);
}

double str2double(const string &s) {
    stringstream ss(s);
    double d;
    ss >> d;
    return d;
}
//static ORB_SLAM2::ViewerAR viewerAR;
static AR ar;
static thread tViewer, t_ar, t_cam, t_tracking, t_extractor;
//static cv::Mat K;
BlockQueue_t<imu_data> *mimu_buffer = new BlockQueue_t<imu_data>(50000);
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
    uint64_t timestamp;
    //cv::Mat frame;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //yyh
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
    double mptimestamp;
    //int nFeatures,nLevels,fIniThFAST,fMinThFAST,mbRGB;
    //float fScaleFactor;

public:

    ORBextractor *mpORBextractor, *mpORBextractorIni;
    BlockQueue<extractORB> *mpORBbuffer;
    volatile bool IsExtractor;
    int framesize = 0;
    double mptimestamp_start;

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
                //gettimeofday(&time2,NULL);
                cv::Mat mImGray;
                im.copyTo(mImGray);
                //imshow("kkkk",im);
                //usleep(10000);
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
                if ((mpstate == 0) || (mpstate == 1)) {
                    (*mpORBextractorIni)(mImGray, cv::Mat(), mvKeys, mDescriptors);
                } else {
                    (*mpORBextractor)(mImGray, cv::Mat(), mvKeys, mDescriptors);
                }
                ORBData.Keypoints = mvKeys;
                ORBData.Descriptors = mDescriptors;
                ORBData.timestamp = mptimestamp;
                mpORBbuffer->Put(ORBData);

                //gettimeofday(&time3,NULL);
                //ms2 = (time3.tv_sec*1000000+time3.tv_usec) - (time2.tv_sec*1000000+time2.tv_usec);
                //cout<<"ORB提取时间 = "<<ms2<<endl;
            } else {
                usleep(1000);
            }
        }
    }


    void setExtractor(cv::Mat im_, int state_ , double timestamp,double timestamp_start) {
        im = im_.clone();
        mpstate = state_;
        mptimestamp = timestamp;
        mptimestamp_start = timestamp_start;
    }
};

//捕捉摄像头图片类
class Cam {
public:
    Cam(int w, int h, float fps) {
        cam_rdy = false;
        capture.open(0);
        if (!capture.isOpened()) {
            cout << "camera open failed" << endl;
            exit(-1);
        }
        cout << "Set VideoCapture:" << endl;
        capture.set(CV_CAP_PROP_FRAME_WIDTH, w);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, h);
        capture.set(CV_CAP_PROP_FPS, fps);
        cout << "\tsize:\t[" << capture.get(CV_CAP_PROP_FRAME_WIDTH) << "*" << capture.get(CV_CAP_PROP_FRAME_HEIGHT)
             << "]" << endl;
        cout << "\tfps:\t" << capture.get(CV_CAP_PROP_FPS) << endl;
        loop = true;
    }

    cv::VideoCapture capture;
    cv::Mat im_bgr, im_rgb;
    bool cam_rdy;
    bool loop;
    int capturesize = 0;

    void run() {
        while (loop) {
            capture >> im_bgr;
            capturesize++;

            cv::cvtColor(im_bgr, im_rgb, CV_BGR2RGB);
            cam_rdy = true;
        }
    }

    cv::Mat getFrame() {
        return im_rgb.clone();
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
    to_msf *mmsf;
    volatile bool put_imu;
    //vector<Meas::Ptr_mea> measurements;
    uint32_t seq_cnt_imu = 0;
    uint32_t seq_cnt_vicon = 0;
    uint64_t imu_timestamp_before;
    uint64_t vicon_timestamp_before;
    BlockQueue<uint64_t> *time_buffer = new BlockQueue<uint64_t>(5000);
    Eigen::Quaterniond C_Q;
    Eigen::Matrix3d C_Q_w_v;
    Eigen::Vector3d P_wv;
    //BlockQueue_t<imu_data> *mimu_buffer = new BlockQueue_t<imu_data>(50000);


public:

    ofstream f_msf;

    Track(bool mapped, ORB_SLAM2::System *system, AR *ar, Extractor *extractor/*, to_msf *msf, BlockQueue_t<imu_data> *imu_buffer*/) {
        mSystem = system;
        mAr = ar;
        mextractor = extractor;
        mapped_ = mapped;
        //mmsf = msf;
        put_imu = false;
        //BlockQueue_t<imu_data> *mimu_buffer = imu_buffer;

    }

    ~Track() { ; }

    void run() {
        mpData mpDataStruct;
        extractORB mpORBData;
//        int timestamp = 0;
//        cv::Mat Twp;
//        float ox,oy,oz,rx,ry,rz;
//        const char *outdir = nullptr;
//        outdir = "./dataset";
//        mynteye::tools::Dataset dataset(outdir);
        //vector<Imu7> mvt_imu;
        uint32_t msf_cnt = 0;
        imu_data mimudata_buffer;
        cv::Mat mTcw;

        cv::Mat R1w;
        cv::Mat t1w;
        cv::Mat T1w;
        //uint32_t vicon_cnt = 0;
        //mimu_buffer = new BlockQueue_t<imu_data>(50000);

        while (1) {
            static ofstream fout_tcw_msf("/home/gzh/data/tcw_msf_d.txt");
            static ofstream fout_tcw_track("/home/gzh/data/tcw_visual_initial_data16.txt");
            cout<<"****"<<endl;
            mextractor->mpORBbuffer->Take(mpORBData);

            cv::Mat frame;

            if (mpORBData.idx_image == 0) {
                frame = (mpORBData.ImageArray[BUFFERSIZE1 - 1]).clone();
            } else {
                frame = (mpORBData.ImageArray[mpORBData.idx_image - 1]).clone();
            }

            // cout<<"1"<<endl;
            uint64_t mtimestamp = mpORBData.timestamp;
            uint64_t mtimestamp_start = mextractor->mptimestamp_start;
            mpDataStruct.ImageArray2[mpDataStruct.idx_image] = frame.clone();
            mpDataStruct.idx_image++;
            mpDataStruct.idx_image = mpDataStruct.idx_image % BUFFERSIZE;

            //cout<<"mtimestamp_start"<< mtimestamp_start<<endl;
            cout<<"+++"<<endl;

            gettimeofday(&time4, NULL);
            cout<<"---"<<endl;
            cv::Mat Tcw_track = mSystem->TrackMonocular(frame, mpORBData.Keypoints, mpORBData.Descriptors, mtimestamp);
            gettimeofday(&time5, NULL);
            std::cout<<"Tcw_track:"<<" "<<Tcw_track<<std::endl;
//            vector<cv::KeyPoint> vKeys = mSystem->GetTrackedKeyPointsUn();
//            vector<ORB_SLAM2::MapPoint *> vMPs = mSystem->GetTrackedMapPoints();
//            mSystem->mpViewer->SetImagePose(frame,Tcw_track,state,vKeys,vMPs);

            //cv::Mat Tcw;//print
            //std::cout<<mtimestamp<<" "<<mtimestamp_start<<"***************************************"<<std::endl;
            if(!Tcw_track.empty()) {

//                Eigen::Matrix3d Rcw_track;
//                Rcw_track << Tcw_track.at<float>(0, 0), Tcw_track.at<float>(0, 1), Tcw_track.at<float>(0, 2),
//                        Tcw_track.at<float>(1, 0), Tcw_track.at<float>(1, 1), Tcw_track.at<float>(1, 2),
//
//                        Tcw_track.at<float>(2, 0), Tcw_track.at<float>(2, 1), Tcw_track.at<float>(2, 2);
//                Eigen::Quaterniond Q_t(Rcw_track);
//                fout_tcw_track << mtimestamp << " " << Q_t.x() << " " << Q_t.y() << " " << Q_t.z() << " " << Q_t.w()
//                               << " " << Tcw_track.at<float>(0, 3) << " " << Tcw_track.at<float>(1, 3) << " "
//                               << Tcw_track.at<float>(2, 3) << endl;


                cv::Mat Rcw = Tcw_track.rowRange(0, 3).colRange(0, 3);    //rotation matrix
                cv::Mat tcw = Tcw_track.rowRange(0, 3).col(3);        //translation matrix
                Eigen::Matrix3f rotation_matrix;

                cv::Mat Rwc = Rcw.t();
                cv::Mat twc = -Rwc * tcw;

                rotation_matrix << Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
                        Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
                        Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2);

                Eigen::Quaternionf Q(rotation_matrix);
                fout_tcw_track << mtimestamp << " " <<setprecision(9)<< twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << Q.x()<< " " << Q.y() << " " <<Q.z() << " "<< Q.w() << endl;
            }
            if (mapped_) {

                if (!Tcw_track.empty()) {
                    cout<<"111"<<endl;
                    campose(Tcw_track, &mpDataStruct);//更换位姿

                }
                cout<<"222"<<endl;
                mAr->mpDatabuffer->Put(mpDataStruct);
                cout<<"333"<<endl;
                state = mSystem->GetTrackingState();
                cout<<"444"<<endl;
                if (state == 3) state = 0;
                else if (state == 2) state = 1;
                mAr->mploader->objs[0]->visible = true;

            }
            cout<<"555"<<endl;
            vector<cv::KeyPoint> vKeys = mSystem->GetTrackedKeyPointsUn();
            vector<ORB_SLAM2::MapPoint *> vMPs = mSystem->GetTrackedMapPoints();
            mSystem->mpViewer->SetImagePose(frame, Tcw_track, state, vKeys, vMPs);
            cout<<"666"<<endl;
//
//
//            if (!Tcw_track.empty() && mtimestamp >= mtimestamp_start) {
//
//                msf_cnt++;
//                cout<<"enter if "<<endl;
//                mimu_buffer->Take_t(mimudata_buffer);//print
//               // std::cout<<"====================================="<<mimudata_buffer.imu_timestamp<<"---------------"<<mtimestamp<<std::endl;
//                while (mimudata_buffer.imu_timestamp <= mtimestamp) {
//                    //std::cout<<"***************************************"<<std::endl;
//                    IMUMeas::Ptr_mea mea_imu(new IMUMeas);
//                    //std::cout<<"***************************************"<<std::endl;
//                    if (msf_cnt == 1) {
//                        if (mimudata_buffer.imu_timestamp <= mtimestamp - 5000000) {
//                               //std::cout<<"don't push"<<std::endl;
//                        } else {
//                            mmsf->to_setGravity(mimudata_buffer.acceleration);/* * 9.8*/;
//
//                            //MSF::setGravity(imudata_buffer.acceleration) /* * 9.8*/;
//                            mea_imu->seq = seq_cnt_imu++;
////                            std::cout<<"don't push"<<" "<<msf_core::constants::GRAVITY[0]<<" "
////                                     <<msf_core::constants::GRAVITY[1]<<" "<<msf_core::constants::GRAVITY[2]<<" "<<std::endl;
//                            //1349442751013921022 -4.37526 4.39488 9.35874 -2.10120188648 0.643241095823 -0.307614280664
//                            mea_imu->timestamp = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;
//                            mea_imu->linear_acceleration[0] = mimudata_buffer.acceleration[0] /* * 9.8*/;
//                            mea_imu->linear_acceleration[1] = mimudata_buffer.acceleration[1] /* * 9.8*/;
//                            mea_imu->linear_acceleration[2] = mimudata_buffer.acceleration[2] /* * 9.8*/;
//                            mea_imu->angular_velocity[0] = mimudata_buffer.gyr_velocity[0] /* * 3.1415 / 180*/;
//                            mea_imu->angular_velocity[1] = mimudata_buffer.gyr_velocity[1] /* * 3.1415 / 180*/;
//                            mea_imu->angular_velocity[2] = mimudata_buffer.gyr_velocity[2] /* * 3.1415 / 180*/;
//                            //std::cout << "MSF_imu" << mea_imu->linear_acceleration[0] << std::endl;
//                            //std::cout<<"push successful"<<std::endl;
//                            mmsf->to_measurements.push_back(mea_imu);
//                            //std::cout<<"successful"<<std::endl;
//
//                        }
//
//                    } else {
//
//                        mea_imu->seq = seq_cnt_imu++;
//                        //1349442751013921022 -4.37526 4.39488 9.35874 -2.10120188648 0.643241095823 -0.307614280664
//                        mea_imu->timestamp = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;
//                        mea_imu->linear_acceleration[0] = mimudata_buffer.acceleration[0]/* * 9.8*/;
//                        mea_imu->linear_acceleration[1] = mimudata_buffer.acceleration[1]/* * 9.8*/;
//                        mea_imu->linear_acceleration[2] = mimudata_buffer.acceleration[2] /* * 9.8*/;
//                        mea_imu->angular_velocity[0] = mimudata_buffer.gyr_velocity[0] /* * 3.1415 / 180*/;
//                        mea_imu->angular_velocity[1] = mimudata_buffer.gyr_velocity[1] /* * 3.1415 / 180*/;
//                        mea_imu->angular_velocity[2] = mimudata_buffer.gyr_velocity[2]/* * 3.1415 / 180*/;
//                        //std::cout << "MSF_imu" << mea_imu->linear_acceleration[0] << std::endl;
//                        mmsf->to_measurements.push_back(mea_imu);
//                    }
//
//
//                    mimu_buffer->Take_t(mimudata_buffer);
//                }
//                //std::cout << "MSF_imu_over"  << std::endl;
//                {
//                    //mimu_buffer->Take_t(mimudata_buffer);
//                    //std::cout<<"***************************************"<<mimudata_buffer.imu_timestamp<<std::endl;
//                    IMUMeas::Ptr_mea mea_imu(new IMUMeas);
//                    imu_timestamp_before = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;//不更新
//                    //std::cout<<"--------------------"<<std::endl;
//                    time_buffer->Put(imu_timestamp_before);//print
//                    //std::cout<<"/////////////////////////////"<<std::endl;
//                    mea_imu->seq = seq_cnt_imu++;
//                    //1349442751013921022 -4.37526 4.39488 9.35874 -2.10120188648 0.643241095823 -0.307614280664
//                    mea_imu->timestamp = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;
//                    mea_imu->linear_acceleration[0] = mimudata_buffer.acceleration[0]/* * 9.8*/;
//                    mea_imu->linear_acceleration[1] = mimudata_buffer.acceleration[1] /* * 9.8*/;
//                    mea_imu->linear_acceleration[2] = mimudata_buffer.acceleration[2]/* * 9.8*/;
//                    mea_imu->angular_velocity[0] = mimudata_buffer.gyr_velocity[0]/* * 3.1415 / 180*/;
//                    mea_imu->angular_velocity[1] = mimudata_buffer.gyr_velocity[1]/* * 3.1415 / 180*/;
//                    mea_imu->angular_velocity[2] = mimudata_buffer.gyr_velocity[2] /* * 3.1415 / 180*/;
//                    //std::cout << "MSF_imu another" << std::endl;
//                    mmsf->to_measurements.push_back(mea_imu);
//                    //std::cout<<"/////////////////////////////"<<std::endl;
//                }
//
//
//                {
//                    std::cout<<"/////////////////////////////"<<std::endl;
//
//                    VICONMeas::Ptr_mea mea_vicon(new VICONMeas);
//                    cv::Mat Rcw = Tcw_track.rowRange(0, 3).colRange(0, 3);    //rotation matrix
//                    cv::Mat tcw = Tcw_track.rowRange(0, 3).col(3);        //translation matrix
//                    Eigen::Matrix3f rotation_matrix;
//
//                    cv::Mat Rwc = Rcw.t();
//                    cv::Mat twc = -Rwc * tcw;
//
//                    rotation_matrix << Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
//                            Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
//                            Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2);
//
//                    Eigen::Quaternionf quat(rotation_matrix);
//                    std::cout<<"~~~~~~~~~~~~~~"<<quat.x()<<"\t"<<quat.y()<<"\t"<<quat.z()<<"\t"<<std::endl;
//
//                    //1349442751009001180 0.193471895296 0.0169222941425 1.19366082671 0.259788004921 0.220592089619 -0.0481239682821 0.93890010447
//                    //std::cout<<"***************************************"<<std::endl;
//                    if (msf_cnt == 1) {
//                        C_Q.x() = quat.x();
//                        C_Q.y() = quat.y();
//                        C_Q.z() = quat.z();
//                        C_Q.w() = quat.w();
//                        //Eigen::Matrix3d C_Q_w_v = C_Q.conjugate().toRotationMatrix();
//                        R1w = Rcw;//R1w
//                        t1w = tcw;//t1w
//                        T1w = Tcw_track;
//
//
//                        C_Q_w_v = C_Q.conjugate().toRotationMatrix();
//                        P_wv[0] = twc.at<float>(0);
//                        P_wv[1] = twc.at<float>(1);
//                        P_wv[2] = twc.at<float>(2);
//                        vicon_timestamp_before = mtimestamp /* * 10000) + 1111110000000000000*/;
//                        cout<<"******"<<mea_vicon->timestamp<<endl;
//                    }
//
//                    mea_vicon->timestamp = mtimestamp /* * 10000) + 1111110000000000000*/;//在静止的第一帧基础上
//                    mea_vicon->p[0] = twc.at<float>(0);
//                    mea_vicon->p[1] = twc.at<float>(1);
//                    mea_vicon->p[2] = twc.at<float>(2);// -
//                    mea_vicon->q.coeffs()[0] = quat.x(); //-
//                    mea_vicon->q.coeffs()[1] = quat.y(); //-
//                    mea_vicon->q.coeffs()[2] = quat.z();
//                    mea_vicon->q.coeffs()[3] = quat.w();
//                    mea_vicon->seq = seq_cnt_vicon++;
//                    if (mea_vicon->timestamp >= vicon_timestamp_before) {
//                        if (mea_vicon->timestamp - vicon_timestamp_before > 150000000) {
//                            std::cout << mea_vicon->timestamp << " ××××××××××××××××××××lost " << vicon_timestamp_before
//                                      << std::endl;
//                        }
//                        vicon_timestamp_before = mea_vicon->timestamp;
//                    }
//                    f_msf << mea_vicon->timestamp << ", " << twc.at<float>(0) << ", " << twc.at<float>(1) << ", "
//                          << twc.at<float>(2)
//                          << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << endl;
//
//                    //   std::cout<<  mea_vicon->timestamp << " " <<quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()  <<" "<< twc.at<float>(0) <<" "<< twc.at<float>(1) <<" "<< twc.at<float>(2)<<std::endl;
//
//                    Eigen::Matrix3d C_Q_p = mea_vicon->q.toRotationMatrix();
//                    C_Q_p = C_Q_w_v * C_Q_p;
//                    Eigen::Quaterniond Q_p(C_Q_p);
//                    mea_vicon->q = Q_p;
//                    mea_vicon->p = C_Q_w_v * mea_vicon->p - P_wv;
//
//
//                    //std::cout<<"////////////////"<<mea_vicon->q.x()<<"\t"<<mea_vicon->q.y()<<"\t"<<mea_vicon->q.z()<<"\t"<<std::endl;
//
//                    mmsf->to_measurements.push_back(mea_vicon);
//                    //std::cout<<"++++++++++++++++++++++"<<std::endl;
//
//                }
////                for(auto &it: mmsf->to_measurements){
////
////                    if (it->type() == Meas::IMU) {
////
////                        IMUMeas::Ptr_mea imuMsg = std::static_pointer_cast<IMUMeas>(it);
////
////                        std::cout << "imuMsg->timestamp："<<imuMsg->timestamp << std::endl;
////
////                    } else if (it->type() == Meas::VICON) {
////                        VICONMeas::Ptr_mea viconMsg = std::static_pointer_cast<VICONMeas>(it);
////                        std::cout << "viconMsg->timestamp："<<viconMsg->timestamp << std::endl;
////                    }
////                }
////                std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
//                std::sort(mmsf->to_measurements.begin(), mmsf->to_measurements.end(), [](const Meas::Ptr_mea &lhs, const Meas::Ptr_mea &rhs) {
//                    return lhs->timestamp < rhs->timestamp;
//                });
////                for(auto &it: mmsf->to_measurements){
////
////                    if (it->type() == Meas::IMU) {
////
////                        IMUMeas::Ptr_mea imuMsg = std::static_pointer_cast<IMUMeas>(it);
////
////                        std::cout << "imuMsg->timestamp："<<imuMsg->timestamp << std::endl;
////
////                    } else if (it->type() == Meas::VICON) {
////                        VICONMeas::Ptr_mea viconMsg = std::static_pointer_cast<VICONMeas>(it);
////                        std::cout << "viconMsg->timestamp："<<viconMsg->timestamp << std::endl;
////                    }
////                }
//                //std::cout<<"***************************************"<<std::endl;
//                if (msf_cnt == 1) {
//                    mmsf->to_MSF_data();
//                    //std::cout<<"11111"<<std::endl;
//                    mmsf->to_MSF_init();
//
//                    //std::cout<<"2222"<<std::endl;
//                    mmsf->to_MSF_process();
//                    //std::cout<<"/////////////////////////////"<<std::endl;
//                } else {
//                    mmsf->to_MSF_data();
//                    //std::cout<<"=========================="<<std::endl;
//                    mmsf->to_MSF_process();
//                    //std::cout<<"5555555555555555555555"<<std::endl;
//                    mTcw = mmsf->to_getTcw_msf();
////                    if(mTcw.empty()){
////                        continue;
////                    }
////                    cv::Mat mRcw = mTcw.rowRange(0, 3).colRange(0, 3);    //rotation matrix
////                    cv::Mat mtcw = mTcw.rowRange(0, 3).col(3);        //translation matrix
////                    mtcw = mRcw*t1w + mtcw;
////                    mRcw = mRcw * R1w;
////
////                    mTcw.rowRange(0, 3).colRange(0, 3) = mRcw;
////                    mTcw.rowRange(0, 3).col(3) = mtcw;
//                        mTcw = mTcw * T1w;
//                    //std::cout<<"msf_track:"<<" "<<mTcw<<std::endl;
//
//
//
////                    Eigen::Quaterniond Q(Rcw);
////                    std::cout << Q.x() << "\t" << Q.y() << "\t" << Q.z() << "\t" << Q.w() << "\t" << mTcw.at<float>(0, 3)
////                              << "\t" << mTcw.at<float>(1, 3) << "\t" << mTcw.at<float>(2, 3) << std::endl;
////                    vector<cv::KeyPoint> vKeys = mSystem->GetTrackedKeyPointsUn();
////                    vector<ORB_SLAM2::MapPoint *> vMPs = mSystem->GetTrackedMapPoints();
////                    mSystem->mpViewer->SetImagePose(frame,Tcw,state,vKeys,vMPs);
//
////                    Eigen::Matrix3d Rcw_track;
////                    Rcw_track << Tcw_track.at<float>(0, 0), Tcw_track.at<float>(0, 1), Tcw_track.at<float>(0, 2),
////                            Tcw_track.at<float>(1, 0), Tcw_track.at<float>(1, 1), Tcw_track.at<float>(1, 2),
////
////                            Tcw_track.at<float>(2, 0), Tcw_track.at<float>(2, 1), Tcw_track.at<float>(2, 2);
////                    Eigen::Quaterniond Q_track(Rcw_track);
////                    std::cout << Q_track.x() << "\t" << Q_track.y() << "\t" << Q_track.z() << "\t" << Q_track.w() << "\t" << Tcw_track.at<float>(0, 3)
////                              << "\t" << Tcw_track.at<float>(1, 3) << "\t" << Tcw_track.at<float>(2, 3) << std::endl;
////                    std::cout<<"======================================="<<std::endl;
////                    std::cout << Q.x()-Q_track.x() << "\t" <<Q.y()- Q_track.y() << "\t" << Q.z()- Q_track.z() << "\t" << Q_track.w() << "\t" << Tcw_track.at<float>(0, 3)
////                              << "\t" << Tcw_track.at<float>(1, 3) << "\t" << Tcw_track.at<float>(2, 3) << std::endl;
//
////                    if(!mTcw.empty()) {
////
////                        Eigen::Matrix3d mRcw;
////                        mRcw << mTcw.at<float>(0, 0), mTcw.at<float>(0, 1), mTcw.at<float>(0, 2),
////                                mTcw.at<float>(1, 0), mTcw.at<float>(1, 1), mTcw.at<float>(1, 2),
////
////                                mTcw.at<float>(2, 0), mTcw.at<float>(2, 1), mTcw.at<float>(2, 2);
////                        Eigen::Quaterniond Q1(mRcw);
////                        fout_tcw << mtimestamp << " " << Q1.x() << " " << Q1.y() << " " << Q1.z() << " " << Q1.w() << " "
////                                 << mTcw.at<float>(0, 3) << " " << mTcw.at<float>(1, 3) << " " << mTcw.at<float>(2, 3)
////                                 << endl;
////                    }
//
//
//
//                }
//
//
////                if(mtimestamp <= mtimestamp_start + 14160990080/*77281719936*/){
////                    mTcw = Tcw_track;
////                    //cout<<"------------------------------"<<endl;
////                }
//
//
//                //Vec3f angles_track = rotationMatrixToEulerAngles(Rcw_track);
//                //Vec3f angles = rotationMatrixToEulerAngles(Rcw);
//                // std::cout << angles_track[0]<< "\t"<<angles[0] << "\t" <<angles_track[1]<< "\t"<<angles[1] << "\t" <<angles_track[2]<< "\t"<<angles[2] <<endl;
//                //std::cout <<"*************************************"<<endl;
////                std::cout <<"x: "<< angles_track[0]-angles[0] << "\t" <<"y: "<<angles_track[1]-angles[1] << "\t" <<"z: "<<angles_track[2]-angles[2] /*<< "\t" << Tcw_track.at<float>(0, 3)- mTcw.at<float>(0, 3)
////                          << "\t" << Tcw_track.at<float>(1, 3)- mTcw.at<float>(1, 3) << "\t" << Tcw_track.at<float>(2, 3)- mTcw.at<float>(2, 3) */<< std::endl;
//
//
//
//
//            }
////            if ((!Tcw_track.empty() && mtimestamp < mtimestamp_start)) {
////                mTcw = Tcw_track;
////            }
////            if(Tcw_track.empty()) {
////                cout << "+++++++++++++++++++++++++++++" << endl;
////            }
////            else{
////                mTcw = Tcw_track;
////            }
//
////            if(!mTcw.empty()) {
////
////            }
//
//            //std::cout<<"msf_track:"<<" "<<mTcw<<std::endl;
//


        }

    };
};
//主函数
int main(int argc, char **argv) {
        // BlockQueue<imu_data> *imu_buffer;

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
        //MSF msf;/home/gzh/data/zheda/camera/sensor.yaml
        //to_msf nmsf("/home/gzh/ORB_SLAM_MSF-Dataset_vio/Examples/MSF/dataset/pose_msf/vicon.yaml");
        //to_msf nmsf("/home/gzh/ORB_SLAM_MSF-Dataset_vio/Examples/MSF/dataset/pose_msf/vicon.yaml");
        //to_msf nmsf("/home/gzh/data/zheda/vicon/sensor.yaml");
//        BlockQueue_t<imu_data> *imu_buffer;
//
//        imu_buffer = new BlockQueue_t<imu_data>(50000);
        //string path_to_data("/home/gzh/data/mav2/");
    //string path_to_data("/home/gzh/data/mav1/mav0/");
    string path_to_data("/home/gzh/data/data16/");
        vector<string> vs_ImDir;
        vector<double> vd_TimeStamps;
        string str_line, s;
        //ifstream f_cam(path_to_data + "cam0/data.csv");
    ifstream f_cam(path_to_data + "left/stream.csv");
        getline(f_cam, str_line);
        while (getline(f_cam, str_line)) {
            if (str_line[str_line.length() - 1] == '\r') {
                str_line[str_line.length() - 1] = '\0';
            }
            stringstream ss(str_line);
            getline(ss, s, ',');
            vd_TimeStamps.push_back(str2double(s));
            getline(ss, s, ',');
            vs_ImDir.push_back(s);
        }
//        cout << "load path of data" << endl;
//
//        ifstream f_imu(path_to_data + "imu0.csv");
//        vector<Imu7> vt_imu;
//        string str_line2, s2;
//        getline(f_imu, str_line2);
//        while (getline(f_imu, str_line2)) {
//            if (str_line2[str_line2.length() - 1] == '\r') {
//                str_line2[str_line2.length() - 1] = '\0';
//            }
//            stringstream ss(str_line2);
//            Imu7 imu7;
//
//            getline(ss, s2, ',');
//            imu7.time_imu = str2double(s2);
//            getline(ss, s2, ',');
//            imu7.gyr_imu0 = str2double(s2);
//            getline(ss, s2, ',');
//            imu7.gyr_imu1 = str2double(s2);
//            getline(ss, s2, ',');
//            imu7.gyr_imu2 = str2double(s2);
//            getline(ss, s2, ',');
//            imu7.acc_imu0 = str2double(s2);
//            getline(ss, s2, ',');
//            imu7.acc_imu1 = str2double(s2);
//            getline(ss, s2, ',');
//            imu7.acc_imu2 = str2double(s2);
//
//
//            vt_imu.push_back(imu7);
//
//        }
//        size_t sz_imu = vt_imu.size();
//
//        imu_data mpimudata_buffer;
//        if (sz_imu > 0) {
//            std::cout << "Imu count: " << sz_imu << std::endl;
//            for (size_t i = 0; i < sz_imu; i++) {
//
//                mpimudata_buffer.imu_timestamp = vt_imu[i].time_imu;
//                mpimudata_buffer.gyr_velocity[0] = vt_imu[i].gyr_imu0;
//                mpimudata_buffer.gyr_velocity[1] = vt_imu[i].gyr_imu1;
//                mpimudata_buffer.gyr_velocity[2] = vt_imu[i].gyr_imu2;
//                mpimudata_buffer.acceleration[0] = vt_imu[i].acc_imu0;
//                mpimudata_buffer.acceleration[1] = vt_imu[i].acc_imu1;
//                mpimudata_buffer.acceleration[2] = vt_imu[i].acc_imu2;
//
//                if (mpimudata_buffer.acceleration[0] == 0 && mpimudata_buffer.acceleration[1] == 0 &&
//                    mpimudata_buffer.acceleration[2] == 0) {
//                    std::cout << "[acce] : 0" << std::endl;
//                } else {
//                    // dataset.SaveMotionData(data);
//                    //std::cout << mpimudata_buffer.imu_timestamp<< std::endl;
//                    //std::cout << "**************************before_Put_imu"<< std::endl;
//                    mimu_buffer->Put_t(mpimudata_buffer);
//                    //std::cout << "**************************put_imu"<< std::endl;
//                }
//
//
//            }
//            std::cout << std::endl;
//        } else {
//            std::cout << "Imu count: " << sz_imu << std::endl;
//        }


        //set VideoCapture
        int w = fSettings["Camera.width"];
        int h = fSettings["Camera.height"];
        float fps = fSettings["Camera.fps"];
        cv::Mat frame;
/*
    cv::Mat Tcb;
    Tcb = (cv::Mat_<double>(4,4)<<0.99996651999999997,0.00430873000000000,0.00695718000000000,-0.04777362000000000108,
    0.00434878000000000, -0.99997400999999997, -0.00575128000000000,-0.00223730999999999991,
    0.00693222000000000, 0.00578135000000000, -0.99995926000000002,-0.00160071000000000008,
    0.0,0.0,0.0,1.0);
    cv::Mat Rcb = Tcb.rowRange(0,3).colRange(0,3);
    cv::Mat tcb = Tcb.rowRange(0,3).col(3);
    cv::Mat Rbc = Rcb.t();
    cv::Mat tbc = -Rbc*tcb;
    cv::Mat Tbc = cv::Mat::eye(4,4,Tcb.type()); Rbc.copyTo(Tbc.rowRange(0,3).colRange(0,3)); tbc.copyTo(Tbc.rowRange(0,3).col(3));
    cout<<"Tbc ["<<setprecision(18)<<Tbc.at<double>(0,0)<<"\t"<<Tbc.at<double>(0,1)<<"\t"<<Tbc.at<double>(0,2)<<"\t"<<Tbc.at<double>(0,3)<<"\n"<<
    Tbc.at<double>(1,0)<<"\t"<<Tbc.at<double>(1,1)<<"\t"<<Tbc.at<double>(1,2)<<"\t"<<Tbc.at<double>(1,3)<<"\n"<<
    Tbc.at<double>(2,0)<<"\t"<<Tbc.at<double>(2,1)<<"\t"<<Tbc.at<double>(2,2)<<"\t"<<Tbc.at<double>(2,3)<<"\n"<<
    Tbc.at<double>(3,0)<<"\t"<<Tbc.at<double>(3,1)<<"\t"<<Tbc.at<double>(3,2)<<"\t"<<Tbc.at<double>(3,3)<<endl;
    */

        double timestamps = 0;
        double time_start;
        //设置AR对象的参数
        if (mapped) {
            string yaml_path = fSettings["YAML_PATH"];
            float near = fSettings["near"];
            float far = fSettings["far"];
            int back_color[3] = {0, 0, 0};
            back_color[0] = fSettings["back_red"];
            back_color[1] = fSettings["back_green"];
            back_color[2] = fSettings["back_blue"];

            float fx = fSettings["Camera.fx"];
            float fy = fSettings["Camera.fy"];
            float cx = fSettings["Camera.cx"];
            float cy = fSettings["Camera.cy"];


            ar = AR(w, h, yaml_path, fps);
            ar.setProject(fx, fy, cx, cy, near, far);
            ar.back_color[0] = back_color[0];
            ar.back_color[1] = back_color[1];
            ar.back_color[2] = back_color[2];

            SLAM.mpViewer->ar = &ar;
            t_ar = thread(&AR::run, &ar);    //cout<<"ar\t"<<t_ar.get_id()<<endl;
            //K = (cv::Mat_<float>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
        }


        mpData mpDataStruct1;
        Extractor extractor(argv[2]);
        Track track(mapped, &SLAM, &ar, &extractor/*, &nmsf, imu_buffer*/);
        // MSF msf("/home/yyh/visiontech_slam1_MSF/Examples/MSF/dataset/pose_msf/vicon.yaml");
        //开启cam,extractor,track线程
        t_extractor = thread(&Extractor::run, &extractor);
        t_tracking = thread(&Track::run, &track);
        //t_msf = thread(&MSF::run, &msf);


        cv::Mat im;
        size_t sz_im = vd_TimeStamps.size();
        for (size_t i = 0; i < sz_im; i++) {
            // cout<<path_to_data+"data/"+vs_ImDir[i]<<endl;
//            im = cv::imread(path_to_data + "cam0/data/" + vs_ImDir[i], CV_LOAD_IMAGE_UNCHANGED);
            im = cv::imread(path_to_data + "left/" + vs_ImDir[i], CV_LOAD_IMAGE_UNCHANGED);
            //cv::flip(im,im,0);
            //cv::flip(im,im,1);
//        if(im.empty())
//        {
//           cout<<"im"<<im.rows<<"\t"<<im.cols<<endl;
//        }
            timestamps = vd_TimeStamps[i];
            if (i == 0) {
                time_start = timestamps + 21160990080/*77281719936*/;
                std::cout << "time_start  " << to_string(time_start) << std::endl;
            }
            cout << i << endl;
            // cout<<sz_im<<endl;
            extractor.setExtractor(im, SLAM.GetTrackingState(),timestamps,time_start);
            extractor.IsExtractor = true;
            //usleep(33000);
            usleep(33000);
            // cout<<sz_im<<endl;
        }





        //cout<<""<<newimage.size();
        cout << "extract size = " << extractor.framesize << endl;
        //cout << "capturesize = " << cam.counter << endl;
        gettimeofday(&time1, NULL);
        cout << "time1 = " << (time1.tv_sec * 1000000 + time1.tv_usec) << endl;
        ms1 = (time1.tv_sec * 1000000 + time1.tv_usec) - (time0.tv_sec * 1000000 + time0.tv_usec);
        cout << "time totle" << (int) ms1 << endl;
        cout << "per frame's average time = " << (float) ms1 / (float) extractor.framesize << endl;
        //SLAM.SaveTrajectory("/home/gzh/data/mono1.txt");

        if (mapped) {
            tViewer.join();
            t_ar.join();
        }

        // cam.loop = false;
        //cam.Close();
        //t_cam.join();
        //t_msf.join();
        f_cam.close();
        //f_imu.close();
        // Stop all threads
        SLAM.Shutdown();

        return 0;
    };

