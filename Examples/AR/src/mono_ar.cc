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

#include "System.h"
#include "AR.h"
#include "Viewer.h"
#include <ORBextractor.h>
#include "mynteyed/util/rate.h"
#include <my_types.h>
#include "to_msf.h"
//#include "iomanip.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include <iomanip>
#include <mynteyed/camera.h>
#include <mynteyed/utils.h>
#include "/home/lee/MYNT-EYE-D-SDK/samples/src/util/counter.h"
#include <sched.h>
#include <stdint-gcc.h>

MYNTEYE_USE_NAMESPACE
#define BUFFERSIZE1 2

using namespace std;
using namespace ORB_SLAM2;
using namespace mynteyed;
//static ORB_SLAM2::ViewerAR viewerAR;
static AR ar;
static thread tViewer, t_ar, t_cam, t_tracking, t_extractor;
Camera cam;
DeviceInfo dev_info;

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


bool judge(cv::Mat mat)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (i == j && mat.at<float>(i,j) != 1)
            {
                return false;
            }
            if (i != j && mat.at<float>(i,j) != 0)
            {
                return false;
            }
        }
        return true;

    }
}

BlockQueue_t<imu_data> *mimu_buffer = new BlockQueue_t<imu_data>(50000);
void campose(cv::Mat &Tcw, mpData *DataStruct, uint64_t timestamp) {
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

    DataStruct->pose_timestamp  = timestamp;
}
double calculate_stdev(vector<double> resultSet){
    double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
    double mean =  sum / resultSet.size(); //均值

    double accum  = 0.0;
    std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d) {
        accum  += (d-mean)*(d-mean);
    });

    double stdev = sqrt(accum/(resultSet.size()-1)); //方差
    return stdev;
}


struct extractORB {
    std::vector<cv::KeyPoint> Keypoints;
    cv::Mat Descriptors;
    cv::Mat ImageArray[BUFFERSIZE1];
    int idx_image = 0;
    uint64_t timestamp;
    //cv::Mat frame;
};

double str2double(const string &s) {
    stringstream ss(s);
    double d;
    ss >> d;
    return d;
}
struct timeval time0, time1, time2, time3, time4, time5,time6;
float ms1 = 0, ms2 = 0, ms3 = 0 , ms4 = 0;


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
    uint64_t mptimestamp;

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
                cv::Mat mImGray;
                im.copyTo(mImGray);
                ORBData.ImageArray[ORBData.idx_image] = im;
                //ORBData.ImageArray[ORBData.idx_image] = im.clone();
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
                ORBData.timestamp = mptimestamp;
                mpORBbuffer->Put(ORBData);

                ms2 = (time3.tv_sec*1000000+time3.tv_usec) - (time2.tv_sec*1000000+time2.tv_usec);
                cout<<" ORB提取时间 = "<<ms2<<endl;
            } else {
                usleep(1000);
            }
        }
    }

    void setExtractor(cv::Mat im_, int state_ , uint64_t timestamp/*,double timestamp_start*/) {
        //im = im_.clone();
        im = im_;
        mpstate = state_;
        mptimestamp = timestamp;
//        mptimestamp_start = timestamp_start;
    }
};

//捕捉摄像头图片类

class Cam {
public:

    bool cam_rdy ;
    cv::Mat im_rgb;
    int cnt ;
    util::Counter counter;
    uint64_t timestamp;


    Cam():cnt(0), cam_rdy(false){}
    void run() {
        for (;;) {
            counter.Update();

            auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
            if (left_color.img) {
                auto left_img = left_color.img->To(ImageFormat::COLOR_BGR);
                cv::Mat left(left_img->height(), left_img->width(), CV_8UC3,
                             left_img->data());
                //frame = left.clone();
                cv::Mat left_image = left_img->ToMat();
                cv::cvtColor(left_image, im_rgb, CV_BGR2RGB);
                timestamp = left_color.img_info->timestamp;
                cam_rdy = true;
//                cv::imshow("left", left_image);
                cnt++;

            }
        }
    }

    cv::Mat getFrame() {
        return im_rgb;
//         return im_rgb.clone();
    }
    std::uint32_t getTimestamp() {
        // if(cTimestamp){
        //LOGE("return");
        return timestamp;

        //}
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
    Extractor* mextractor;
    uint64_t mtimestamp_start;                      //第一个位姿的时间戳
    //vector<Meas::Ptr> measurements;
    uint32_t   seq_cnt_imu = 0;
    uint32_t   seq_cnt_vicon = 0;
    uint64_t imu_timestamp_before;
    uint64_t vicon_timestamp_before;
//    BlockQueue<uint64_t> *time_buffer;
    to_msf *mmsf;


public:


    Track(bool mapped, ORB_SLAM2::System *system, AR *ar, Extractor *extractor, to_msf *msf) {
        mSystem = system;
        mAr = ar;
        mextractor = extractor;
        mapped_ = mapped;
        mmsf = msf;
    }

    ~Track() { ; }

    void run() {
        mpData mpDataStruct;
        extractORB mpORBData;
//        imu_data mpimudata_buffer;
        uint32_t imu_cnt = 0;
        vector<imu_data> imu_modify;
        cv::Mat mTcw= Mat::eye(4, 4, CV_32F);
        vector<imu_data>gravity_list;
//        int timestamp = 0;
        uint32_t msf_cnt = 0;
        uint32_t msf_flag = 0;

        cv::Mat t1w;
        cv::Mat Tc1, T1c;
        cv::Mat T1w;
        cv::Mat Twc= Mat::eye(4, 4, CV_32F);            //融合位姿（直接从msf中得到的位姿）
        cv::Mat Tcw_msf = Mat::eye(4, 4, CV_32F);       //融合位姿（从Twc变换而来）
        bool tcw_flag = false;
        vector<double>ax_sum;
        vector<double>ay_sum;
        vector<double>az_sum;
        vector<uint32_t >gravity_flag;
        while (1) {
            cout << " start track thread " << endl;
            bool is_mTcw_msf = false;                               //最终位姿是融合位姿的 flag ( mTcw 就最终所用的 位姿）
            mextractor->mpORBbuffer->Take(mpORBData);
            static ofstream fout_tcw_msf("./tcw_msf.txt");
            static ofstream visual_track("./visual_track.txt");
//            static ofstream imu_output("./imu.txt");
//            static ofstream imu_output1("./imu1.txt");
            cv::Mat frame;
            if (mpORBData.idx_image == 0) {
                //frame = (mpORBData.ImageArray[BUFFERSIZE1 - 1]).clone();
                frame = (mpORBData.ImageArray[BUFFERSIZE1 - 1]);
            } else {
                //frame = (mpORBData.ImageArray[mpORBData.idx_image - 1]).clone();
                frame = (mpORBData.ImageArray[mpORBData.idx_image - 1]);
            }

            //mpDataStruct.ImageArray2[mpDataStruct.idx_image] = frame.clone();
            uint64_t mtimestamp = mpORBData.timestamp*1e4;
            mpDataStruct.ImageArray2[mpDataStruct.idx_image] = frame;
            mpDataStruct. img_timestamp = mpORBData.timestamp*1e4;
            cout << " put 进 mpDataStruct 的 img_timestamp 是 " << mpDataStruct.img_timestamp << endl;
            mpDataStruct.idx_image++;
            mpDataStruct.idx_image = mpDataStruct.idx_image % BUFFERSIZE;
            cout << " mpDataStruct.idx_image " << mpDataStruct.idx_image << endl;

            gettimeofday(&time4,NULL);
            cv::Mat Tcw_track = mSystem->TrackMonocular(frame, mpORBData.Keypoints, mpORBData.Descriptors, mtimestamp);
            gettimeofday(&time5,NULL);


            ms3 = (time5.tv_sec*1000000+time5.tv_usec) - (time4.tv_sec*1000000+time4.tv_usec);
            cout<<" 视觉track时间 = "<<ms3<<endl;

            if(!Tcw_track.empty()) {
                imu_data mimudata_buffer;
                Eigen::Vector3d acce(0,0,0);
                auto motion_datas = cam.GetMotionDatas();
//                if(tcw_flag==false)
//                    tcw_flag=true;
//                else {
                    if (motion_datas.size() > 0) {
                        //std::cout << "Imu count: " << motion_datas.size() << std::endl;
                        for (auto data : motion_datas) {
                            if (data.imu) {
                                imu_data mpimudata_buffer;
                                imu_data imudata_raw;

                                if (data.imu->flag == MYNTEYE_IMU_ACCEL) {
                                    imu_cnt++;
                                    if (imu_cnt == 1)
                                        imu_cnt = 0;
                                    else {
                                        imudata_raw.imu_timestamp = data.imu->timestamp;
                                        imudata_raw.acceleration[0] = data.imu->accel[0] * 9.8;
                                        imudata_raw.acceleration[1] = data.imu->accel[1] * 9.8;
                                        imudata_raw.acceleration[2] = data.imu->accel[2] * 9.8;
                                        imu_modify.push_back(imudata_raw);
                                        if (imu_cnt != 2) {

                                            mpimudata_buffer.imu_timestamp = imu_modify[1].imu_timestamp*1e4;
                                            mpimudata_buffer.acceleration[0] = imu_modify[1].acceleration[0];
                                            mpimudata_buffer.acceleration[1] = imu_modify[1].acceleration[1];
                                            mpimudata_buffer.acceleration[2] = imu_modify[1].acceleration[2];
                                            mpimudata_buffer.gyr_velocity[0] = (1 -
                                                                                (double) (imu_modify[1].imu_timestamp -
                                                                                          imu_modify[0].imu_timestamp) /
                                                                                (imu_modify[2].imu_timestamp -
                                                                                 imu_modify[0].imu_timestamp)) *
                                                                               imu_modify[0].gyr_velocity[0]
                                                                               + (double) (imu_modify[1].imu_timestamp -
                                                                                           imu_modify[0].imu_timestamp) /
                                                                                 (imu_modify[2].imu_timestamp -
                                                                                  imu_modify[0].imu_timestamp) *
                                                                                 imu_modify[2].gyr_velocity[0];
                                            mpimudata_buffer.gyr_velocity[1] = (1 -
                                                                                (double) (imu_modify[1].imu_timestamp -
                                                                                          imu_modify[0].imu_timestamp) /
                                                                                (imu_modify[2].imu_timestamp -
                                                                                 imu_modify[0].imu_timestamp)) *
                                                                               imu_modify[0].gyr_velocity[1]
                                                                               + (double) (imu_modify[1].imu_timestamp -
                                                                                           imu_modify[0].imu_timestamp) /
                                                                                 (imu_modify[2].imu_timestamp -
                                                                                  imu_modify[0].imu_timestamp) *
                                                                                 imu_modify[2].gyr_velocity[1];
                                            mpimudata_buffer.gyr_velocity[2] = (1 -
                                                                                (double) (imu_modify[1].imu_timestamp -
                                                                                          imu_modify[0].imu_timestamp) /
                                                                                (imu_modify[2].imu_timestamp -
                                                                                 imu_modify[0].imu_timestamp)) *
                                                                               imu_modify[0].gyr_velocity[2]
                                                                               + (double) (imu_modify[1].imu_timestamp -
                                                                                           imu_modify[0].imu_timestamp) /
                                                                                 (imu_modify[2].imu_timestamp -
                                                                                  imu_modify[0].imu_timestamp) *
                                                                                 imu_modify[2].gyr_velocity[2];
                                            if (mpimudata_buffer.gyr_velocity[0] == 0 &&
                                                mpimudata_buffer.gyr_velocity[1] == 0 &&
                                                mpimudata_buffer.gyr_velocity[2] == 0) {
                                                //std::cout << "[acce] : 0"<< std::endl;
                                            } else {
//                                    imu_output<<
//                                            imu_output<< setfill('1')<<setw(15)<<mpimudata_buffer.imu_timestamp<<"0000 "<<mpimudata_buffer.acceleration[0] << " "<<mpimudata_buffer.acceleration[1]<<" "<<mpimudata_buffer.acceleration[2]<<" "
//                                                      <<mpimudata_buffer.gyr_velocity[0]<< " "<<mpimudata_buffer.gyr_velocity[1] <<" "<< mpimudata_buffer.gyr_velocity[2] <<endl;
                                                mimu_buffer->Put_t(mpimudata_buffer);
//                                    mimu_buffer->Put_t(mpimudata_buffer);
//                                    dataset.SaveMotionData(data);
//                                    mmsf->imu_buffer->Put(mpimudata_buffer);
//                                    put_imu = true;

                                                //std::cout << "put_imu"<< std::endl;
                                            }
                                            imu_modify.erase(imu_modify.begin());
                                        }
                                    }
                                    // mpimudata_buffer.imu_timestamp = (data.imu->timestamp * 10000) + 1111110000000000000;
//                                acce[0] = data.imu->accel[0]*9.8;
//                                acce[1] = data.imu->accel[1]*9.8;
//                                acce[2] = data.imu->accel[2]*9.8;
                                    // mpimudata_buffer.gyr_velocity[0] = data.imu->gyro[0] * 3.1415 / 180;
                                    // mpimudata_buffer.gyr_velocity[1] = data.imu->gyro[1] * 3.1415 / 180;
                                    // mpimudata_buffer.gyr_velocity[2] = data.imu->gyro[2] * 3.1415 / 180;
//                                    imu_output1 << setfill('1') << setw(15) << data.imu->timestamp
//                                                << "0000,"
//                                                << data.imu->accel[0] * 9.8
//                                                << ","
//                                                << data.imu->accel[1] * 9.8
//                                                << ","
//                                                << data.imu->accel[2] * 9.8
//                                                << ",0,0,0 "
//                                                << endl;
                                    //mmsf->imu_buffer->Put(mpimudata_buffer);

                                } else if (data.imu->flag == MYNTEYE_IMU_GYRO) {
                                    imu_cnt++;
                                    imudata_raw.imu_timestamp = data.imu->timestamp;
                                    imudata_raw.gyr_velocity[0] = data.imu->gyro[0] / 180 * 3.1415926;
                                    imudata_raw.gyr_velocity[1] = data.imu->gyro[1] / 180 * 3.1415926;
                                    imudata_raw.gyr_velocity[2] = data.imu->gyro[2] / 180 * 3.1415926;
                                    imu_modify.push_back(imudata_raw);
                                    if (imu_cnt != 1) {

                                        mpimudata_buffer.imu_timestamp = imu_modify[1].imu_timestamp*1e4;
                                        mpimudata_buffer.acceleration[0] = imu_modify[1].acceleration[0];
                                        mpimudata_buffer.acceleration[1] = imu_modify[1].acceleration[1];
                                        mpimudata_buffer.acceleration[2] = imu_modify[1].acceleration[2];
                                        mpimudata_buffer.gyr_velocity[0] = (1 - (double) (imu_modify[1].imu_timestamp -
                                                                                          imu_modify[0].imu_timestamp) /
                                                                                (imu_modify[2].imu_timestamp -
                                                                                 imu_modify[0].imu_timestamp)) *
                                                                           imu_modify[0].gyr_velocity[0]
                                                                           + (double) (imu_modify[1].imu_timestamp -
                                                                                       imu_modify[0].imu_timestamp) /
                                                                             (imu_modify[2].imu_timestamp -
                                                                              imu_modify[0].imu_timestamp) *
                                                                             imu_modify[2].gyr_velocity[0];
                                        mpimudata_buffer.gyr_velocity[1] = (1 - (double) (imu_modify[1].imu_timestamp -
                                                                                          imu_modify[0].imu_timestamp) /
                                                                                (imu_modify[2].imu_timestamp -
                                                                                 imu_modify[0].imu_timestamp)) *
                                                                           imu_modify[0].gyr_velocity[1]
                                                                           + (double) (imu_modify[1].imu_timestamp -
                                                                                       imu_modify[0].imu_timestamp) /
                                                                             (imu_modify[2].imu_timestamp -
                                                                              imu_modify[0].imu_timestamp) *
                                                                             imu_modify[2].gyr_velocity[1];
                                        mpimudata_buffer.gyr_velocity[2] = (1 - (double) (imu_modify[1].imu_timestamp -
                                                                                          imu_modify[0].imu_timestamp) /
                                                                                (imu_modify[2].imu_timestamp -
                                                                                 imu_modify[0].imu_timestamp)) *
                                                                           imu_modify[0].gyr_velocity[2]
                                                                           + (double) (imu_modify[1].imu_timestamp -
                                                                                       imu_modify[0].imu_timestamp) /
                                                                             (imu_modify[2].imu_timestamp -
                                                                              imu_modify[0].imu_timestamp) *
                                                                             imu_modify[2].gyr_velocity[2];


                                        if (mpimudata_buffer.acceleration[0] == 0 &&
                                            mpimudata_buffer.acceleration[1] == 0 &&
                                            mpimudata_buffer.acceleration[2] == 0) {
//                                            std::cout << "[acce] : 0" << std::endl;
                                        } else {
//                                    imu_output<<
//                                        imu_output<< setfill('1')<<setw(15)<<mpimudata_buffer.imu_timestamp<<"0000 "<<mpimudata_buffer.acceleration[0] << " "<<mpimudata_buffer.acceleration[1]<<" "<<mpimudata_buffer.acceleration[2]<<" "
//                                                  <<mpimudata_buffer.gyr_velocity[0]<< " "<<mpimudata_buffer.gyr_velocity[1] <<" "<< mpimudata_buffer.gyr_velocity[2] <<endl;
//                                    mimu_buffer->Put_t(mpimudata_buffer);
                                            mimu_buffer->Put_t(mpimudata_buffer);
//                                    dataset.SaveMotionData(data);
//                                    mmsf->imu_buffer->Put(mpimudata_buffer);
//                                    put_imu = true;

                                            //std::cout << "put_imu"<< std::endl;
                                        }
                                        imu_modify.erase(imu_modify.begin());
//                                    if(imu_modify.size()==4){
//                                        mpimudata_buffer.imu_timestamp = imu_modify[2].imu_timestamp;
//                                        mpimudata_buffer.acceleration[0] = imu_modify[2].acceleration[0];
//                                        mpimudata_buffer.acceleration[1] = imu_modify[2].acceleration[1];
//                                        mpimudata_buffer.acceleration[2] = imu_modify[2].acceleration[2];
//                                        mpimudata_buffer.gyr_velocity[0] = (1-(double)(imu_modify[2].imu_timestamp-imu_modify[1].imu_timestamp)/(imu_modify[3].imu_timestamp-imu_modify[1].imu_timestamp))*imu_modify[1].gyr_velocity[0]
//                                                                           +(double)(imu_modify[2].imu_timestamp-imu_modify[1].imu_timestamp)/(imu_modify[3].imu_timestamp-imu_modify[1].imu_timestamp)*imu_modify[3].gyr_velocity[0];
//                                        mpimudata_buffer.gyr_velocity[1] = (1-(double)(imu_modify[2].imu_timestamp-imu_modify[1].imu_timestamp)/(imu_modify[3].imu_timestamp-imu_modify[1].imu_timestamp))*imu_modify[1].gyr_velocity[1]
//                                                                           +(double)(imu_modify[2].imu_timestamp-imu_modify[1].imu_timestamp)/(imu_modify[3].imu_timestamp-imu_modify[1].imu_timestamp)*imu_modify[3].gyr_velocity[1];
//                                        mpimudata_buffer.gyr_velocity[2] = (1-(double)(imu_modify[2].imu_timestamp-imu_modify[1].imu_timestamp)/(imu_modify[3].imu_timestamp-imu_modify[1].imu_timestamp))*imu_modify[1].gyr_velocity[2]
//                                                                           +(double)(imu_modify[2].imu_timestamp-imu_modify[1].imu_timestamp)/(imu_modify[3].imu_timestamp-imu_modify[1].imu_timestamp)*imu_modify[3].gyr_velocity[2];
//                                        imu_modify.erase(imu_modify.begin(), imu_modify.begin()+3);
//                                    } else if(imu_modify.size()==3){
//                                        imu_modify.erase(imu_modify.begin(), imu_modify.begin()+2);
//                                    }else{
//
//                                        std::cerr << "Imu is too many" << std::endl;
//                                    }


                                    }

//                                mpimudata_buffer.imu_timestamp = data.imu->timestamp;
//                                mpimudata_buffer.gyr_velocity[0] = data.imu->gyro[0]/180*3.1415926;
//                                mpimudata_buffer.gyr_velocity[1] = data.imu->gyro[1]/180*3.1415926;
//                                mpimudata_buffer.gyr_velocity[2] = data.imu->gyro[2]/180*3.1415926;
//                                mpimudata_buffer.acceleration[0] = acce[0];
//                                mpimudata_buffer.acceleration[1] = acce[1];
//                                mpimudata_buffer.acceleration[2] = acce[2];
//                                data.imu->accel[0] = acce[0];
//                                data.imu->accel[1] = acce[1];
//                                data.imu->accel[2] = acce[2];
//                                    imu_output1 << setfill('1') << setw(15) << data.imu->timestamp
//                                                << "0000,0,0,0,"
//                                                << data.imu->gyro[0] / 180 * 3.1415926
//                                                << ","
//                                                << data.imu->gyro[1] / 180 * 3.1415926
//                                                << ","
//                                                << data.imu->gyro[2] / 180 * 3.1415926
//                                                << endl;


                                } else {
                                    std::cerr << "Imu type is unknown" << std::endl;
                                }

                            } else {
                                std::cerr << "Motion data is empty" << std::endl;
                            }
                        }
//                        std::cout << std::endl;
                    }
//                }
//                rate.Sleep();


                cv::Mat Rcw_track = Tcw_track.rowRange(0, 3).colRange(0, 3);    //rotation matrix
                cv::Mat tcw_track = Tcw_track.rowRange(0, 3).col(3);        //translation matrix
                Eigen::Matrix3f rotation_matrix;

                cv::Mat Rwc_track = Rcw_track.t();
                cv::Mat twc_track = -Rwc_track * tcw_track;

                rotation_matrix << Rwc_track.at<float>(0, 0), Rwc_track.at<float>(0, 1), Rwc_track.at<float>(0, 2),
                        Rwc_track.at<float>(1, 0), Rwc_track.at<float>(1, 1), Rwc_track.at<float>(1, 2),
                        Rwc_track.at<float>(2, 0), Rwc_track.at<float>(2, 1), Rwc_track.at<float>(2, 2);

                Eigen::Quaternionf Q(rotation_matrix);
//                cout<<"-----------"<<get_length(mtimestamp)<<endl;
//                visual_track << setfill('1')<<setw(15)<<mtimestamp << "0000" << " "<<setprecision(9)<< twc_track.at<float>(0) << " " << twc_track.at<float>(1) << " " << twc_track.at<float>(2) << " " << Q.x()<< " " << Q.y() << " " <<Q.z() << " "<< Q.w() << endl;
                mimu_buffer->Take_t(mimudata_buffer);

                while (mimudata_buffer.imu_timestamp <= mtimestamp) {
                    IMUMeas::Ptr_mea mea_imu(new IMUMeas);
                    if (msf_cnt == 0 && msf_flag == 0) {
//                        cout<<"***"<<endl;
                        double stdev_x, stdev_y,stdev_z;
                        if(gravity_list.size()==50) {
                            gravity_list.erase(gravity_list.begin());
                            ax_sum.erase(ax_sum.begin());
                            ay_sum.erase(ay_sum.begin());
                            az_sum.erase(az_sum.begin());
                        }
                        //                    vector<imu_data>::iterator it;
                        gravity_list.push_back(mimudata_buffer);

                        //                    for(it=gravity_list.begin();it!=gravity_list.end();it++){
                        ax_sum.push_back(mimudata_buffer.acceleration[0]);
                        ay_sum.push_back(mimudata_buffer.acceleration[1]);
                        az_sum.push_back(mimudata_buffer.acceleration[2]);
                        //                    }
                        vector<double>::iterator it;
//                        for(it=ax_sum.begin();it!=ax_sum.end();it++){
//                            cout<<"ax_sum"<<*it<<endl;
//                        }
                        if(gravity_list.size()==50){
                            stdev_x = calculate_stdev(ax_sum);
                            stdev_y = calculate_stdev(ay_sum);
                            stdev_z = calculate_stdev(az_sum);
                            double stdev = stdev_x + stdev_y + stdev_z;
                            if(gravity_flag.size()==50)
                                gravity_flag.erase(gravity_flag.begin());
//                            cout<<"stdev:"<<stdev<<endl;
                            if(stdev < 0.07)
                                gravity_flag.push_back(1);
                            else
                                gravity_flag.push_back(0);
                            vector<uint32_t>::iterator it;
                            uint32_t g_num = 0;
                            for(it=gravity_flag.begin();it!=gravity_flag.end();it++)
                                g_num = g_num + *it;
//                            cout<<"g_num:"<<g_num<<endl;
                            if(g_num==50) {

                                if (mimudata_buffer.imu_timestamp <= mtimestamp - 4900000) {
                                    //    std::cout<<"don't push"<<std::endl;
                                    //                            cout<<"***"<<endl;

                                } else {
                                    cout << "imu:  " << mimudata_buffer.imu_timestamp << endl;
                                    mmsf->to_setGravity(mimudata_buffer.acceleration);   //第一次进入ｍｓｆ时设置重力
                                    cout << "set Gravity" << endl;
                                    msf_flag = 1;
                                    mea_imu->seq = seq_cnt_imu++;
                                    mea_imu->timestamp = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;
                                    mea_imu->linear_acceleration[0] = mimudata_buffer.acceleration[0] /* * 9.8*/;
                                    mea_imu->linear_acceleration[1] = mimudata_buffer.acceleration[1] /* * 9.8*/;
                                    mea_imu->linear_acceleration[2] = mimudata_buffer.acceleration[2] /* * 9.8*/;
                                    mea_imu->angular_velocity[0] = mimudata_buffer.gyr_velocity[0] /* * 3.1415 / 180*/;
                                    mea_imu->angular_velocity[1] = mimudata_buffer.gyr_velocity[1] /* * 3.1415 / 180*/;
                                    mea_imu->angular_velocity[2] = mimudata_buffer.gyr_velocity[2] /* * 3.1415 / 180*/;
                                    //  std::cout << "MSF_imu" << mea_imu->linear_acceleration[0] << std::endl;
                                    mmsf->to_measurements.push_back(
                                            mea_imu);           //将存放imu数据的mea_imu对象push_back进入容器to_measurements

                                }
                            }


                        }


                        //用来存放imu数据的临时对象
//                    if (msf_cnt == 1) {                                                           //找到离视觉图像最近的imu

                    } else {

                        mea_imu->seq = seq_cnt_imu++;
                        mea_imu->timestamp = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;
                        mea_imu->linear_acceleration[0] = mimudata_buffer.acceleration[0]/* * 9.8*/;
                        mea_imu->linear_acceleration[1] = mimudata_buffer.acceleration[1]/* * 9.8*/;
                        mea_imu->linear_acceleration[2] = mimudata_buffer.acceleration[2] /* * 9.8*/;
                        mea_imu->angular_velocity[0] = mimudata_buffer.gyr_velocity[0] /* * 3.1415 / 180*/;
                        mea_imu->angular_velocity[1] = mimudata_buffer.gyr_velocity[1] /* * 3.1415 / 180*/;
                        mea_imu->angular_velocity[2] = mimudata_buffer.gyr_velocity[2]/* * 3.1415 / 180*/;
                        mmsf->to_measurements.push_back(mea_imu);
                        //        cout << "taking imudata " << endl;
                    }
                    mimu_buffer->Take_t(mimudata_buffer);                       //多拿一个imu
                }
                if(msf_flag == 1) {
                    {
//                    cout<<"---------------"<<endl;
                        IMUMeas::Ptr_mea mea_imu(new IMUMeas);
                        imu_timestamp_before = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;//不更新
//                        time_buffer->Put(imu_timestamp_before);//print

                        mea_imu->seq = seq_cnt_imu++;
                        mea_imu->timestamp = mimudata_buffer.imu_timestamp/* * 10000) + 1111110000000000000*/;
                        mea_imu->linear_acceleration[0] = mimudata_buffer.acceleration[0]/* * 9.8*/;
                        mea_imu->linear_acceleration[1] = mimudata_buffer.acceleration[1] /* * 9.8*/;
                        mea_imu->linear_acceleration[2] = mimudata_buffer.acceleration[2]/* * 9.8*/;
                        mea_imu->angular_velocity[0] = mimudata_buffer.gyr_velocity[0]/* * 3.1415 / 180*/;
                        mea_imu->angular_velocity[1] = mimudata_buffer.gyr_velocity[1]/* * 3.1415 / 180*/;
                        mea_imu->angular_velocity[2] = mimudata_buffer.gyr_velocity[2] /* * 3.1415 / 180*/;
                        mmsf->to_measurements.push_back(mea_imu);

                    }
                    {
                        VICONMeas::Ptr_mea mea_vicon(new VICONMeas);                //用来存放视觉位姿的临时对象
                        cv::Mat Rcw = Tcw_track.rowRange(0, 3).colRange(0, 3);      //rotation matrix
                        cv::Mat tcw = Tcw_track.rowRange(0, 3).col(3);              //translation matrix
                        Eigen::Matrix3f rotation_matrix;

                        cv::Mat Rwc = Rcw.t();
                        cv::Mat twc = -Rwc * tcw;
//
                        rotation_matrix << Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
                                Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
                                Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2);

                        Eigen::Quaternionf quat(rotation_matrix);                   //quat 为Twc的归一化四元数形式
//                    Eigen::Quaternionf Qw1(rotation_matrix);                   //quat 为Twc的归一化四元数形式

                        if (msf_cnt == 0) {
//
                            Eigen::Matrix3d R1w;
                            R1w << Rcw.at<float>(0, 0), Rcw.at<float>(0, 1), Rcw.at<float>(0, 2),
                                    Rcw.at<float>(1, 0), Rcw.at<float>(1, 1), Rcw.at<float>(1, 2),
                                    Rcw.at<float>(2, 0), Rcw.at<float>(2, 1), Rcw.at<float>(2, 2);
                            Eigen::Vector3d t1w;
                            t1w << tcw.at<float>(0), tcw.at<float>(1), tcw.at<float>(2);
//
                            cout << " R1w = " << R1w << endl;
                            cout << " t1w = " << t1w << endl;
//
                            mmsf->to_setT1w(R1w, t1w);
                            vicon_timestamp_before = mtimestamp /* * 10000) + 1111110000000000000*/;
                        }
                        mea_vicon->timestamp = mtimestamp /* * 10000) + 1111110000000000000*/;//在静止的第一帧基础上
                        mea_vicon->p[0] = twc.at<float>(0);
                        mea_vicon->p[1] = twc.at<float>(1);
                        mea_vicon->p[2] = twc.at<float>(2);// -
                        mea_vicon->q.coeffs()[0] = quat.x(); //-
                        mea_vicon->q.coeffs()[1] = quat.y(); //-
                        mea_vicon->q.coeffs()[2] = quat.z();
                        mea_vicon->q.coeffs()[3] = quat.w();
                        mea_vicon->seq = seq_cnt_vicon++;
                        if (mea_vicon->timestamp >= vicon_timestamp_before) {
                            if (mea_vicon->timestamp - vicon_timestamp_before > 150000000) {
                                std::cout << mea_vicon->timestamp << " ××××××××××××××××××××lost "
                                          << vicon_timestamp_before
                                          << std::endl;
                            }
                            vicon_timestamp_before = mea_vicon->timestamp;
                        }
//
                        mmsf->to_measurements.push_back(mea_vicon);

                    }
                }
                if (msf_cnt == 0 && msf_flag == 1) {
                    mtimestamp_start = vicon_timestamp_before;
                    mmsf->to_MSF_data();
                    mmsf->to_MSF_init();
                    mmsf->to_MSF_process();
                    msf_cnt++;
                } else if(msf_cnt == 1 && msf_flag == 1) {
                    mmsf->to_MSF_data();
                    mmsf->to_MSF_process();

                        Twc = mmsf->to_getTcw_msf();
//                    cout << "------Twc:----" << Twc << endl;
                    if( mtimestamp - mtimestamp_start >= 5000000000 ) {
                        if (!Twc.empty() && !judge(Twc)) {
                            Eigen::Matrix3f rotation_matrix;
                            rotation_matrix << Twc.at<float>(0, 0), Twc.at<float>(0, 1), Twc.at<float>(0, 2),
                                    Twc.at<float>(1, 0), Twc.at<float>(1, 1), Twc.at<float>(1, 2),
                                    Twc.at<float>(2, 0), Twc.at<float>(2, 1), Twc.at<float>(2, 2);
                            Eigen::Quaternionf Q(rotation_matrix);
                            cv::Mat twc = Twc.rowRange(0, 3).col(3);        //translation matrix

//                        fout_tcw_msf << setfill('1')<<setw(15)<< mtimestamp <<"0000 "<< twc.at<float>(0)<< " " << twc.at<float>(1)<< " " << twc.at<float>(2) << " " << Q.x()<<" "<<Q.y()<<" "<<Q.z()<<" "<<Q.w()<<endl;
                            cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);    //rotation matrix
//                        cout << "------Rwc:----" << Rwc << endl;
//                        cout << "------twc:----" << twc << endl;

                            cv::Mat Rcw = Rwc.t();
//                        cout << "------Rcw:----" << Rcw << endl;

                            cv::Mat tcw = -Rcw * twc;
//                        cout << "------tcw:----" << tcw << endl;
                            Tcw_msf = (cv::Mat_<float>(4, 4) << Rcw.at<float>(0, 0), Rcw.at<float>(0, 1), Rcw.at<float>(
                                    0, 2), tcw.at<float>(0),
                                    Rcw.at<float>(1, 0), Rcw.at<float>(1, 1), Rcw.at<float>(1, 2), tcw.at<float>(1),
                                    Rcw.at<float>(2, 0), Rcw.at<float>(2, 1), Rcw.at<float>(2, 2), tcw.at<float>(2),
                                    0, 0, 0, 1);

                            mTcw = Tcw_msf;
                            is_mTcw_msf = true;
                            cout << " mTcw by Tcw_msf " << endl;

//                        cout << "------mTcw:----" << mTcw << endl;
                        }
                    }
                }
            }
            gettimeofday(&time6,NULL);
            ms4 = (time6.tv_sec*1000000+time6.tv_usec) - (time5.tv_sec*1000000+time5.tv_usec);
            cout << " msf的总时长是 " << ms4 << endl;
            if(!Tcw_track.empty() && is_mTcw_msf == false) {
                mTcw = Tcw_track;
                cout << " mTcw by Tcw_track " << endl;
            }
            if (mapped_) {
                if (!mTcw.empty()) {
                    campose(mTcw, &mpDataStruct,mtimestamp);
                }
                mAr->mpDatabuffer->Put(mpDataStruct);

                state = mSystem->GetTrackingState();
                if (state == 3) state = 0;
                else if (state == 2) state = 1;
                mAr->mploader->objs[0]->visible = true;
                vector<cv::KeyPoint> vKeys = mSystem->GetTrackedKeyPointsUn();
                vector<ORB_SLAM2::MapPoint *> vMPs = mSystem->GetTrackedMapPoints();
                mSystem->mpViewer->SetImagePose(frame,mTcw,state,vKeys,vMPs);
            }
            cout << " Tcw_track is " << endl << Tcw_track << endl;
            cout << " Tcw_msf is " << endl << Tcw_msf << endl;
            cout << " track thread ends " << endl;
            cout << endl;
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
    to_msf msf("/home/lee/mono_ar_slam_ok_集成/vicon_m.yaml");


    int w = fSettings["Camera.width"];
    int h = fSettings["Camera.height"];
    float fps = fSettings["Camera.fps"];

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float k1 = fSettings["Camera.k1"];
    float k2 = fSettings["Camera.k2"];
    float p1 = fSettings["Camera.p1"];
    float p2 = fSettings["Camera.p2"];

//    double time_start;
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

    Cam camera;
    if (!util::select(cam, &dev_info)) {
        return 1;
    }
    util::print_stream_infos(cam, dev_info.index);
    std::cout << "Open device: " << dev_info.index << ", "
              << dev_info.name << std::endl << std::endl;

    OpenParams params(dev_info.index);
    params.stream_mode = StreamMode::STREAM_640x480;
    params.depth_mode = DepthMode::DEPTH_COLORFUL;
    params.color_mode = ColorMode::COLOR_RAW;
    params.ir_intensity = 0;
    params.framerate = 15;
    params.state_ae = false;

    cam.EnableImageInfo(true);
    cam.EnableStreamData(ImageType::IMAGE_LEFT_COLOR);
    cam.EnableMotionDatas();
    cam.Open(params);
    cam.SetExposureTime(18);
    cam.SetGlobalGain(1);
    cam.AutoExposureControl(false);

    Rate rate(params.framerate);
    std::cout << std::endl;
    if (!cam.IsOpened()) {
        std::cerr << "Error: Open camera failed" << std::endl;
        return 1;
    }
    std::cout << "Open device success" << std::endl << std::endl;


    Extractor extractor(argv[2]);
    Track track(mapped, &SLAM, &ar, &extractor ,&msf);
//    //开启cam,extractor,track线程
    t_cam = thread(&Cam::run, &camera);
    t_extractor = thread(&Extractor::run, &extractor);
    t_tracking = thread(&Track::run, &track);
    std::cout << "Press ESC/Q on Windows to terminate" << std::endl;

//    sched_param sch,sch1;
//    int policy;
//    pthread_getschedparam(t_extractor.native_handle(), &policy, &sch);
//    cout<<"now"<<sch.__sched_priority<<endl;
//    cout<<"max"<<sched_get_priority_max(policy)<<endl;
//    cout<<"min"<<sched_get_priority_min(policy)<<endl;
//    switch(policy)
//    {
//        case SCHED_FIFO:
//            cout<<"policy = FIFO"<<endl;
//            break;
//        case SCHED_RR:
//            cout<<"policy = RR"<<endl;
//            break;
//        case SCHED_OTHER:
//            cout<<"policy = OTHER"<<endl;
//            break;
//    }
//
//
//    //设置调度策略，观察优先级最大值最小值是否改变，从而确定调度策略的改变是否成功。
//
//    sch.sched_priority = 99;
//    sch1.sched_priority = 40;
//    policy = SCHED_RR;
//    pthread_setschedparam(t_extractor.native_handle(), policy, &sch);
//    pthread_setschedparam(t_tracking.native_handle(), policy, &sch1);
//    cout<<"now"<<sch.__sched_priority<<endl;
//    cout<<"max"<<sched_get_priority_max(policy)<<endl;
//    cout<<"min"<<sched_get_priority_min(policy)<<endl;
//    switch(policy)
//    {
//        case SCHED_FIFO:
//            cout<<"policy = FIFO"<<endl;
//            break;
//        case SCHED_RR:
//            cout<<"policy = RR"<<endl;
//            break;
//        case SCHED_OTHER:
//            cout<<"policy = OTHER"<<endl;
//            break;
//    }



    //cv::namedWindow("left");
    uint64_t timestamps = 0;

    Size imageSize(w,h);
    // Size newimageSize(w,h)
    cv::Mat distCoeffs,cameraMatrix,newcameraMatrix,map1,map2,newimage;
    distCoeffs = (cv::Mat_<double>(5,1)<<k1,k2,p1,p2,0);
    cameraMatrix = (cv::Mat_<double>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
    // newcameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, newimageSize, 0);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(),
                            imageSize, CV_16SC2, map1, map2);
    mpData mpDataStruct1;

    while (camera.getFrame().empty()) {;}
    cv::Mat frame;
    while (!SLAM.isShutdown()) {
        // cout<<"333"<<endl;
        if (camera.cam_rdy) {
            // cout<<"444"<<endl;
            //ar.flag2 = true;
            camera.cam_rdy = false;
            frame = camera.getFrame();
            timestamps = camera.getTimestamp();
            extractor.setExtractor(frame, SLAM.GetTrackingState(),timestamps);
            extractor.IsExtractor = true;
        } else {
            usleep(33000);
        }
    }

    cout << "extract size = " << extractor.framesize << endl;
    //cout << "capturesize = " << cam.counter << endl;
    gettimeofday(&time1, NULL);
    cout << "time1 = " << (time1.tv_sec * 1000000 + time1.tv_usec) << endl;
    ms1 = (time1.tv_sec * 1000000 + time1.tv_usec) - (time0.tv_sec * 1000000 + time0.tv_usec);
    cout << "time totle" << (int) ms1 << endl;
    cout << "per frame's average time = " << (float) ms1 / (float) extractor.framesize << endl;
    //SLAM.SaveTrajectory("./dataset/camera.txt");

    if (mapped) {
        tViewer.join();
        t_ar.join();
    }

    cam.Close();
    t_cam.join();

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
