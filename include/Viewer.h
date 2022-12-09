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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "AR.h"

#include <mutex>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Plane
{
public:
    Plane(const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcw);
    Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);
    Plane(){;};
    void Recompute();

    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //arbitrary orientation along normal
    float rang;
    //transformation from world to the plane
    cv::Mat Tpw;
    Eigen::Matrix3d Rpw;
    pangolin::OpenGlMatrix glTpw;
    //MapPoints that define the plane
    std::vector<MapPoint*> mvMPs;
    //camera pose when the plane was first observed (to compute normal direction)
    cv::Mat mTcw, XC;
};

class Viewer
{
public:
	bool mbReuse;
	Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer,
		Tracking *pTracking, const string &strSettingPath, bool bReuse);

    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    AR* ar;

    void DrawPlane(int ndivs, float ndivsize);
    Plane* DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, double x,double y,const int iterations=50);
    Plane* DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs,const int iterations=50);

    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
                      const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs);

    void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                      std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs);

    static void on_mouse(int event, int x, int y, int flags, void* ustc);

    cv::Mat K;
private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    std::mutex mMutexPoseImage;
    cv::Mat mTcw;
    cv::Mat mImage;
    int mStatus;
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<MapPoint*> mvMPs;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
