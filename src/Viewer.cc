///**
//* This file is part of ORB-SLAM2.
//*
//* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
//* For more information see <https://github.com/raulmur/ORB_SLAM2>
//*
//* ORB-SLAM2 is free software: you can redistribute it and/or modify
//* it under the terms of the GNU General Public License as published by
//* the Free Software Foundation, either version 3 of the License, or
//* (at your option) any later version.
//*
//* ORB-SLAM2 is distributed in the hope that it will be useful,
//* but WITHOUT ANY WARRANTY; without even the implied warranty of
//* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//* GNU General Public License for more details.
//*
//* You should have received a copy of the GNU General Public License
//* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
//*/
//
//#include "Viewer.h"
//#include <pangolin/pangolin.h>
//#include <unistd.h>
//#include <cmath>
//#include <mutex>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
//
//#define PI 3.1416
//
//namespace ORB_SLAM2
//{
//const float eps = 1e-4;
//bool flag_mouse = false;
//
//int mouse_x = -1;
//int mouse_y = -1;
//
//
//cv::Mat ExpSO3(const float &x, const float &y, const float &z)
//{
//  //cout<<"x,y,z"<<x<<","<<y<<","<<z<<endl;
//    cv::Mat I = cv::Mat::eye(3,3,CV_32FC1);
//    const float d2 = x*x+y*y+z*z; //向量长度平方
//    const float d = sqrt(d2); //向量长度
//    //cout<<"d = "<<d<<endl;
//    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
//                 z, 0, -x,
//                 -y,  x, 0);
//    cv::Mat R ;
//    if(d<eps){
//      R = (I + W + 0.5f*W*W);
//      //cout<<"R1 = "<<R<<endl;
//        return R;}
//    else{
//      R = (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
//      //cout<<"R2 = "<<R<<endl;
//        return R;}
//}
//
//cv::Mat ExpSO3(const cv::Mat &v)
//{
//    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
//}
//
//
//
//
//Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
//	Tracking *pTracking, const string &strSettingPath, bool bReuse):
//    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
//    mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
//{
//    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
//
//    float fps = fSettings["Camera.fps"];
//    if(fps<1)
//        fps=30;
//    mT = 1e3/fps;
//
//    mImageWidth = fSettings["Camera.width"];
//    mImageHeight = fSettings["Camera.height"];
//    if(mImageWidth<1 || mImageHeight<1)
//    {
//        mImageWidth = 640;
//        mImageHeight = 480;
//    }
//
//    mViewpointX = fSettings["Viewer.ViewpointX"];
//    mViewpointY = fSettings["Viewer.ViewpointY"];
//    mViewpointZ = fSettings["Viewer.ViewpointZ"];
//    mViewpointF = fSettings["Viewer.ViewpointF"];
//    float fx = fSettings["Camera.fx"];
//    float fy = fSettings["Camera.fy"];
//    float cx = fSettings["Camera.cx"];
//    float cy = fSettings["Camera.cy"];
//    mbReuse = bReuse;
//    K = (cv::Mat_<float>(3,3)<<fx,0,cx,0,fy,cy,0,0,1);
//}
//
//Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
//    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
//    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
//{
//    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
//
//    float fps = fSettings["Camera.fps"];
//    if(fps<1)
//        fps=30;
//    mT = 1e3/fps;
//
//    mImageWidth = fSettings["Camera.width"];
//    mImageHeight = fSettings["Camera.height"];
//    if(mImageWidth<1 || mImageHeight<1)
//    {
//        mImageWidth = 640;
//        mImageHeight = 480;
//    }
//
//    mViewpointX = fSettings["Viewer.ViewpointX"];
//    mViewpointY = fSettings["Viewer.ViewpointY"];
//    mViewpointZ = fSettings["Viewer.ViewpointZ"];
//    mViewpointF = fSettings["Viewer.ViewpointF"];
//}
//void Viewer::Run()
//{
//
//    static cv::Mat im,Tcw;
//    static int status;
//    static vector<cv::KeyPoint> vKeys;          // im 和 关键点在viewer中没有用处
//    static vector<MapPoint*> vMPs;              //地图点和位姿会在 DetectPlan(...)中用到
//
//    if(mbReuse)
//    {
//    while(1)
//    {
//	    GetImagePose(im,Tcw,status,vKeys,vMPs); //只有在定位模式中才会检测测平面，才会用到地图点 MapPoint 和 位姿 Tcw，所以可以只在定位模式下SetImagePose(...)
//	    if(!im.empty()) break;                  //这个循环要求在定位模式下一定要 调用 SetImagePose(...)才会继续执行后续内容（包括了窗口的显示和绘制）
//    }
//	//ar->mpRotateCallBack->ChangePos(-0.360357,0.149369,-1.02328,0.309938,0.297111,-0.105747);
//    }
//
//    mbFinished = false;
//    mbStopped = false;
//    pangolin::CreateWindowAndBind("地图显示",mImageWidth+180,mImageHeight);
//
//    // 3D Mouse handler requires depth testing to be enabled
//    glEnable(GL_DEPTH_TEST);
//
//    // Issue specific OpenGl we might need
//    glEnable (GL_BLEND);
//    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(180));
//    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
//    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
//    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
//    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
//    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",mbReuse,true);
//    pangolin::Var<bool> menuLight("menu.Light",false,true);
//    pangolin::Var<bool> menuDetectPlane("menu.Detect Plane",false,false);
//    pangolin::Var<bool> menuReset("menu.Reset map",false,false);
//    pangolin::Var<bool> menuSaveMap("menu.Save Map",false,false); //TODO
//    pangolin::Var<float> menu_size("menu. Element Size",1.0,0.0,5.0);
//    /*
//    pangolin::Var<bool> menuLeft("menu.Left",false,false);
//    pangolin::Var<bool> menuRight("menu.Right",false,false);
//    pangolin::Var<bool> menuUp("menu.Up",false,false);
//    pangolin::Var<bool> menuDown("menu.Down",false,false);
//    pangolin::Var<bool> menuFar("menu.Far",false,false);
//    pangolin::Var<bool> menuNear("menu.Near",false,false);
//    pangolin::Var<bool> menuRotateX("menu.RotateX",false,false);
//    pangolin::Var<bool> menuRotateY("menu.RotateY",false,false);
//    pangolin::Var<bool> menuRotateZ("menu.RotateZ",false,false);
//    */
//    pangolin::Var<bool> menuShutDown("menu.Shut Down",false,false);
//    // Define Camera Render Object (for view / scene browsing)
//    pangolin::OpenGlRenderState s_cam(
//                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.001,1000),
//                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
//                );
//
//    // Add named OpenGL viewport to window and provide 3D Handler
//    pangolin::View& d_cam = pangolin::CreateDisplay()
//            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -(float)mImageWidth/mImageHeight)
//            .SetHandler(new pangolin::Handler3D(s_cam));
//
//    pangolin::OpenGlMatrix Twc;
//    Twc.SetIdentity();
//
//    cv::namedWindow("Current Frame");
//
//    bool bFollow = true;
//    bool bLocalizationMode = mbReuse;
//    std::vector<Plane*> vpPlane;
//    Plane* mpPlane;
//  //  float ox = -0.360357,oy = 0.149369,oz = 1.02328,
//  //        rx = 0.309938,ry = 0.297111,rz = -0.105747;
//    float ox = 0,oy = 0,oz = 0,
//          rx = 0,ry = 0,rz = 0,size = 1;
//    Eigen::Isometry3d Tdp = Eigen::Isometry3d::Identity();
//    int imageidx=1;
//    bool detectonly = true;
//    while(1)
//    {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
////        while(1)
////        {
////            GetImagePose(im,Tcw,status,vKeys,vMPs);
////            if(!im.empty()) break;
////            usleep(10000);
////        }
//        GetImagePose(im,Tcw,status,vKeys,vMPs);
//
//        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
//        if(menuFollowCamera && bFollow)
//        {
//            s_cam.Follow(Twc);
//        }
//        else if(menuFollowCamera && !bFollow)
//        {
//            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
//            s_cam.Follow(Twc);
//            bFollow = true;
//        }
//        else if(!menuFollowCamera && bFollow)
//        {
//            bFollow = false;
//        }
//
//        if(menuLocalizationMode && !bLocalizationMode)
//        {
//            mpSystem->ActivateLocalizationMode();
//            bLocalizationMode = true;
//        }
//        else if(!menuLocalizationMode && bLocalizationMode)
//        {
//            mpSystem->DeactivateLocalizationMode();
//            bLocalizationMode = false;
//        }
//        if(menuLight){
//            ar->iflight = true;
//        }
//        else{
//            ar->iflight = false;
//        }
//        if(menuSaveMap)
//        {
//          mpSystem->SaveMapRequest();
//          menuSaveMap = false;
//        }
//
//      	if(mbReuse)
//      	{
//            /*
//          if(menuLeft)
//          {
//            ox = ox - 0.05;
//            menuLeft = false;
//          }
//          if(menuRight)
//          {
//            ox = ox + 0.05;
//            menuRight = false;
//          }
//          if(menuUp)
//          {
//            oy = oy -0.05;
//            menuUp = false;
//          }
//          if(menuDown)
//          {
//            oy = oy + 0.05;
//            menuDown = false;
//          }
//          if(menuFar)
//          {
//            oz = oz + 0.05;
//            menuFar = false;
//          }
//          if(menuNear)
//          {
//            oz = oz - 0.05;
//            menuNear = false;
//          }
//          if(menuRotateX)
//          {
//            rx = rx + 0.05;
//          //  menuRotateX = false;
//          }
//          if(menuRotateY)
//          {
//            ry = ry + 0.05;
//          //  menuRotateY = false;
//          }
//          if(menuRotateZ)
//          {
//            rz = rz + 0.05;
//          //  menuRotateZ = false;
//          }
//             */
//            if(status) imageidx++;
//            if((imageidx)%60==0 && detectonly) {
//                mpPlane = DetectPlane(Tcw,vMPs,50);
//                if(mpPlane)
//                {
//                    ox = mpPlane->o.at<float>(0,0);
//                    oy = mpPlane->o.at<float>(1,0);
//                    oz = mpPlane->o.at<float>(2,0);
//                    Eigen::Vector3d euler_angles = mpPlane->Rpw.eulerAngles(0,1,2);
//                    rx = (float)euler_angles[0];
//                    ry = (float)euler_angles[1];
//                    rz = (float)euler_angles[2];
//
//                    vector<MapPoint*> vmp = mpPlane->mvMPs;
//                    vector<float> distance(vmp.size(),0);
//                    for(int i=0;i<vmp.size();i++){
//                        cv::Mat mpw = vmp[i]->GetWorldPos();
//                        float dis = sqrt((mpw.at<float>(0)-ox)*(mpw.at<float>(0)-ox)+(mpw.at<float>(1)-oy)*(mpw.at<float>(1)-oy)+(mpw.at<float>(2)-oz)*(mpw.at<float>(2)-oz));
//                        distance[i] = dis;
//                    }
//                    sort(distance.begin(),distance.end());
//                    for(int i=0;i<vmp.size();i++){
//                        //cout<<distance[i]<<endl;
//                    }
//                    //cout<<"distan"<<distance[40];
//                    size = distance[int(0.9*vmp.size())];
//                    //cout<<"size"<<size<<endl;
////                    cout<<"mpPlane's euler"<<ox<<","<<oy<<","<<oz<<","<<rx<<","<<ry<<","<<rz<<endl;
//                    ar->mppCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,size);
//                    //ar->mpRotateCallBack->ChangePos(ox,oy,oz,rx,ry,rz,menu_size);
//                    //ar->mppCallBack->ChangePos(ox,oy,oz,rx,ry,rz,menu_size);
//                }
//            }
//      		if(PickHandle::PICK)
//      		{
//                int x = ar->pick->GetX();
//                int y = 480-ar->pick->GetY();
//                cout<<"x,y = "<<x<<"\t"<<y<<endl;
//
//            mpPlane = DetectPlane(Tcw,vMPs,x,y,50);
////                cout<<"mpPlane's euler "<<ox<<","<<oy<<","<<oz<<",   "<<rx<<","<<ry<<","<<rz<<endl;
////
////                cout<<"mpPlane.vector = "<<mpPlane->n.at<float>(0,0)<<","<<mpPlane->n.at<float>(1,0)<<","<<mpPlane->n.at<float>(2,0)<<endl;
//            /*
//            Plane* mPlane = DetectPlane(Tcw,vMPs,200);
//
//            if(mPlane)
//            {
//              vpPlane.push_back(mPlane);
//            }
//           if(!vpPlane.empty())
//           {
//              cout<<"New plane is created!"<<endl;
//              cout<<"nums of plane = "<<vpPlane.size()<<endl;
//              float max_CosAngle = 0;
//              cv::Mat z_world = (cv::Mat_<float>(3,1)<<0.0, 1.0 ,0.0);
//              cout<<"z_world creat"<<endl;
//              for(size_t i=0; i<vpPlane.size(); i++)
//              {
//                float CosAngle = abs((float)(vpPlane[i]->n).dot(z_world));
//                cout<<"CosAngle = "<<CosAngle<<endl;
//                if(CosAngle > max_CosAngle)
//                {
//                  max_CosAngle = CosAngle;
//                  mpPlane = vpPlane[i];
//                }
//              }*/
//
//              if(mpPlane)
//              {
//              ox = mpPlane->o.at<float>(0,0);
//              oy = mpPlane->o.at<float>(1,0);
//              oz = mpPlane->o.at<float>(2,0);
//              Eigen::Vector3d euler_angles = mpPlane->Rpw.eulerAngles(0,1,2);
//              rx = (float)euler_angles[0];
//              ry = (float)euler_angles[1];
//              rz = (float)euler_angles[2];
//
//                  vector<MapPoint*> vmp = mpPlane->mvMPs;
//                  vector<float> distance(vmp.size(),0);
//                  for(int i=0;i<vmp.size();i++){
//                      cv::Mat mpw = vmp[i]->GetWorldPos();
//                      float dis = sqrt((mpw.at<float>(0)-ox)*(mpw.at<float>(0)-ox)+(mpw.at<float>(1)-oy)*(mpw.at<float>(1)-oy)+(mpw.at<float>(2)-oz)*(mpw.at<float>(2)-oz));
//                      distance[i] = dis;
//                  }
//                  sort(distance.begin(),distance.end());
//                  for(int i=0;i<vmp.size();i++){
////                      cout<<distance[i]<<endl;
//                  }
//                  //cout<<"distan"<<distance[40];
//                  size = distance[int(0.9*vmp.size())];
//                  //cout<<"size"<<size<<endl;
//              //cout<<"mpPlane's euler"<<ox<<","<<oy<<","<<oz<<","<<rx<<","<<ry<<","<<rz<<endl;
//              ar->mpRotateCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,size);
//              ar->mppCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,size);
//                  //ar->mpRotateCallBack->ChangePos(ox,oy,oz,rx,ry,rz,menu_size);
//                  //ar->mppCallBack->ChangePos(ox,oy,oz,rx,ry,rz,menu_size);
//                detectonly = false;
//
//              }
//              else
//              {
//                   cout<<"No plane detected!"<<endl;
//              }
//            //if(flag_mouse)
//            //{
//
//              //flag_mouse = false;
//          	//}
//          		 PickHandle::PICK = false;
//             }
//            if(!detectonly) ar->mpRotateCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,size);
//            else ar->mpRotateCallBack->ChangePos(0,0,0,0,0,0,menu_size);
//             ar->mppCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,size);
//
//            //ar->mpRotateCallBack->ChangePos(ox,oy,oz,rx,ry,rz,menu_size);
//            //ar->mppCallBack->ChangePos(ox,oy,oz,rx,ry,rz,menu_size);
//            /*
//            //平面做相对位置用的代码
//            Eigen::Matrix3d Rdw;
//            Rdw = Eigen::AngleAxisd((double)rx*PI,Eigen::Vector3d::UnitZ())
//                  *Eigen::AngleAxisd((double)ry*PI,Eigen::Vector3d::UnitY())
//                  *Eigen::AngleAxisd((double)rz*PI,Eigen::Vector3d::UnitX());
//            //Eigen::AngleAxisd Vdw ((rz,Eigen::Vector3d (0,0,1))*(ry,Eigen::Vector3d (0,1,0))*(rx,Eigen::Vector3d (1,0,0)));
//            Eigen::AngleAxisd Vdw(Rdw);
//            Eigen::Isometry3d Tdw = Eigen::Isometry3d::Identity();
//            Tdw.rotate(Vdw); //
//            Tdw.pretranslate(Eigen::Vector3d((double)ox,(double)oy,(double)oz));
//            //Tdw << Rdw<float>(0,0),Rdw.at<float>(0,1),Rdw.at<float>(0,2),ox,
//            //       Rdw.at<float>(1,0),Rdw.at<float>(1,1),Rdw.at<float>(1,2),oy,
//            //       Rdw.at<float>(2,0),Rdw.at<float>(2,1),Rdw.at<float>(2)(2),oz,
//            //       0.0 , 0.0 , 0.0 , 1.0;
//            Eigen::Isometry3d Tpw = Eigen::Isometry3d::Identity();
//            Eigen::AngleAxisd Vpw(mpPlane->Rpw);
//            Tpw.rotate(Vpw);
//            Tpw.pretranslate(Eigen::Vector3d(mpPlane->Tpw.at<double>(0,3),mpPlane->Tpw.at<double>(1,3),mpPlane->Tpw.at<double>(2,3)));
//            //Tpw<<(mpPlane->Tpw).at<float>(0,0),(mpPlane->Tpw).at<float>(0,1),(mpPlane->Tpw).at<float>(0,2),(mpPlane->Tpw).at<float>(0,3),
//            //     (mpPlane->Tpw).at<float>(1,0),(mpPlane->Tpw).at<float>(1,1),(mpPlane->Tpw).at<float>(1,2),(mpPlane->Tpw).at<float>(1,3),
//            //     (mpPlane->Tpw).at<float>(2,0),(mpPlane->Tpw).at<float>(2,1),(mpPlane->Tpw).at<float>(2,2),(mpPlane->Tpw).at<float>(2,3),
//            //     (mpPlane->Tpw).at<float>(3,0),(mpPlane->Tpw).at<float>(3,1),(mpPlane->Tpw).at<float>(3,2),(mpPlane->Tpw).at<float>(3,3);
//            Tdp = Tpw*(Tdw.inverse());
//            */
//            /*
//          if(mpPlane)
//          {
//            // Recompute plane if there has been a loop closure or global BA
//            // In localization mode, map is not updated so we do not need to recompute
//            //glPushMatrix();
//          //  mpPlane->glTpw.Multiply();
//          ///  DrawPlane(menu_ngrid,menu_sizegrid);
//          //  glPopMatrix();
//            bool bRecompute = false;
//            if(!bLocalizationMode)
//            {
//                if(mpSystem->MapChanged())
//                {
//                    cout << "Map changed. All virtual elements are recomputed!" << endl;
//                    bRecompute = true;
//                }
//
//                if(mpPlane)
//                {
//                    if(bRecompute)
//                    {
//                        mpPlane->Recompute();
//                        cout<<"plane is recomputed!"<<endl;*/
//                        /*
//                        //计算平面改变后的3D物体坐标
//                        Eigen::Isometry3d Tdw = Eigen::Isometry3d::Identity();
//                        Eigen::Isometry3d Tpw = Eigen::Isometry3d::Identity();
//                        Eigen::AngleAxisd Vpw(mpPlane->Rpw);
//                        Tpw.rotate(Vpw);
//                        Tpw.pretranslate(Eigen::Vector3d(mpPlane->Tpw.at<double>(0),mpPlane->Tpw.at<double>(1),mpPlane->Tpw.at<double>(2)));
//                        Tdw = Tpw * (Tdp.inverse());
//                        Eigen::Matrix3d Rdw = Eigen::Matrix3d::Identity(3,3);
//                        Rdw << (double)Tdw.matrix()(0,0),(double)Tdw.matrix()(0,1),(double)Tdw.matrix()(0,2),
//                               (double)Tdw.matrix()(1,0),(double)Tdw.matrix()(1,1),(double)Tdw.matrix()(1,2),
//                               (double)Tdw.matrix()(2,0),(double)Tdw.matrix()(2,1),(double)Tdw.matrix()(2,2);
//                        Eigen::Vector3d tdw;
//                        tdw << (double)Tdw.matrix()(3,0),(double)Tdw.matrix()(3,1),(double)Tdw.matrix()(3,2);
//
//                  			ox = (float)tdw(0,0);
//                  			oy = (float)tdw(1,0);
//                  			oz = (float)tdw(2,0);
//                  			Eigen::Vector3d euler_angles = Rdw.eulerAngles(0,1,2);
//                  			rx = (float)euler_angles[0];
//                  			ry = (float)euler_angles[1];
//                  			rz = (float)euler_angles[2];
//                        */
//                //    }
//            //    }
//        //    }
////}
//
//          //cout<<"ox = "<<ox<<"\t"<<"oy = "<<oy<<"\t"<<"oz = "<<oz<<"\t"<<"rx = "<<rx<<"\t"<<"ry = "<<ry<<"\t"<<"rz = "<<rz<<endl;
//
//        }
//        d_cam.Activate(s_cam);
//        glClearColor(1.0f,1.0f,1.0f,1.0f);
//        mpMapDrawer->DrawCurrentCamera(Twc);
//        if(menuShowKeyFrames || menuShowGraph)
//            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
//        if(menuShowPoints)
//            mpMapDrawer->DrawMapPoints();
//        pangolin::glDrawAxis(0.5f);
//        pangolin::FinishFrame();
//
//        cv::Mat im = mpFrameDrawer->DrawFrame();
//        float x=Twc.m[12],y=Twc.m[13],z=Twc.m[14];
//        cv::putText(im,"x "+std::to_string(x),cv::Point(20,40),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,0,255),2,8);
//        cv::putText(im,"y "+std::to_string(y),cv::Point(20,80),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,255,0),2,8);
//        cv::putText(im,"z "+std::to_string(z),cv::Point(20,120),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(255,0,0),2,8);
//        cv::imshow("Current Frame",im);
//        cv::waitKey(mT);
//
//        if(menuReset)
//        {
//            menuShowGraph = true;
//            menuShowKeyFrames = true;
//            menuShowPoints = true;
//            menuLocalizationMode = false;
//            if(bLocalizationMode)
//                mpSystem->DeactivateLocalizationMode();
//            bLocalizationMode = false;
//            bFollow = true;
//            menuFollowCamera = true;
//            mpSystem->Reset();
//            menuReset = false;
//        }
//        if(menuShutDown)
//        {
//	         mpSystem->ShutdownRequest();
//	         menuShutDown = false;
//        }
//
//        if(Stop())
//        {
//          while(isStopped())
//          {
//            usleep(3000);
//          }
//        }
//
//        if(CheckFinish())  break;
//    }
//
//    SetFinish();
//}
//void Viewer::DrawPlane(int ndivs, float ndivsize)
//{
//    // Plane parallel to x-z at origin with normal -y
//    const float minx = -ndivs*ndivsize;
//    const float minz = -ndivs*ndivsize;
//    const float maxx = ndivs*ndivsize;
//    const float maxz = ndivs*ndivsize;
//
//
//    glLineWidth(2);
//    glColor3f(0.7f,0.7f,1.0f);
//    glBegin(GL_LINES);
//
//    for(int n = 0; n<=2*ndivs; n++)
//    {
//        glVertex3f(minx+ndivsize*n,0,minz);
//        glVertex3f(minx+ndivsize*n,0,maxz);
//        glVertex3f(minx,0,minz+ndivsize*n);
//        glVertex3f(maxx,0,minz+ndivsize*n);
//    }
//
//    glEnd();
//
//}
//
//void Viewer::RequestFinish()
//{
//    unique_lock<mutex> lock(mMutexFinish);
//    mbFinishRequested = true;
//}
//
//bool Viewer::CheckFinish()
//{
//    unique_lock<mutex> lock(mMutexFinish);
//    return mbFinishRequested;
//}
//
//void Viewer::SetFinish()
//{
//    unique_lock<mutex> lock(mMutexFinish);
//    mbFinished = true;
//}
//
//bool Viewer::isFinished()
//{
//    unique_lock<mutex> lock(mMutexFinish);
//    return mbFinished;
//}
//
//void Viewer::RequestStop()
//{
//    unique_lock<mutex> lock(mMutexStop);
//    if(!mbStopped)
//        mbStopRequested = true;
//}
//
//bool Viewer::isStopped()
//{
//    unique_lock<mutex> lock(mMutexStop);
//    return mbStopped;
//}
//
//bool Viewer::Stop()
//{
//    unique_lock<mutex> lock(mMutexStop);
//    unique_lock<mutex> lock2(mMutexFinish);
//
//    if(mbFinishRequested)
//        return false;
//    else if(mbStopRequested)
//    {
//        mbStopped = true;
//        mbStopRequested = false;
//        return true;
//    }
//
//    return false;
//
//}
//
//void Viewer::Release()
//{
//    unique_lock<mutex> lock(mMutexStop);
//    mbStopped = false;
//}
//
//void Viewer::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM2::MapPoint*> &vMPs)
//{
//
//    unique_lock<mutex> lock(mMutexPoseImage);
//    mImage = im.clone();
////    if(mImage.empty()) cout<<"mImage is empty"<<endl;
////    else cout<<"mImage is not empty"<<endl;
//    mTcw = Tcw.clone();
//    mStatus = status;
//    mvKeys = vKeys;
//    mvMPs = vMPs;
//}
//
//void Viewer::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs)
//{
//    unique_lock<mutex> lock(mMutexPoseImage);
//	im = mImage.clone();
//    Tcw = mTcw.clone();
//    status = mStatus;
//    vKeys = mvKeys;
//    vMPs = mvMPs;
//}
//
//
//Plane* Viewer::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs,double x,double y, const int iterations)
//{
//    // Retrieve 3D points
//    vector<cv::Mat> vPoints;
//    vPoints.reserve(vMPs.size());
//    std::vector<MapPoint*> vPointMP;
//    vPointMP.reserve(vMPs.size());
//
//    for(size_t i=0; i<vMPs.size(); i++)
//    {
//        MapPoint* pMP=vMPs[i];
//        if(pMP)
//        {
//            if(pMP->Observations()>5)	//地图点存在且观测数大于5
//            {
//                cv::Mat puv = cv::Mat::zeros(3,1,CV_32F);
//                /*
//                cout<<"K"<<K.cols<<"\t"<<K.rows<<endl;
//                cout<<"R"<<(Tcw.colRange(0,3).rowRange(0,3)).cols<<"\t"<<(Tcw.colRange(0,3).rowRange(0,3)).rows<<endl;
//                cout<<"T"<<(Tcw.rowRange(0,3).col(3)).cols<<"\t"<<(Tcw.rowRange(0,3).col(3)).rows<<endl;
//                cout<<"mMp"<<pMP->GetWorldPos().cols<<"\t"<<pMP->GetWorldPos().rows<<endl;
//                 */
//                puv = K*(Tcw.colRange(0,3).rowRange(0,3)*pMP->GetWorldPos()+Tcw.rowRange(0,3).col(3));
//                double px = puv.at<float>(0)/puv.at<float>(2);
//                double py = puv.at<float>(1)/puv.at<float>(2);
//                //cout<<"x,y"<<x<<"\t"<<y<<endl;
//                if(px<x+mImageWidth*0.3 && px>x-mImageWidth*0.3 && py<y+mImageHeight*0.3 && py>y-mImageHeight*0.3) {
//                    //cout<<"puv"<<puv.at<float>(0)<<"\t"<<puv.at<float>(1)<<"\t"<<puv.at<float>(2)<<endl;
//                    //cout<<"puv"<<puv.at<float>(0)/puv.at<float>(2)<<"\t"<<puv.at<float>(1)/puv.at<float>(2)<<endl;
//                    vPoints.push_back(pMP->GetWorldPos());    //地图点坐标
//                    //cout<<pMP->GetWorldPos().at<float>(0,0)<<","<<pMP->GetWorldPos().at<float>(1,0)<<","<<pMP->GetWorldPos().at<float>(2,0)<<";"<<endl;
//                    vPointMP.push_back(pMP);    //地图点
//                }
//            }
//        }
//    }
//
//    const int N = vPoints.size();
//
//    if(N<50)
//        return NULL;
//
//
//    // Indices for minimum set selection
//    vector<size_t> vAllIndices;
//    vAllIndices.reserve(N);
//    vector<size_t> vAvailableIndices;
//
//    for(int i=0; i<N; i++)
//    {
//        vAllIndices.push_back(i);
//    }
//
//    float bestDist = 1e5;
//    vector<float> bestvDist;
//
//    //RANSAC
//    for(int n=0; n<iterations; n++)	//迭代求存在且观测数大于5的地图点的最佳法线
//    {
//        vAvailableIndices = vAllIndices;
//
//        cv::Mat A(3,4,CV_32F);
//        A.col(3) = cv::Mat::ones(3,1,CV_32F);
//
//        // Get min set of points
//        for(short i = 0; i < 3; ++i)	//随机选3个点做最小集？
//        {
//            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);
//
//            int idx = vAvailableIndices[randi];
//
//            A.row(i).colRange(0,3) = vPoints[idx].t();
//
//            vAvailableIndices[randi] = vAvailableIndices.back();
//            vAvailableIndices.pop_back();
//        }
//
//        cv::Mat u,w,vt;
//        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);	//	svd分解
//
//        const float a = vt.at<float>(3,0);	//最小集3点的平面法线
//        const float b = vt.at<float>(3,1);
//        const float c = vt.at<float>(3,2);
//        const float d = vt.at<float>(3,3);
//        const float vf = 1.0f/sqrt(a*a+b*b+c*c);
//        const float nx = a*vf;	//单位法线
//        const float ny = b*vf;
//        const float nz = c*vf;
//        //cout<<"nx,ny,nz = "<<nx<<","<<ny<<","<<nz<<endl;
//        vector<float> vDistances(N,0);
//
//        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);
//
//        for(int i=0; i<N; i++)
//        {
//            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;	//所有点在法线投影的距离
//        }
//
//        vector<float> vSorted = vDistances;
//        sort(vSorted.begin(),vSorted.end());
//
//        int nth = max((int)(0.2*N),20);
//        const float medianDist = vSorted[nth];	//中等距离
//
//        if(medianDist<bestDist)
//        {
//            bestDist = medianDist;
//            bestvDist = vDistances;
//        }
//    }
//
//    // Compute threshold inlier/outlier
//    const float th = 1.4*bestDist;
//    vector<bool> vbInliers(N,false);
//    int nInliers = 0;
//    for(int i=0; i<N; i++)
//    {
//        if(bestvDist[i]<th)
//        {
//            nInliers++;
//            vbInliers[i]=true;	//小于门限设为内点
//        }
//    }
//
//    vector<MapPoint*> vInlierMPs(nInliers,NULL);
//    int nin = 0;
//    //cout<<"MapPoint's world"<<endl;
//    for(int i=0; i<N; i++)
//    {
//        if(vbInliers[i])
//        {
//            vInlierMPs[nin] = vPointMP[i];
//            //cout<<"MapPoint"<<nin<<"("<<vInlierMPs[nin]->GetWorldPos().at<float>(0,0)<<","<<vInlierMPs[nin]->GetWorldPos().at<float>(1,0)<<","<<vInlierMPs[nin]->GetWorldPos().at<float>(2,0)<<")"<<endl;
//            nin++;
//        }
//    }
//
//    return new Plane(vInlierMPs,Tcw);
//}
//Plane* Viewer::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs,const int iterations)
//{
//    // Retrieve 3D points
//    vector<cv::Mat> vPoints;
//    vPoints.reserve(vMPs.size());
//    std::vector<MapPoint*> vPointMP;
//    vPointMP.reserve(vMPs.size());
//
//    for(size_t i=0; i<vMPs.size(); i++)
//    {
//        MapPoint* pMP=vMPs[i];
//        if(pMP)
//        {
//            if(pMP->Observations()>5)	//地图点存在且观测数大于5
//            {
//                    vPoints.push_back(pMP->GetWorldPos());    //地图点坐标
////                        cout<< " 地图点的坐标是 "<<pMP->GetWorldPos()<<endl;
//                    vPointMP.push_back(pMP);    //地图点
//
//            }
//        }
//    }
//
//    const int N = vPoints.size();
//
//    if(N<50)
//        return NULL;
//
//
//    // Indices for minimum set selection
//    vector<size_t> vAllIndices;
//    vAllIndices.reserve(N);
//    vector<size_t> vAvailableIndices;
//
//    for(int i=0; i<N; i++)
//    {
//        vAllIndices.push_back(i);
//    }
//
//    float bestDist = 1e5;
//    vector<float> bestvDist;
//
//    //RANSAC
//    for(int n=0; n<iterations; n++)	//迭代求存在且观测数大于5的地图点的最佳法线
//    {
//        vAvailableIndices = vAllIndices;
//
//        cv::Mat A(3,4,CV_32F);
//        A.col(3) = cv::Mat::ones(3,1,CV_32F);
//
//        // Get min set of points
//        for(short i = 0; i < 3; ++i)	//随机选3个点做最小集？
//        {
//            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);
//
//            int idx = vAvailableIndices[randi];
//
//            A.row(i).colRange(0,3) = vPoints[idx].t();
//
//            vAvailableIndices[randi] = vAvailableIndices.back();
//            vAvailableIndices.pop_back();
//        }
//
//        cv::Mat u,w,vt;
//        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);	//	svd分解
//
//        const float a = vt.at<float>(3,0);	//最小集3点的平面法线
//        const float b = vt.at<float>(3,1);
//        const float c = vt.at<float>(3,2);
//        const float d = vt.at<float>(3,3);
//        const float vf = 1.0f/sqrt(a*a+b*b+c*c);
//        const float nx = a*vf;	//单位法线
//        const float ny = b*vf;
//        const float nz = c*vf;
//        //cout<<"nx,ny,nz = "<<nx<<","<<ny<<","<<nz<<endl;
//        vector<float> vDistances(N,0);
//
//        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);
//
//        for(int i=0; i<N; i++)
//        {
//            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;	//所有点在法线投影的距离
//        }
//
//        vector<float> vSorted = vDistances;
//        sort(vSorted.begin(),vSorted.end());
//
//        int nth = max((int)(0.2*N),20);
//        const float medianDist = vSorted[nth];	//中等距离
//
//        if(medianDist<bestDist)
//        {
//            bestDist = medianDist;
//            bestvDist = vDistances;
//        }
//    }
//
//    // Compute threshold inlier/outlier
//    const float th = 1.4*bestDist;
//    vector<bool> vbInliers(N,false);
//    int nInliers = 0;
//    for(int i=0; i<N; i++)
//    {
//        if(bestvDist[i]<th)
//        {
//            nInliers++;
//            vbInliers[i]=true;	//小于门限设为内点
//        }
//    }
//
//    vector<MapPoint*> vInlierMPs(nInliers,NULL);
//    int nin = 0;
//    //cout<<"MapPoint's world"<<endl;
//    for(int i=0; i<N; i++)
//    {
//        if(vbInliers[i])
//        {
//            vInlierMPs[nin] = vPointMP[i];
//            //cout<<"MapPoint"<<nin<<"("<<vInlierMPs[nin]->GetWorldPos().at<float>(0,0)<<","<<vInlierMPs[nin]->GetWorldPos().at<float>(1,0)<<","<<vInlierMPs[nin]->GetWorldPos().at<float>(2,0)<<")"<<endl;
//            nin++;
//        }
//    }
//
//    return new Plane(vInlierMPs,Tcw);
//}
//
//Plane::Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
//{
//    rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
//    Recompute();
//}
//
//void Plane::Recompute()
//{
//    const int N = mvMPs.size();
//
//    // Recompute plane with all points
//    cv::Mat A = cv::Mat(N,4,CV_32F);
//    A.col(3) = cv::Mat::ones(N,1,CV_32F);
//
//    o = cv::Mat::zeros(3,1,CV_32F);
//
//    int nPoints = 0;
//    for(int i=0; i<N; i++)
//    {
//        MapPoint* pMP = mvMPs[i];
//        if(!pMP->isBad())		//好地图点存入A矩阵
//        {
//            cv::Mat Xw = pMP->GetWorldPos();
//            o+=Xw;
//            A.row(nPoints).colRange(0,3) = Xw.t();
//            cout << " Xw.t() is  " <<  Xw.t() << endl;
//            nPoints++;
//        }
//    }
//    A.resize(nPoints);
//
//    cv::Mat u,w,vt;
//    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);	//对A进行svd分解
//
//    float a = vt.at<float>(3,0);	//法线？
//    float b = vt.at<float>(3,1);
//    float c = vt.at<float>(3,2);
//
//    o = o*(1.0f/nPoints);	//好点中心
//    const float f = 1.0f/sqrt(a*a+b*b+c*c);
//
//    // Compute XC just the first time
//    if(XC.empty())
//    {
//        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);	//twc = -Rwc*tcw
//        XC = Oc-o;	//从好点中心o到相机光心Oc的向量
//    }
//
//    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)	//法线方向
//    {
//        a=-a;
//        b=-b;
//        c=-c;
//    }
//
//    const float nx = a*f;	//单位法线
//    const float ny = b*f;
//    const float nz = c*f;
//    //cout<<"recompute n = "<<nx<<","<<ny<<","<<nz<<endl;
//    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);
//
//    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);
//
//    cv::Mat v = up.cross(n); //up和n叉乘，是y轴和法向量都垂直的向量
//    const float sa = cv::norm(v); //v的范数 sin(e)
//    //cout<<"sa = "<<sa<<endl;
//    const float ca = up.dot(n); //up点乘n，是平面法向量在y轴上的投影 cos(e)
//    float aca = acos(ca);
//    //cout<<"ca = "<<ca<<endl;
//    const float ang = atan2(sa,ca); //求反正切 arctan(sa/ca)=e
//    //cout<<"ang = "<<ang<<endl;
//    /*
//    Eigen::AngleAxisd Vpw ( aca, Eigen::Vector3d ( v.at<float>(0,0),v.at<float>(1,0),v.at<float>(2,0) ) );
//    Eigen::Vector3d eulerAngle_pw = Vpw.matrix().eulerAngles(0,1,2);
//    rx = (float)eulerAngle_pw[0];
//    ry = (float)eulerAngle_pw[1];
//    rz = (float)eulerAngle_pw[2];
//    cout<<"mpPlane's euler2"<<rx<<","<<ry<<","<<rz<<endl;
//*/
//    //Tpw = cv::Mat::eye(4,4,CV_32FC1);
//
//    cv::Mat R = ExpSO3(v*ang/sa);
//    cout << "ang is " << ang*180/3.14159267  << endl << " v is " << v << endl;
//    cout << " v*ang/sa is "  << v*ang/sa << endl;
//    Tpw = (cv::Mat_<float>(4,4)<<R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),o.at<float>(0,0),
//                           R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),o.at<float>(1,0),
//                           R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),o.at<float>(2,0),
//                           0,0,0,1);
//    //Tpw.rowRange(0,3).colRange(0,3) = R;
//    //Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
//    //o.copyTo(Tpw.col(3).rowRange(0,3));
//    //cout<<"Tpw"<<Tpw;
//    glTpw.m[0] = Tpw.at<float>(0,0);
//    glTpw.m[1] = Tpw.at<float>(1,0);
//    glTpw.m[2] = Tpw.at<float>(2,0);
//    glTpw.m[3]  = 0.0;
//
//    glTpw.m[4] = Tpw.at<float>(0,1);
//    glTpw.m[5] = Tpw.at<float>(1,1);
//    glTpw.m[6] = Tpw.at<float>(2,1);
//    glTpw.m[7]  = 0.0;
//
//    glTpw.m[8] = Tpw.at<float>(0,2);
//    glTpw.m[9] = Tpw.at<float>(1,2);
//    glTpw.m[10] = Tpw.at<float>(2,2);
//    glTpw.m[11]  = 0.0;
//
//    glTpw.m[12] = Tpw.at<float>(0,3);
//    glTpw.m[13] = Tpw.at<float>(1,3);
//    glTpw.m[14] = Tpw.at<float>(2,3);
//    glTpw.m[15]  = 1.0;
//
//    Rpw = Eigen::Matrix3d::Identity(3,3);
//    Rpw <<   Tpw.at<float>(0,0),Tpw.at<float>(0,1),Tpw.at<float>(0,2),
//	     Tpw.at<float>(1,0),Tpw.at<float>(1,1),Tpw.at<float>(1,2),
//	     Tpw.at<float>(2,0),Tpw.at<float>(2,1),Tpw.at<float>(2,2);
////    cout<<"    Rpw  ="<<   Tpw.at<float>(0,0)<<","<<Tpw.at<float>(0,1)<<","<<Tpw.at<float>(0,2)<<","<<
////    	     Tpw.at<float>(1,0)<<","<<Tpw.at<float>(1,1)<<","<<Tpw.at<float>(1,2)<<","<<
////    	     Tpw.at<float>(2,0)<<","<<Tpw.at<float>(2,1)<<","<<Tpw.at<float>(2,2);
//}
//
//Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
//{
//    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);	//normal	法线
//    o = (cv::Mat_<float>(3,1)<<ox,oy,oz);	//origin	原点
//    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);
//
//    cv::Mat v = up.cross(n);	//v = up X n
//    const float s = cv::norm(v);	//s = |v| = |n| sin<up,n>
//    const float c = up.dot(n);		//c = up . n = |n| cos<up,n>
//    const float a = atan2(s,c);		//a = tan<up,n>
//    Tpw = cv::Mat::eye(4,4,CV_32F);
//    const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
//    //cout << rang;
//    //Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);	//旋转向量tan<up,n>*v/|v|和rang*up
//    //o.copyTo(Tpw.col(3).rowRange(0,3));	//平移向量tpw = o
//    cv::Mat R = ExpSO3(v*a/s);
//    Tpw = (cv::Mat_<float>(4,4)<<R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),o.at<float>(0,0),
//                           R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),o.at<float>(1,0),
//                           R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),o.at<float>(2,0),
//                           0,0,0,1);
//    glTpw.m[0] = Tpw.at<float>(0,0);
//    glTpw.m[1] = Tpw.at<float>(1,0);
//    glTpw.m[2] = Tpw.at<float>(2,0);
//    glTpw.m[3] = 0.0;
//
//    glTpw.m[4] = Tpw.at<float>(0,1);
//    glTpw.m[5] = Tpw.at<float>(1,1);
//    glTpw.m[6] = Tpw.at<float>(2,1);
//    glTpw.m[7] = 0.0;
//
//    glTpw.m[8] = Tpw.at<float>(0,2);
//    glTpw.m[9] = Tpw.at<float>(1,2);
//    glTpw.m[10] = Tpw.at<float>(2,2);
//    glTpw.m[11] = 0.0;
//
//    glTpw.m[12] = Tpw.at<float>(0,3);
//    glTpw.m[13] = Tpw.at<float>(1,3);
//    glTpw.m[14] = Tpw.at<float>(2,3);
//    glTpw.m[15] = 1.0;
//}
//}
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

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <cmath>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI 3.1416

namespace ORB_SLAM2
{
    const float eps = 1e-4;
    bool flag_mouse = false;

    int mouse_x = -1;
    int mouse_y = -1;


    cv::Mat ExpSO3(const float &x, const float &y, const float &z)
    {
        //cout<<"x,y,z"<<x<<","<<y<<","<<z<<endl;
        cv::Mat I = cv::Mat::eye(3,3,CV_32FC1);
        const float d2 = x*x+y*y+z*z; //向量长度平方
        const float d = sqrt(d2); //向量长度
        //cout<<"d = "<<d<<endl;
        cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                z, 0, -x,
                -y,  x, 0);
        cv::Mat R ;
        if(d<eps){
            R = (I + W + 0.5f*W*W);
            //cout<<"R1 = "<<R<<endl;
            return R;}
        else{
            R = (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
            //cout<<"R2 = "<<R<<endl;
            return R;}
    }

    cv::Mat ExpSO3(const cv::Mat &v)
    {
        return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
    }




    Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                   Tracking *pTracking, const string &strSettingPath, bool bReuse):
            mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
            mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if(fps<1)
            fps=30;
        mT = 1e3/fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if(mImageWidth<1 || mImageHeight<1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];
        mbReuse = bReuse;
        //K = (cv::Mat_<float>(3,3)<<nx,ny,nz);
    }

    Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
            mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
            mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if(fps<1)
            fps=30;
        mT = 1e3/fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if(mImageWidth<1 || mImageHeight<1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }

    void Viewer::Run()
    {
        static cv::Mat im,Tcw;
        static int status;
        static vector<cv::KeyPoint> vKeys;
        static vector<MapPoint*> vMPs;
        if(mbReuse)
        {
            while(1)
            {
                GetImagePose(im,Tcw,status,vKeys,vMPs);
                if(!im.empty()) break;
            }
            //ar->mpRotateCallBack->ChangePos(-0.360357,0.149369,-1.02328,0.309938,0.297111,-0.105747);
        }
        mbFinished = false;
        mbStopped = false;
        pangolin::CreateWindowAndBind("地图显示",mImageWidth+180,mImageHeight);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(180));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",mbReuse,true);
        pangolin::Var<bool> menuLight("menu.Light",false,true);
        pangolin::Var<bool> menuDetectPlane("menu.Detect Plane",false,false);
        pangolin::Var<bool> menuReset("menu.Reset map",false,false);
        pangolin::Var<bool> menuSaveMap("menu.Save Map",false,false); //TODO
        pangolin::Var<float> menu_size("menu. Element Size",1.0,0.0,5.0);
        /*
        pangolin::Var<bool> menuLeft("menu.Left",false,false);
        pangolin::Var<bool> menuRight("menu.Right",false,false);
        pangolin::Var<bool> menuUp("menu.Up",false,false);
        pangolin::Var<bool> menuDown("menu.Down",false,false);
        pangolin::Var<bool> menuFar("menu.Far",false,false);
        pangolin::Var<bool> menuNear("menu.Near",false,false);
        pangolin::Var<bool> menuRotateX("menu.RotateX",false,false);
        pangolin::Var<bool> menuRotateY("menu.RotateY",false,false);
        pangolin::Var<bool> menuRotateZ("menu.RotateZ",false,false);
        */
        pangolin::Var<bool> menuShutDown("menu.Shut Down",false,false);
        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.001,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -(float)mImageWidth/mImageHeight)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        cv::namedWindow("Current Frame");

        bool bFollow = true;
        bool bLocalizationMode = mbReuse;
        vector<Plane*> vpPlane;
        Plane* mpPlane;
        //  float ox = -0.360357,oy = 0.149369,oz = 1.02328,
        //        rx = 0.309938,ry = 0.297111,rz = -0.105747;
        float ox = 0,oy = 0,oz = 0,
                rx = 0,ry = 0,rz = 0;
        Eigen::Isometry3d Tdp = Eigen::Isometry3d::Identity();
        while(1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            GetImagePose(im,Tcw,status,vKeys,vMPs);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if(menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }
            if(menuLight){
                ar->iflight = true;
            }
            else{
                ar->iflight = false;
            }
            if(menuSaveMap)
            {
                mpSystem->SaveMapRequest();
                menuSaveMap = false;
            }

            if(mbReuse)
            {
                /*
              if(menuLeft)
              {
                ox = ox - 0.05;
                menuLeft = false;
              }
              if(menuRight)
              {
                ox = ox + 0.05;
                menuRight = false;
              }
              if(menuUp)
              {
                oy = oy -0.05;
                menuUp = false;
              }
              if(menuDown)
              {
                oy = oy + 0.05;
                menuDown = false;
              }
              if(menuFar)
              {
                oz = oz + 0.05;
                menuFar = false;
              }
              if(menuNear)
              {
                oz = oz - 0.05;
                menuNear = false;
              }
              if(menuRotateX)
              {
                rx = rx + 0.05;
              //  menuRotateX = false;
              }
              if(menuRotateY)
              {
                ry = ry + 0.05;
              //  menuRotateY = false;
              }
              if(menuRotateZ)
              {
                rz = rz + 0.05;
              //  menuRotateZ = false;
              }
                 */
                if(menuDetectPlane)
                {
                    /*
                    if(!vpPlane.empty())
                    {
                      for(size_t i=0; i<vpPlane.size(); i++)
                      {
                        delete vpPlane[i];
                      }
                      vpPlane.clear();
                    }
                    for(int nums=0; nums<10; nums++)
                    {
                      Plane* mPlane = DetectPlane(Tcw,vMPs,200);
                      if(mPlane)
                      {
                        vpPlane.push_back(mPlane);
                      }
                    }
                    */
                    //setMouseCallback("Current Frame",on_mouse,0);//调用回调函数
                    //cout<<"鼠标点击坐标：("<<mouse_x<<","<<mouse_y<<")"<<endl;
                    mpPlane = DetectPlane(Tcw,vMPs,50);
                    //cout<<"mpPlane.vector = "<<mpPlane->n.at<float>(0,0)<<","<<mpPlane->n.at<float>(1,0)<<","<<mpPlane->n.at<float>(2,0)<<endl;
                    /*
                    Plane* mPlane = DetectPlane(Tcw,vMPs,200);

                    if(mPlane)
                    {
                      vpPlane.push_back(mPlane);
                    }
                   if(!vpPlane.empty())
                   {
                      cout<<"New plane is created!"<<endl;
                      cout<<"nums of plane = "<<vpPlane.size()<<endl;
                      float max_CosAngle = 0;
                      cv::Mat z_world = (cv::Mat_<float>(3,1)<<0.0, 1.0 ,0.0);
                      cout<<"z_world creat"<<endl;
                      for(size_t i=0; i<vpPlane.size(); i++)
                      {
                        float CosAngle = abs((float)(vpPlane[i]->n).dot(z_world));
                        cout<<"CosAngle = "<<CosAngle<<endl;
                        if(CosAngle > max_CosAngle)
                        {
                          max_CosAngle = CosAngle;
                          mpPlane = vpPlane[i];
                        }
                      }*/
                    if(mpPlane)
                    {
                        ox = mpPlane->o.at<float>(0,0);
                        oy = mpPlane->o.at<float>(1,0);
                        oz = mpPlane->o.at<float>(2,0);
                        Eigen::Vector3d euler_angles = mpPlane->Rpw.eulerAngles(0,1,2);
                        rx = (float)euler_angles[0];
                        ry = (float)euler_angles[1];
                        rz = (float)euler_angles[2];
                        //cout<<"mpPlane's euler"<<ox<<","<<oy<<","<<oz<<","<<rx<<","<<ry<<","<<rz<<endl;
                        ar->mpRotateCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,menu_size);
                        ar->mppCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,menu_size);


                    }
                    else
                    {
                        cout<<"No plane detected!"<<endl;
                    }
                    //if(flag_mouse)
                    //{

                    //flag_mouse = false;
                    //}
                    menuDetectPlane = false;
                }
                ar->mpRotateCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,menu_size);
                ar->mppCallBack->ChangePos(ox,-oy,-oz,rx,ry,rz,menu_size);
                /*
                //平面做相对位置用的代码
                Eigen::Matrix3d Rdw;
                Rdw = Eigen::AngleAxisd((double)rx*PI,Eigen::Vector3d::UnitZ())
                      *Eigen::AngleAxisd((double)ry*PI,Eigen::Vector3d::UnitY())
                      *Eigen::AngleAxisd((double)rz*PI,Eigen::Vector3d::UnitX());
                //Eigen::AngleAxisd Vdw ((rz,Eigen::Vector3d (0,0,1))*(ry,Eigen::Vector3d (0,1,0))*(rx,Eigen::Vector3d (1,0,0)));
                Eigen::AngleAxisd Vdw(Rdw);
                Eigen::Isometry3d Tdw = Eigen::Isometry3d::Identity();
                Tdw.rotate(Vdw); //
                Tdw.pretranslate(Eigen::Vector3d((double)ox,(double)oy,(double)oz));
                //Tdw << Rdw<float>(0,0),Rdw.at<float>(0,1),Rdw.at<float>(0,2),ox,
                //       Rdw.at<float>(1,0),Rdw.at<float>(1,1),Rdw.at<float>(1,2),oy,
                //       Rdw.at<float>(2,0),Rdw.at<float>(2,1),Rdw.at<float>(2)(2),oz,
                //       0.0 , 0.0 , 0.0 , 1.0;
                Eigen::Isometry3d Tpw = Eigen::Isometry3d::Identity();
                Eigen::AngleAxisd Vpw(mpPlane->Rpw);
                Tpw.rotate(Vpw);
                Tpw.pretranslate(Eigen::Vector3d(mpPlane->Tpw.at<double>(0,3),mpPlane->Tpw.at<double>(1,3),mpPlane->Tpw.at<double>(2,3)));
                //Tpw<<(mpPlane->Tpw).at<float>(0,0),(mpPlane->Tpw).at<float>(0,1),(mpPlane->Tpw).at<float>(0,2),(mpPlane->Tpw).at<float>(0,3),
                //     (mpPlane->Tpw).at<float>(1,0),(mpPlane->Tpw).at<float>(1,1),(mpPlane->Tpw).at<float>(1,2),(mpPlane->Tpw).at<float>(1,3),
                //     (mpPlane->Tpw).at<float>(2,0),(mpPlane->Tpw).at<float>(2,1),(mpPlane->Tpw).at<float>(2,2),(mpPlane->Tpw).at<float>(2,3),
                //     (mpPlane->Tpw).at<float>(3,0),(mpPlane->Tpw).at<float>(3,1),(mpPlane->Tpw).at<float>(3,2),(mpPlane->Tpw).at<float>(3,3);
                Tdp = Tpw*(Tdw.inverse());
                */
                /*
              if(mpPlane)
              {
                // Recompute plane if there has been a loop closure or global BA
                // In localization mode, map is not updated so we do not need to recompute
                //glPushMatrix();
              //  mpPlane->glTpw.Multiply();
              ///  DrawPlane(menu_ngrid,menu_sizegrid);
              //  glPopMatrix();
                bool bRecompute = false;
                if(!bLocalizationMode)
                {
                    if(mpSystem->MapChanged())
                    {
                        cout << "Map changed. All virtual elements are recomputed!" << endl;
                        bRecompute = true;
                    }

                    if(mpPlane)
                    {
                        if(bRecompute)
                        {
                            mpPlane->Recompute();
                            cout<<"plane is recomputed!"<<endl;*/
                /*
                //计算平面改变后的3D物体坐标
                Eigen::Isometry3d Tdw = Eigen::Isometry3d::Identity();
                Eigen::Isometry3d Tpw = Eigen::Isometry3d::Identity();
                Eigen::AngleAxisd Vpw(mpPlane->Rpw);
                Tpw.rotate(Vpw);
                Tpw.pretranslate(Eigen::Vector3d(mpPlane->Tpw.at<double>(0),mpPlane->Tpw.at<double>(1),mpPlane->Tpw.at<double>(2)));
                Tdw = Tpw * (Tdp.inverse());
                Eigen::Matrix3d Rdw = Eigen::Matrix3d::Identity(3,3);
                Rdw << (double)Tdw.matrix()(0,0),(double)Tdw.matrix()(0,1),(double)Tdw.matrix()(0,2),
                       (double)Tdw.matrix()(1,0),(double)Tdw.matrix()(1,1),(double)Tdw.matrix()(1,2),
                       (double)Tdw.matrix()(2,0),(double)Tdw.matrix()(2,1),(double)Tdw.matrix()(2,2);
                Eigen::Vector3d tdw;
                tdw << (double)Tdw.matrix()(3,0),(double)Tdw.matrix()(3,1),(double)Tdw.matrix()(3,2);

                      ox = (float)tdw(0,0);
                      oy = (float)tdw(1,0);
                      oz = (float)tdw(2,0);
                      Eigen::Vector3d euler_angles = Rdw.eulerAngles(0,1,2);
                      rx = (float)euler_angles[0];
                      ry = (float)euler_angles[1];
                      rz = (float)euler_angles[2];
                */
                //    }
                //    }
                //    }
//}

                //cout<<"ox = "<<ox<<"\t"<<"oy = "<<oy<<"\t"<<"oz = "<<oz<<"\t"<<"rx = "<<rx<<"\t"<<"ry = "<<ry<<"\t"<<"rz = "<<rz<<endl;

            }
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            if(menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            if(menuShowPoints)
                mpMapDrawer->DrawMapPoints();
            pangolin::glDrawAxis(0.5f);
            pangolin::FinishFrame();

            cv::Mat im = mpFrameDrawer->DrawFrame();
            float x=Twc.m[12],y=Twc.m[13],z=Twc.m[14];
            cv::putText(im,"x "+std::to_string(x),cv::Point(20,40),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,0,255),2,8);
            cv::putText(im,"y "+std::to_string(y),cv::Point(20,80),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,255,0),2,8);
            cv::putText(im,"z "+std::to_string(z),cv::Point(20,120),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(255,0,0),2,8);
            cv::imshow("Current Frame",im);
            cv::waitKey(mT);

            if(menuReset)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if(bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;
            }
            if(menuShutDown)
            {
                mpSystem->ShutdownRequest();
                menuShutDown = false;
            }

            if(Stop())
            {
                while(isStopped())
                {
                    usleep(3000);
                }
            }

            if(CheckFinish())  break;
        }

        SetFinish();
    }
    void Viewer::DrawPlane(int ndivs, float ndivsize)
    {
        // Plane parallel to x-z at origin with normal -y
        const float minx = -ndivs*ndivsize;
        const float minz = -ndivs*ndivsize;
        const float maxx = ndivs*ndivsize;
        const float maxz = ndivs*ndivsize;


        glLineWidth(2);
        glColor3f(0.7f,0.7f,1.0f);
        glBegin(GL_LINES);

        for(int n = 0; n<=2*ndivs; n++)
        {
            glVertex3f(minx+ndivsize*n,0,minz);
            glVertex3f(minx+ndivsize*n,0,maxz);
            glVertex3f(minx,0,minz+ndivsize*n);
            glVertex3f(maxx,0,minz+ndivsize*n);
        }

        glEnd();

    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if(!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if(mbFinishRequested)
            return false;
        else if(mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

    void Viewer::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM2::MapPoint*> &vMPs)
    {

        unique_lock<mutex> lock(mMutexPoseImage);
        mImage = im.clone();
//    if(mImage.empty()) cout<<"mImage is empty"<<endl;
//    else cout<<"mImage is not empty"<<endl;
        mTcw = Tcw.clone();
        mStatus = status;
        mvKeys = vKeys;
        mvMPs = vMPs;
    }

    void Viewer::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs)
    {
        unique_lock<mutex> lock(mMutexPoseImage);
        im = mImage.clone();
        Tcw = mTcw.clone();
        status = mStatus;
        vKeys = mvKeys;
        vMPs = mvMPs;
    }

    Plane* Viewer::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations)
    {
        // Retrieve 3D points
        vector<cv::Mat> vPoints;
        vPoints.reserve(vMPs.size());
        vector<MapPoint*> vPointMP;
        vPointMP.reserve(vMPs.size());

        for(size_t i=0; i<vMPs.size(); i++)
        {
            MapPoint* pMP=vMPs[i];
            if(pMP)
            {
                if(pMP->Observations()>5)	//地图点存在且观测数大于5
                {
                    vPoints.push_back(pMP->GetWorldPos());	//地图点坐标
                    //cout<<pMP->GetWorldPos().at<float>(0,0)<<","<<pMP->GetWorldPos().at<float>(1,0)<<","<<pMP->GetWorldPos().at<float>(2,0)<<";"<<endl;
                    vPointMP.push_back(pMP);	//地图点
                }
            }
        }

        const int N = vPoints.size();

        if(N<50)
            return NULL;


        // Indices for minimum set selection
        vector<size_t> vAllIndices;
        vAllIndices.reserve(N);
        vector<size_t> vAvailableIndices;

        for(int i=0; i<N; i++)
        {
            vAllIndices.push_back(i);
        }

        float bestDist = 1e5;
        vector<float> bestvDist;

        //RANSAC
        for(int n=0; n<iterations; n++)	//迭代求存在且观测数大于5的地图点的最佳法线
        {
            vAvailableIndices = vAllIndices;

            cv::Mat A(3,4,CV_32F);
            A.col(3) = cv::Mat::ones(3,1,CV_32F);

            // Get min set of points
            for(short i = 0; i < 3; ++i)	//随机选3个点做最小集？
            {
                int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

                int idx = vAvailableIndices[randi];

                A.row(i).colRange(0,3) = vPoints[idx].t();

                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }

            cv::Mat u,w,vt;
            cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);	//	svd分解

            const float a = vt.at<float>(3,0);	//最小集3点的平面法线
            const float b = vt.at<float>(3,1);
            const float c = vt.at<float>(3,2);
            const float d = vt.at<float>(3,3);
            const float vf = 1.0f/sqrt(a*a+b*b+c*c);
            const float nx = a*vf;	//单位法线
            const float ny = b*vf;
            const float nz = c*vf;
            //cout<<"nx,ny,nz = "<<nx<<","<<ny<<","<<nz<<endl;
            vector<float> vDistances(N,0);

            const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

            for(int i=0; i<N; i++)
            {
                vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;	//所有点在法线投影的距离
            }

            vector<float> vSorted = vDistances;
            sort(vSorted.begin(),vSorted.end());

            int nth = max((int)(0.2*N),20);
            const float medianDist = vSorted[nth];	//中等距离

            if(medianDist<bestDist)
            {
                bestDist = medianDist;
                bestvDist = vDistances;
            }
        }

        // Compute threshold inlier/outlier
        const float th = 1.4*bestDist;
        vector<bool> vbInliers(N,false);
        int nInliers = 0;
        for(int i=0; i<N; i++)
        {
            if(bestvDist[i]<th)
            {
                nInliers++;
                vbInliers[i]=true;	//小于门限设为内点
            }
        }

        vector<MapPoint*> vInlierMPs(nInliers,NULL);
        int nin = 0;
        //cout<<"MapPoint's world"<<endl;
        for(int i=0; i<N; i++)
        {
            if(vbInliers[i])
            {
                vInlierMPs[nin] = vPointMP[i];
                //cout<<"MapPoint"<<nin<<"("<<vInlierMPs[nin]->GetWorldPos().at<float>(0,0)<<","<<vInlierMPs[nin]->GetWorldPos().at<float>(1,0)<<","<<vInlierMPs[nin]->GetWorldPos().at<float>(2,0)<<")"<<endl;
                nin++;
            }
        }

        return new Plane(vInlierMPs,Tcw);
    }

    Plane::Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
    {
        rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
        Recompute();
    }

    void Plane::Recompute()
    {
        const int N = mvMPs.size();

        // Recompute plane with all points
        cv::Mat A = cv::Mat(N,4,CV_32F);
        A.col(3) = cv::Mat::ones(N,1,CV_32F);

        o = cv::Mat::zeros(3,1,CV_32F);

        int nPoints = 0;
        for(int i=0; i<N; i++)
        {
            MapPoint* pMP = mvMPs[i];
            if(!pMP->isBad())		//好地图点存入A矩阵
            {
                cv::Mat Xw = pMP->GetWorldPos();
                o+=Xw;
                cout << " 内点的坐标是 " << Xw.t() << endl;
                A.row(nPoints).colRange(0,3) = Xw.t();
                nPoints++;
            }
        }
        A.resize(nPoints);

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);	//对A进行svd分解

        float a = vt.at<float>(3,0);	//法线？
        float b = vt.at<float>(3,1);
        float c = vt.at<float>(3,2);

        o = o*(1.0f/nPoints);	//好点中心
        cout << " 内点的中心是 " << o.t() << endl;
        const float f = 1.0f/sqrt(a*a+b*b+c*c);

        // Compute XC just the first time
        if(XC.empty())
        {
            cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);	//twc = -Rwc*tcw
            XC = Oc-o;	//从好点中心o到相机光心Oc的向量
        }

        if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)	//法线方向
        {
            a=-a;
            b=-b;
            c=-c;
        }

        const float nx = a*f;	//单位法线
        const float ny = b*f;
        const float nz = c*f;
        //cout<<"recompute n = "<<nx<<","<<ny<<","<<nz<<endl;
        n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

        cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

        cv::Mat v = up.cross(n); //up和n叉乘，是y轴和法向量都垂直的向量
        const float sa = cv::norm(v); //v的范数 sin(e)
        //cout<<"sa = "<<sa<<endl;
        const float ca = up.dot(n); //up点乘n，是平面法向量在y轴上的投影 cos(e)
        float aca = acos(ca);
        //cout<<"ca = "<<ca<<endl;
        const float ang = atan2(sa,ca); //求反正切 arctan(sa/ca)=e
        //cout<<"ang = "<<ang<<endl;
        /*
        Eigen::AngleAxisd Vpw ( aca, Eigen::Vector3d ( v.at<float>(0,0),v.at<float>(1,0),v.at<float>(2,0) ) );
        Eigen::Vector3d eulerAngle_pw = Vpw.matrix().eulerAngles(0,1,2);
        rx = (float)eulerAngle_pw[0];
        ry = (float)eulerAngle_pw[1];
        rz = (float)eulerAngle_pw[2];
        cout<<"mpPlane's euler2"<<rx<<","<<ry<<","<<rz<<endl;
    */
        //Tpw = cv::Mat::eye(4,4,CV_32FC1);

        cv::Mat R = ExpSO3(v*ang/sa);
        cout << "ang is " << ang*180/3.14159267  << endl << " v is " << v << endl;
        cout << " v*ang/sa is "  << v*ang/sa << endl;
        Tpw = (cv::Mat_<float>(4,4)<<R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),o.at<float>(0,0),
                R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),o.at<float>(1,0),
                R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),o.at<float>(2,0),
                0,0,0,1);
        //Tpw.rowRange(0,3).colRange(0,3) = R;
        //Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
        //o.copyTo(Tpw.col(3).rowRange(0,3));
        //cout<<"Tpw"<<Tpw;
        glTpw.m[0] = Tpw.at<float>(0,0);
        glTpw.m[1] = Tpw.at<float>(1,0);
        glTpw.m[2] = Tpw.at<float>(2,0);
        glTpw.m[3]  = 0.0;

        glTpw.m[4] = Tpw.at<float>(0,1);
        glTpw.m[5] = Tpw.at<float>(1,1);
        glTpw.m[6] = Tpw.at<float>(2,1);
        glTpw.m[7]  = 0.0;

        glTpw.m[8] = Tpw.at<float>(0,2);
        glTpw.m[9] = Tpw.at<float>(1,2);
        glTpw.m[10] = Tpw.at<float>(2,2);
        glTpw.m[11]  = 0.0;

        glTpw.m[12] = Tpw.at<float>(0,3);
        glTpw.m[13] = Tpw.at<float>(1,3);
        glTpw.m[14] = Tpw.at<float>(2,3);
        glTpw.m[15]  = 1.0;

        Rpw = Eigen::Matrix3d::Identity(3,3);
        Rpw <<   Tpw.at<float>(0,0),Tpw.at<float>(0,1),Tpw.at<float>(0,2),
                Tpw.at<float>(1,0),Tpw.at<float>(1,1),Tpw.at<float>(1,2),
                Tpw.at<float>(2,0),Tpw.at<float>(2,1),Tpw.at<float>(2,2);
//    cout<<"    Rpw  ="<<   Tpw.at<float>(0,0)<<","<<Tpw.at<float>(0,1)<<","<<Tpw.at<float>(0,2)<<","<<
//    	     Tpw.at<float>(1,0)<<","<<Tpw.at<float>(1,1)<<","<<Tpw.at<float>(1,2)<<","<<
//    	     Tpw.at<float>(2,0)<<","<<Tpw.at<float>(2,1)<<","<<Tpw.at<float>(2,2);
    }

    Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
    {
        n = (cv::Mat_<float>(3,1)<<nx,ny,nz);	//normal	法线
        o = (cv::Mat_<float>(3,1)<<ox,oy,oz);	//origin	原点
        cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

        cv::Mat v = up.cross(n);	//v = up X n
        const float s = cv::norm(v);	//s = |v| = |n| sin<up,n>
        const float c = up.dot(n);		//c = up . n = |n| cos<up,n>
        const float a = atan2(s,c);		//a = tan<up,n>
        Tpw = cv::Mat::eye(4,4,CV_32F);
        const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
        //cout << rang;
        //Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);	//旋转向量tan<up,n>*v/|v|和rang*up
        //o.copyTo(Tpw.col(3).rowRange(0,3));	//平移向量tpw = o
        cv::Mat R = ExpSO3(v*a/s);
        Tpw = (cv::Mat_<float>(4,4)<<R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),o.at<float>(0,0),
                R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),o.at<float>(1,0),
                R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),o.at<float>(2,0),
                0,0,0,1);
        glTpw.m[0] = Tpw.at<float>(0,0);
        glTpw.m[1] = Tpw.at<float>(1,0);
        glTpw.m[2] = Tpw.at<float>(2,0);
        glTpw.m[3] = 0.0;

        glTpw.m[4] = Tpw.at<float>(0,1);
        glTpw.m[5] = Tpw.at<float>(1,1);
        glTpw.m[6] = Tpw.at<float>(2,1);
        glTpw.m[7] = 0.0;

        glTpw.m[8] = Tpw.at<float>(0,2);
        glTpw.m[9] = Tpw.at<float>(1,2);
        glTpw.m[10] = Tpw.at<float>(2,2);
        glTpw.m[11] = 0.0;

        glTpw.m[12] = Tpw.at<float>(0,3);
        glTpw.m[13] = Tpw.at<float>(1,3);
        glTpw.m[14] = Tpw.at<float>(2,3);
        glTpw.m[15] = 1.0;
    }



    void Viewer::on_mouse(int event, int x, int y, int flags, void* ustc)
    {
        Point prept = Point(-1, -1);
        Point curpt = Point(-1, -1);
        if (event == CV_EVENT_LBUTTONDOWN)    //左键按下
        {
            mouse_x = x;
            mouse_y = y;
            //cout<<"鼠标点击坐标：("<<x<<","<<y<<")"<<endl;
            flag_mouse = true;
        }
    }

}