#ifndef AR_H
#define AR_H

#include "Loader.h"
#include "CameraManipulator.h"

#include <queue>
#include <mutex>
#include <condition_variable>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <osg/Camera>

#include<osgDB/ReadFile>
#include <string>
#include <thread>
#include <osg/Texture2D>
#include "Pickhandle.h"


#define BUFFERSIZE 5
//#include <semaphore.h>
using namespace std;

struct mpData
{
  float t0,t1,t2,x,y,z,w;
  cv::Mat ImageArray2[BUFFERSIZE];
  int idx_image = 0;
};



 class RotateCallBack: public osg::NodeCallback
 {

 private:
    double _rotateZ;
    float px,py,pz,ax,ay,az,sx,sy,sz;

 public:
    RotateCallBack():_rotateZ(0.0) {}
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
         osg::PositionAttitudeTransform* pat =
             dynamic_cast<osg::PositionAttitudeTransform*>(node);
         if(pat)
         {
             //osg::Vec3 vec(0, 0, 1);
             //osg::Quat quat = osg::Quat(osg::DegreesToRadians(_rotateZ), osg::Z_AXIS);
             //pat->setAttitude(quat);
             pat->setPosition(osg::Vec3(px,py,pz));	//位置
             pat->setScale(osg::Vec3(sx,sy,sz)); //大小
             osg::Vec3 x(1,0,0),y(0,1,0),z(0,0,1);
             pat->setAttitude(osg::Quat(ax,x)*osg::Quat(ay,y)*osg::Quat(az,z)*osg::Quat(-1.57,x));	//角度
            //pat->setAttitude(osg::Quat(ax,x)*osg::Quat(ay,y)*osg::Quat(az,z)*osg::Quat(3.14,z)*osg::Quat(1.57,x));	//角度
             //_rotateZ += 0.10;
         }

         //traverse(node, nv);
     }
    void ChangePos(float px_,float py_,float pz_,float ax_,float ay_,float az_,float s)
    {
      	px = px_;
      	py = py_;
      	pz = pz_;
      	ax = ax_;
      	ay = ay_;
      	az = az_;
        sx = s;
        sy = s;
        sz = s;
    }
 };

 class pCallBack: public osg::NodeCallback
 {

 private:
    double _rotateZ;
    float px,py,pz,ax,ay,az,sx,sy,sz;

 public:
   pCallBack():_rotateZ(0.0) {}
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
         osg::PositionAttitudeTransform* pat =
             dynamic_cast<osg::PositionAttitudeTransform*>(node);
         if(pat)
         {
             //osg::Vec3 vec(0, 0, 1);
             //osg::Quat quat = osg::Quat(osg::DegreesToRadians(_rotateZ), osg::Z_AXIS);
             //pat->setAttitude(quat);
             //float px,py,pz,ax,ay,az;
             //cout<<"px = "<<px<<endl;
             //cout<<"py = "<<py<<endl;
             //cout<<"pz = "<<pz<<endl;
             pat->setPosition(osg::Vec3(px,py,pz));	//位置
             pat->setScale(osg::Vec3(sx,sy,sz)); //大小
             osg::Vec3 x(1,0,0),y(0,1,0),z(0,0,1);
             pat->setAttitude(osg::Quat(ax,x)*osg::Quat(ay,y)*osg::Quat(az,z)*osg::Quat(-1.57,x));	//角度
            //pat->setAttitude(osg::Quat(ax,x)*osg::Quat(ay,y)*osg::Quat(az,z)*osg::Quat(3.14,z)*osg::Quat(1.57,x));	//角度
             //_rotateZ += 0.10;
         }
         //traverse(node, nv);
     }
    void ChangePos(float px_,float py_,float pz_,float ax_,float ay_,float az_,float s)
    {
       px = px_;
       py = py_;
       pz = pz_;
       ax = ax_;
       ay = ay_;
       az = az_;
       sx = s;
       sy = s;
       sz = s;
    }
 };

template <typename T> class BlockQueue
{
public:
 	  //类的构造函数，初始化对象使用（可定义长度）
    BlockQueue(int maxSize)
    {
      m_maxSize = maxSize;
    }
    BlockQueue(){m_maxSize = 3;}

    ~BlockQueue(){;}
 		//向队列中加数据，如果满则阻塞
    void Put(const T& x)
    {
        unique_lock<mutex> lock(m_mutexqueue);

        while (m_queue.size() == m_maxSize)
        {
            //cout << "the blocking queue is full,waiting..." << endl;
            m_notFull.wait(m_mutexqueue);
        }
        m_queue.push(x);
        m_notEmpty.notify_one();
    }
 		//从队列中取数据，如果空则阻塞
    void Take( T& x)
    {
        unique_lock<mutex> lock(m_mutexqueue);

        while (m_queue.empty())
        {
             //cout << "the blocking queue is empty,wating..." << endl;
            m_notEmpty.wait(m_mutexqueue);
        }

        x = m_queue.front();
        m_queue.pop();
        m_notFull.notify_one();
    }

    int GetSizeQueue()
    {
	     return m_queue.size();
    }

private:
    queue<T> m_queue;                  //缓冲区
    mutex m_mutexqueue;                    //互斥量和条件变量结合起来使用
    condition_variable_any m_notEmpty;//不为空的条件变量
    condition_variable_any m_notFull; //没有满的条件变量
    unsigned int m_maxSize;                         //同步队列最大的size
};

class AR
{
public:
    PickHandle* pick;

    double condition;
    bool iflight;
    osg::ref_ptr<osg::LightSource> lightSource;

	int mW,mH;
    float fps_ar;
	bool new_a, new_w;
	osg::ref_ptr<osg::Image> screen;
    cv::Mat K;

	Loader* mploader;
	CameraManipulator* mpcam;
	RotateCallBack* mpRotateCallBack;
    pCallBack* mppCallBack;
	RotateCallBack* mpRotateCallBack1;
    BlockQueue<mpData>* mpDatabuffer;

	float L,R,T,B,N,F;
	int back_color[3];
    osg::ref_ptr<osg::Group> createLight(osg::ref_ptr<osg::Node>node);
	AR(const int widht,const int height,const string loadpath,float fps);
    //void GetDelay(int total_idx,int extra_idx);
	AR(){condition = false;
    iflight = false;}
	~AR(){;}

  struct CaptureCallback:public osg::Camera::DrawCallback
  {
  public:
    int mW,mH;
  	std::mutex mutex;
    CaptureCallback(osg::ref_ptr<osg::Image> image,int w,int h)
	  {
      image_ = image ;
      mW=w;
      mH=h;
      //mutex = *(lock);
    }
    ~CaptureCallback(){;}

    virtual void operator()(const osg::Camera &camera) const
    {
      //unique_lock<mutex> lock(mutex);
      //分配一个image
      image_->allocateImage(mW,mH , 1 , GL_RGB , GL_UNSIGNED_BYTE) ;
      //读取像素信息抓图
      image_->readPixels(0 , 0 , mW , mH , GL_RGB , GL_UNSIGNED_BYTE) ;
    }

  private:
    osg::ref_ptr<osg::Image> image_ ;
};

    void run();

	void setProject(float fx,float fy,float cx,float cy,float zNear,float zFar);

	cv::Mat* getScreen();

  //std::mutex mMutexTcw;
};

#endif
