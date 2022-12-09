#include "AR.h"

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/GraphicsWindow>
#include <osg/Group>
#include <osg/BufferObject>
#include <osg/LineSegment>
#include <osg/Point>
#include <osg/Quat>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osg/Depth>
#include <osgAnimation/BasicAnimationManager>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <sys/time.h>

#define QUEUESIZE 20.0
#define QUEUESIZE1 8.0

AR::AR(const int widht,const int height,const string loadpath,float fps)
{
	condition = false;
	iflight = false;
	mW = widht;
	mH = height;
	fps_ar = fps;
	mploader = new Loader(loadpath);
	mpcam = new CameraManipulator();
	mpcam->setLoader(mploader);
	screen = new osg::Image();
	mpDatabuffer = new BlockQueue<mpData>(BUFFERSIZE);

}

osg::ref_ptr<osg::Group> AR::createLight(osg::ref_ptr<osg::Node>node)
{
	osg::ref_ptr<osg::Group>lightRoot = new osg::Group();
	lightRoot->addChild(node);

	osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet();
	stateset = lightRoot->getOrCreateStateSet();
	stateset->setMode(GL_LIGHTING,osg::StateAttribute::ON);
	stateset->setMode(GL_LIGHT0,osg::StateAttribute::ON);

	osg::ref_ptr<osg::Light> light = new osg::Light;

	light->setLightNum(0);//启用第几个光源  OpenGL有8个光源

	//light->setAmbient(osg::Vec4(0.0f,1.0f,1.0f,1.0f));//环境光成分 太强影响照射 此处不用
	light->setAmbient(osg::Vec4(0.1,0.1,0.1,1.0));

	light->setDiffuse(osg::Vec4(1.0,1.0,1.0,1.0));//散射光成分

	//l->setSpecular()镜面光成分

	light->setDirection(osg::Vec3(0,0,-1));//方向

	light->setPosition(osg::Vec4(0,0,0,0));//位置

	lightSource = new osg::LightSource();
	lightSource->setLight(light.get());

	lightRoot->addChild(lightSource.get());
	return lightRoot.get();
}


void AR::run()
{
	osgViewer::Viewer* viewer = new osgViewer::Viewer();
	viewer->setCameraManipulator(mpcam);
	//申请一个定时器类
	osg::Timer *timer=new osg::Timer;
	osg::Timer_t start_timer=0;
	double start_timer1=0.0;

	//控制帧速使用睡眠时间
	double sleep_time=0.0;

	//每帧的开始时间和结束时间
	double per_str_time=0.0;
	double per_end_time=0.0;

	//每帧开始的时间戳存储队列（滑动窗口），滑动窗口平均每帧的时间 和 1/帧率 的差值
	queue<double> time_queue;
	float queue_t = 0;

	//每帧开始时的缓冲长度存储队列（滑动窗口），滑动窗口所有帧的缓冲队列长度的和
	queue<int> length_queue;
	int length_sum = 0;

	//得到一个tick值为多少Seconds
	cout<<timer->getSecondsPerTick()<<endl;
	start_timer=timer->tick();
	start_timer1=viewer->elapsedTime();
	timer->setStartTick();
/*
    osg::ref_ptr<osg::Node> animationNode = osgDB::readNodeFile("/home/wangchenyu/OpenSceneGraph-Data-3.0.0/会动的模型/15.ive");
    //获得节点的动画列表
    osg::ref_ptr<osgAnimation::BasicAnimationManager> anim =
            dynamic_cast<osgAnimation::BasicAnimationManager*>(animationNode->getUpdateCallback());
    const osgAnimation::AnimationList& list = anim->getAnimationList();
    while(1){
        cout<< (animationNode->getUpdateCallback()==NULL) <<endl;
    }
*/
	//从动画列表中选择一个动画，播放
	//if(list[0].get()== NULL){
	//  cout<<"no no no"<<endl;
	//} else{
	//  cout<<"yes"<<endl;
	//}
	//anim->playAnimation(list[0].get());

	osg::ref_ptr<osg::Camera>  _camera = viewer->getCamera();
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits();
	traits->x=0;
	traits->y=0;
	traits->width=mW;
	traits->height=mH;
	traits->windowDecoration =true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	cout<<"窗口mW"<<mW<<endl;
	cout<<"窗口mH"<<mH<<endl;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> texture2 = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> texture3 = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> texture4 = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> texture5 = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> texture6 = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> texture7 = new osg::Texture2D;
	osg::ref_ptr<osg::Texture2D> texture8 = new osg::Texture2D;
	texture->setDataVariance(osg::Object::DYNAMIC);
	texture2->setDataVariance(osg::Object::DYNAMIC);
	texture3->setDataVariance(osg::Object::DYNAMIC);
	texture4->setDataVariance(osg::Object::DYNAMIC);
	texture5->setDataVariance(osg::Object::DYNAMIC);
	texture6->setDataVariance(osg::Object::DYNAMIC);
	texture7->setDataVariance(osg::Object::DYNAMIC);
	texture8->setDataVariance(osg::Object::DYNAMIC);

	//texture->setImage( image.get() );
	//osg::ref_ptr<osg::Drawable> quad = osg::createTexturedQuadGeometry(osg::Vec3(0.1f,0.0f,0.0f), osg::Vec3(0.9f, 0.0f, 1.0f), osg::Vec3(0.0f, 0.71111f, 1.0f),0.0f,0.0f,1.0f,1.0f);

//分辨率  1280×720
//    osg::ref_ptr<osg::Drawable> quad = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.2888889f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.7111111f, 0.0f),0.0f,1.0f,1.0f,0.0f);
//    osg::ref_ptr<osg::Drawable> quad2 = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.111111f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.1777778f, 0.0f),0.0f,1.0f,1.0f,0.0f);
//  osg::ref_ptr<osg::Drawable> quad3 = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.022222f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.0888889f, 0.0f),0.0f,1.0f,1.0f,0.0f);
//  osg::ref_ptr<osg::Drawable> quad4 = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.022222f, 0.0f),0.0f,1.0f,1.0f,0.0f);
//  osg::ref_ptr<osg::Drawable> quad5 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.2888889f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.7111111f, 0.0f),0.0f,1.0f,1.0f,0.0f);
//  osg::ref_ptr<osg::Drawable> quad6 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.111111f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.1777778f, 0.0f),0.0f,1.0f,1.0f,0.0f);
//  osg::ref_ptr<osg::Drawable> quad7 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.022222f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.0888889f, 0.0f),0.0f,1.0f,1.0f,0.0f);
//  osg::ref_ptr<osg::Drawable> quad8 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.0f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.022222f, 0.0f),0.0f,1.0f,1.0f,0.0f);

//分辨率  640×480
	osg::ref_ptr<osg::Drawable> quad = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.4666667f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.5333333f, 0.0f),0.0f,1.0f,1.0f,0.0f);
	osg::ref_ptr<osg::Drawable> quad2 = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.2f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.2666667f, 0.0f),0.0f,1.0f,1.0f,0.0f);
	osg::ref_ptr<osg::Drawable> quad3 = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.066667f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.1333333f, 0.0f),0.0f,1.0f,1.0f,0.0f);
	osg::ref_ptr<osg::Drawable> quad4 = osg::createTexturedQuadGeometry(osg::Vec3(0.0f,0.0f,0.0f), osg::Vec3(0.8f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.066667f, 0.0f),0.0f,1.0f,1.0f,0.0f);
	osg::ref_ptr<osg::Drawable> quad5 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.4666667f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.5333333f, 0.0f),0.0f,1.0f,1.0f,0.0f);
	osg::ref_ptr<osg::Drawable> quad6 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.2f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.2666667f, 0.0f),0.0f,1.0f,1.0f,0.0f);
	osg::ref_ptr<osg::Drawable> quad7 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.066667f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.1333333f, 0.0f),0.0f,1.0f,1.0f,0.0f);
	osg::ref_ptr<osg::Drawable> quad8 = osg::createTexturedQuadGeometry(osg::Vec3(0.8f,0.0f,0.0f), osg::Vec3(0.2f, 0.0f, 0.0f), osg::Vec3(0.0f, 0.066667f, 0.0f),0.0f,1.0f,1.0f,0.0f);

	quad->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture.get());
	quad2->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture2.get());
	quad3->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture3.get());
	quad4->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture4.get());
	quad5->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture5.get());
	quad6->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture6.get());
	quad7->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture7.get());
	quad8->getOrCreateStateSet()->setTextureAttributeAndModes( 0,texture8.get());


	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable( quad.get() );
	geode->addDrawable( quad2.get() );
	geode->addDrawable( quad3.get() );
	geode->addDrawable( quad4.get() );
	geode->addDrawable( quad5.get() );
	geode->addDrawable( quad6.get() );
	geode->addDrawable( quad7.get() );
	geode->addDrawable( quad8.get() );


	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setClearMask( 0 );
	camera->setCullingActive( false );
	camera->setAllowEventFocus( false );
	camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
	camera->setRenderOrder( osg::Camera::POST_RENDER );
	camera->setProjectionMatrix( osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0) );
	camera->addChild( geode.get() );
	camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);

	osg::StateSet* ss = camera->getOrCreateStateSet();
	//ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
	ss->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 1.0, 1.0) );

	_camera->setViewport(0,0,mW,mH);

	viewer->getCamera()->setGraphicsContext(gc.get());

	//_camera->setProjectionMatrixAsFrustum(L,R,B,T,N,F);
	//_camera->setProjectionMatrix(osg::Matrixd::frustum(L, R , B , T, N, F));
	_camera->setProjectionMatrix(osg::Matrixd::frustum(L, R , T , B, N, F));
	osg::Matrixd d = _camera->getProjectionMatrix();
	double fovy, aspectRatio, zNear,zFar;
	d.getPerspective(fovy,aspectRatio,zNear,zFar);
	osg::Vec3 mPos = osg::Vec3(-0.582199, -0.0161676,0.219525);
	osg::Quat mQuat = osg::Quat(-0.117829,-0.512224,0.203997,0.825911);
	osg::Matrixd mat;
	osg::Matrixd mat2;
	mat2 = mat * osg::Matrixd::rotate(mQuat);
	//_camera->setViewMatrix(osg::Matrixd::inverse(mat2 * osg::Matrixd::translate(mPos)));
	_camera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);
	//_camera->setViewMatrixAsLookAt(osg::Vec3(-0.582199,-0.0161676,0.219525),osg::Vec3(0,0,0),osg::Vec3(0,0,1));//0.1915 -0.0202555,0.0913102,0.21486
/*
	while(1){
	cout<<"fovy"<<fovy<<endl;
cout<<"aspectRatio"<<aspectRatio<<endl;
cout<<"zNear"<<zNear<<endl;
cout<<"zFar"<<zFar<<endl;
}
*/
	cout<<"L"<<L<<endl;
	cout<<"R"<<R<<endl;
	cout<<"B"<<B<<endl;
	cout<<"T"<<T<<endl;
	cout<<"N"<<N<<endl;
	cout<<"F"<<F<<endl;

	_camera->setClearColor(osg::Vec4((float)back_color[0]/255, (float)back_color[1]/255,(float)back_color[2]/255, 1.0));

	osg::ref_ptr<osg::Group> root =new osg::Group();

	mpRotateCallBack = new RotateCallBack();
	mppCallBack = new pCallBack();

	mploader->objs[0]->pat->setUpdateCallback(mpRotateCallBack);

	mploader->add2group(root);



	root->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON /*| osg::StateAttribute::OVERRIDE*/);  //对根场景设置光照模式关闭
	//root->getOrCreateStateSet()->setMode(GL_LIGHT0,osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	root->addChild(mploader->objs[0]->pat.get());        //将模型obj[0]添加到根场景中
	//root->addChild( animationNode);
	osgUtil::Optimizer optimizer;
	optimizer.optimize(root.get());
	root->addChild( camera.get() );
	viewer->getCamera()->setPostDrawCallback(new CaptureCallback(screen.get(),mW,mH)) ;
	viewer->setSceneData(root.get());
	osg::PositionAttitudeTransform* plane =dynamic_cast<osg::PositionAttitudeTransform*>(viewer->getSceneData()->asGroup()->getChild(1));
	plane->setUpdateCallback(mppCallBack);
	viewer->addEventHandler( new osgViewer::StatsHandler() );
	viewer->addEventHandler( new osgViewer::WindowSizeHandler() );
	viewer->addEventHandler( new osgViewer::HelpHandler );
	viewer->addEventHandler(new osgViewer::ScreenCaptureHandler);
	viewer->addEventHandler(new osgViewer::RecordCameraPathHandler);

	viewer->realize();

	mpData mDataStruct; //结构体对象，详情参见AR.h
	int counts = 0; //ar显示帧的计数器
	int counts1 = 0; //缓冲队列长度为2的连续的帧数
	bool flag1 = 0;
	double light,light1,light2;
	while (!viewer->done())//这里不能写成return viewer.run()这个函数会对场景中是否有漫游器进行判断，如果没有，就会加上默认漫游器。
	{
		fps_ar = 30.5; //有额外花销，所以帧率用30.5
//        fps_ar = 15.5;
		//睡眠60ms，使其缓冲两帧
		if(flag1 == 0)
		{
//			usleep(60000);
            usleep(30000);
			flag1 = 1;
		}

		per_str_time=timer->tick(); //得到当前时间，每帧的开始时间

		//长度为QUEUESIZE的滑动窗口队列，存放每帧开始时的时间
		if(time_queue.size() == QUEUESIZE)
		{
			time_queue.pop();
			time_queue.push(per_str_time);
		}
		else
		{
			time_queue.push(per_str_time);
		}

		//长度为QUEUESIZE1的滑动窗口队列，存放每帧开始前队列的长度
		int queue_size = mpDatabuffer->GetSizeQueue();
		if(length_queue.size() == QUEUESIZE1)
		{
			length_sum = length_sum - length_queue.front();
			length_queue.pop();
			length_queue.push(queue_size);
			length_sum = length_sum +queue_size;
		}
		else
		{
			length_queue.push(queue_size);
			length_sum = length_sum + queue_size;
		}
		//缓冲队列长度是2的连续的帧数，以此判断显示稳定性
		if(queue_size == 2) counts1++;
		else counts1 = 0;

		counts++; //AR显示的帧的计数

		mpDatabuffer->Take(mDataStruct); //从缓冲队列中取对象

		cout << " take 出 mpDataStruct 的 img_timestamp 是 " << mDataStruct.img_timestamp << endl;
		cv::Mat back_total;

		if(mDataStruct.idx_image == 0)
		{
			back_total = (mDataStruct.ImageArray2[BUFFERSIZE-1]).clone();
		}
		else
		{
			back_total = (mDataStruct.ImageArray2[mDataStruct.idx_image - 1]).clone();
		}


//        cv::Size imageSize(640,480);
//        cv::Mat distCoeffs,cameraMatrix,newcameraMatrix,map1,map2,newimage;
//        distCoeffs = (cv::Mat_<double>(4,1)<<-0.26901626586914062,0.05582427978515625,-0.00033950805664062,0.00159072875976562);
//        cameraMatrix = (cv::Mat_<double>(3,3)<<348.83370971679687500,0,314.07852172851562500,0,349.76763916015625000,237.41012573242187500,0,0,1);
//        initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
//                                imageSize, CV_16SC2, map1, map2);
//        remap(back_total,newimage,map1,map2,cv::INTER_LINEAR);


		if(iflight) {
			cv::Mat hsv;
			cv::cvtColor(back_total, hsv, CV_RGB2HSV);;//RGB to HSV
//            cv::cvtColor(newimage, hsv, CV_RGB2HSV);;//RGB to HSV
			light = mean(hsv).val[2];//计算HSV的均值
			//light1 = light;
			light1 = light2 - light;//计算两帧之间亮度差值
			light2 = light1;
			if (light1 > 30)//若变化过大，则进行中和
			{
				light = (light1 + light2) / 2;
				cout << "亮度突变" << endl;
			}
			//cout << light << endl;
			if (light > 150) light = 150;
			if (light < 80) light = 80;
			condition = (light - 80) / 70;
			//光照调节 只设置setAmbient，范围从0~1

			osg::ref_ptr<osg::Light> light = new osg::Light;

			light->setLightNum(0);//启用第几个光源  OpenGL有8个光源

			//light->setAmbient(osg::Vec4(0.0f,1.0f,1.0f,1.0f));
			light->setAmbient(osg::Vec4(1.0 * condition + 0.1, 1.0 * condition + 0.1, 1.0 * condition + 0.1, 1.0));

			light->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));//散射光成分，不调节

			//l->setSpecular()镜面光成分

			light->setDirection(osg::Vec3(0, 0, -1));//方向

			light->setPosition(osg::Vec4(0, 0, 0, 0));//位置

			mploader->lightSource->setLight(light.get());
		}
		else{
			osg::ref_ptr<osg::Light> light = new osg::Light;

			light->setLightNum(0);//启用第几个光源  OpenGL有8个光源

			//light->setAmbient(osg::Vec4(0.0f,1.0f,1.0f,1.0f));
			light->setAmbient(osg::Vec4(1.0 , 1.0 , 1.0, 1.0));

			light->setDiffuse(osg::Vec4(1.0, 1.0, 1.0, 1.0));//散射光成分，不调节

			//l->setSpecular()镜面光成分

			light->setDirection(osg::Vec3(0, 0, -1));//方向

			light->setPosition(osg::Vec4(0, 0, 0, 0));//位置

			mploader->lightSource->setLight(light.get());
		}


		//分割图片
//		cv::Rect rec1(0,0,1024,512);
//		cv::Rect rec2(0,512,1024,128);
//		cv::Rect rec3(0,640,1024,64);
//		cv::Rect rec4(0,704,1024,16);
//		cv::Rect rec5(1024,0,256,512);
//		cv::Rect rec6(1024,512,256,128);
//		cv::Rect rec7(1024,640,256,64);
//		cv::Rect rec8(1024,704,256,16);
		cv::Rect rec1(0,0,512,256);
		cv::Rect rec2(0,256,512,128);
		cv::Rect rec3(0,384,512,64);
		cv::Rect rec4(0,448,512,32);
		cv::Rect rec5(512,0,128,256);
		cv::Rect rec6(512,256,128,128);
		cv::Rect rec7(512,384,128,64);
		cv::Rect rec8(512,448,128,32);

		cv::Mat back1_(back_total,rec1);
		cv::Mat back2_(back_total,rec2);
		cv::Mat back3_(back_total,rec3);
		cv::Mat back4_(back_total,rec4);
		cv::Mat back5_(back_total,rec5);
		cv::Mat back6_(back_total,rec6);
		cv::Mat back7_(back_total,rec7);
		cv::Mat back8_(back_total,rec8);

//        cv::Mat back1_(newimage,rec1);
//        cv::Mat back2_(newimage,rec2);
//        cv::Mat back3_(newimage,rec3);
//        cv::Mat back4_(newimage,rec4);
//        cv::Mat back5_(newimage,rec5);
//        cv::Mat back6_(newimage,rec6);
//        cv::Mat back7_(newimage,rec7);
//        cv::Mat back8_(newimage,rec8);

		cv::Mat  back1=back1_.clone();
		cv::Mat  back2=back2_.clone();
		cv::Mat  back3=back3_.clone();
		cv::Mat  back4=back4_.clone();
		cv::Mat  back5=back5_.clone();
		cv::Mat  back6=back6_.clone();
		cv::Mat  back7=back7_.clone();
		cv::Mat  back8=back8_.clone();

		osg::ref_ptr<osg::Image> image1 = new osg::Image;
		osg::ref_ptr<osg::Image> image2 = new osg::Image;
		osg::ref_ptr<osg::Image> image3 = new osg::Image;
		osg::ref_ptr<osg::Image> image4 = new osg::Image;
		osg::ref_ptr<osg::Image> image5 = new osg::Image;
		osg::ref_ptr<osg::Image> image6 = new osg::Image;
		osg::ref_ptr<osg::Image> image7 = new osg::Image;
		osg::ref_ptr<osg::Image> image8 = new osg::Image;

		image1->setImage(back1.cols,back1.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back1.data,osg::Image::NO_DELETE,1);
		image2->setImage(back2.cols,back2.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back2.data,osg::Image::NO_DELETE,1);
		image3->setImage(back3.cols,back3.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back3.data,osg::Image::NO_DELETE,1);
		image4->setImage(back4.cols,back4.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back4.data,osg::Image::NO_DELETE,1);
		image5->setImage(back5.cols,back5.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back5.data,osg::Image::NO_DELETE,1);
		image6->setImage(back6.cols,back6.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back6.data,osg::Image::NO_DELETE,1);
		image7->setImage(back7.cols,back7.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back7.data,osg::Image::NO_DELETE,1);
		image8->setImage(back8.cols,back8.rows,3,GL_RGB,GL_RGB,GL_UNSIGNED_BYTE,back8.data,osg::Image::NO_DELETE,1);

		texture->setImage( image1.get() );
		texture2->setImage( image2.get() );
		texture3->setImage( image3.get() );
		texture4->setImage( image4.get() );
		texture5->setImage( image5.get() );
		texture6->setImage( image6.get() );
		texture7->setImage( image7.get() );
		texture8->setImage( image8.get() );

		mpcam->setPos( mDataStruct.t0 , mDataStruct.t1 , mDataStruct.t2 );
		mpcam->setQuat( mDataStruct.x, mDataStruct.y, mDataStruct.z, mDataStruct.w);
//        cout << " pose_timestamp " <<  mDataStruct.pose_timestamp << endl;
//        cout << " img_timestamp  "  <<  mDataStruct.img_timestamp  << endl;
		/*
        控制帧速的公式为：每帧睡眠的时间=期望每帧绘制的时间-实际绘制的时间
        假如：帧速为x,那么每帧绘制的时间为：1.0/x(秒)，实际绘制的的时间为：t
        那么需要睡眠的时间=1.0/x-t
        然后有三个调整机制：
        1.滑动窗口调整---连续QUEUESIZE帧的总体的每帧的平均时间queue_t 的多少 对睡眠时间的影响。
        2.为了控制队列长度保持在2,所以连续QUEUESIZE1帧队列如果都是3以上或1以下就调整帧率。
        3.极端的AR非常慢的情况，就让其丢帧不睡眠，不影响后面的显示。
        */
		if(queue_size - 1 <= 2)
		{
			viewer->frame(); //渲染显示
		}

		per_end_time=timer->tick();

		if(counts1 >= QUEUESIZE1) /*fps_ar = 30.5;*/fps_ar =15.5;
		if(counts < QUEUESIZE)
		{
			queue_t = 0;
		}
		else
		{
			if(length_sum >= (QUEUESIZE1*3-3))
			{
				fps_ar = 30.5+3;
//                fps_ar =15.5 + 1.5;
				counts1 = 0;
			}
			else if(length_sum <= (QUEUESIZE1*1+2))
			{
				fps_ar = 30.5-3;
//                fps_ar =15.5 -1.5;
				counts1 = 0;
			}
			queue_t = (timer->delta_s(time_queue.front(),per_end_time))/QUEUESIZE - 1.0/fps_ar;
		}

		//delta_s是计算per_str_time，per_end_time之间的差值，就是per_end_time-per_str_time
		sleep_time=1.0/fps_ar - timer->delta_s(per_str_time,per_end_time) - queue_t;

		//sleep_time是得到每帧应当停顿睡眠的时间，如果sleep_time的值小于0，则说明实际绘制的时间大于期望的时间
		//因此此时就不需要再睡眠了，如果在继续睡眠，则效率会更差
		if (sleep_time<0)
		{
			sleep_time=0.0;
		}
		//缓存队列长度大于3时应该不睡眠，前面缓存队列长度大于3时不显示，相当于直接扔掉该帧。
		if(queue_size - 1 <= 2)
		{
			OpenThreads::Thread::microSleep(sleep_time*1000000);
		}
	}
}

void AR::setProject(float fx,float fy,float cx,float cy,float zNear,float zFar)
{
	L = -(cx) * zNear / fx;
	R = +(mW-cx) * zNear / fx;
	T = -(cy) * zNear / fy;
	B = +(mH-cy) * zNear / fy;
	N = zNear;
	F = zFar;
	cout<<"fx"<<fx<<endl;
	cout<<"fy"<<fy<<endl;
	cout<<"cx"<<cx<<endl;
	cout<<"cy"<<cy<<endl;
	cout<<"mH"<<mH<<endl;
	cout<<"mW"<<mW<<endl;
}

cv::Mat* AR::getScreen()
{
	return new cv::Mat(screen->t(),screen->s(),CV_8UC3,screen->data());
}