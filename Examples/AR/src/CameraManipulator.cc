#include "CameraManipulator.h"
#include <osg/Group>

	CameraManipulator::CameraManipulator():
		m_fMoveSpeed(1.0f),m_bLeftButtonDown(false),
		m_fpushX(0),m_fpushY(0)
	{
		mPos = osg::Vec3(0.0f, 0.0f, 0.0f);
		mQuat = osg::Quat(0,osg::Vec3(1,0,0));
	}

	void CameraManipulator::setLoader(Loader* loader)
	{
		mploader = loader;
		size_objs = loader -> size_objs;
		size_pics = loader -> size_pics;
	}

	void CameraManipulator::setPos(float x,float y,float z)
	{
		//cout<<__FILE__<<"\t"<<__LINE__<<endl;
		mPos = osg::Vec3(x,-y,z);
		//mPos = osg::Vec3(x,y,z);

	}

	void CameraManipulator::setQuat(float x,float y,float z,float w)
	{
		//cout<<__FILE__<<"\t"<<__LINE__<<endl;
		mQuat = osg::Quat(-x,y,-z,w);
		//mQuat = osg::Quat(x,y,z,w);
	}

	// 把漫游器添加到场景中
	CameraManipulator * CameraManipulator::CameraToScence(osg::ref_ptr<osgViewer::Viewer>viewer)
	{
		CameraManipulator* camera = new CameraManipulator;

		viewer->setCameraManipulator(camera);

		camera->m_pHostViewer = viewer;

		return camera;
	}

	// 获取矩阵
	osg::Matrixd CameraManipulator::getMatrix() const
	{
		osg::Matrixd mat;
		osg::Matrixd mat2;
		mat2 = mat * osg::Matrixd::rotate(mQuat);//通过滑动来改变相机的姿态矩
		return mat2 * osg::Matrixd::translate(mPos);

	}

	// 获取逆矩阵
	osg::Matrixd CameraManipulator::getInverseMatrix() const
	{
		osg::Matrixd mat;
		osg::Matrixd mat2;
		mat2 = mat * osg::Matrixd::rotate(mQuat);//通过滑动来改变相机的姿态矩
		return osg::Matrixd::inverse(mat2 * osg::Matrixd::translate(mPos));

	}

	// 事件处理函数
	bool CameraManipulator::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us)
	{
		osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *> (&us);
		osg::ref_ptr<osg::Group> root = viewer->getSceneData()->asGroup();
		int i = 0;
		osg::Node * node;// = dynamic_cast<osg::Node *>(root->getChild(i));
		//node->setNodeMask(mploader->objs[i]->visible);
//cout<<mploader->objs[i]->visible;
            //   cout<<mploader->objs[0]->visible;
		for(;i<size_objs;i++)
		{
			node = dynamic_cast<osg::Node *>(root->getChild(i));
			//cout<<mploader->objs[i]->visible;
node->setNodeMask(mploader->objs[i]->visible);
	node->setNodeMask(1);
	 if(mploader->objs[i]->visible==0){
	   mPos=osg::Vec3(100,100,100);
	}
		}

		for(;i<size_objs+size_pics;i++)
		{
			node = dynamic_cast<osg::Node *>(root->getChild(i));
			node->setNodeMask(mploader->pics[i-size_objs]->visible);
		}

		return true;
	}
