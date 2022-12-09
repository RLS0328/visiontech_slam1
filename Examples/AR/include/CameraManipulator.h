#ifndef CAMERAMANIPULATOR_H
#define CAMERAMANIPULATOR_H

#include <osgGA/CameraManipulator>
#include <osgViewer/Viewer>
#include "Loader.h"
#include <mutex>
class CameraManipulator:public osgGA::CameraManipulator
{
private:
	osg::Vec3 mPos;
	osg::Quat mQuat;
public:
	std::mutex mutexPos,mutexQuat;
	Loader* mploader;
	int size_objs,size_pics;

	CameraManipulator();
	~CameraManipulator(void){;}

	void setLoader(Loader* loader);

	void setPos(float x,float y,float z);

	void setQuat(float x,float y,float z,float w);

	// 把漫游器添加到场景中
	static CameraManipulator * CameraToScence(osg::ref_ptr<osgViewer::Viewer>viewer);

private:

	osg::ref_ptr<osgViewer::Viewer>m_pHostViewer;

	// 移动速度
	float m_fMoveSpeed;

public:

	// 鼠标左键状态
	bool m_bLeftButtonDown;

	// 鼠标位置
	float m_fpushX;
	float m_fpushY;

	// 设置矩阵
	void setByMatrix(const osg::Matrix &matrix){;}

	// 设置逆矩阵
	void setByInverseMatrix(const osg::Matrix &matrix){;}

	// 获取矩阵
	osg::Matrixd getMatrix() const;

	// 获取逆矩阵
	osg::Matrixd getInverseMatrix() const;

	// 事件处理函数
	bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &us);

};

#endif
