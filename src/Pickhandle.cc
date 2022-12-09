#include "Pickhandle.h"
//#include <osgUtil/>
#include <iostream>
#include "MapPoint.h"
using namespace std;
PickHandle::PickHandle():x(0),y(0){//const osgEarth::SpatialReference *srs) {
    //m_pEllipsoid = srs->getGeodeticSRS()->getEllipsoid();
}

PickHandle::~PickHandle() {}

bool PickHandle::PICK = false;
bool PickHandle::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    // 存储经纬度信息
    //osg::Vec3d vecPos;
    switch (ea.getEventType()) {
        // 点击事件
        case osgGA::GUIEventAdapter::PUSH: {

            //cerr<<"x,y p"<<x<<"\t"<<y<<endl;
            //osg::Vec3d pos = getPos(ea, aa, vecPos);
            // 鼠标左键
            if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
                x = ea.getX();
                y = ea.getY();
                //ORB_SLAM2::Viewer::detectplane = true;
                PICK = true;
            }
            break;
        }
        // 鼠标移动事件
        case osgGA::GUIEventAdapter::MOVE: {
            //osg::Vec3d pos = getPos(ea, aa, vecPos);
            //emit signalMoving(vecPos);
            //emit signalMovingXYZ(pos);
            break;
        }
        // 鼠标释放事件
        case osgGA::GUIEventAdapter::RELEASE: {
            /*
            osg::Vec3d pos = getPos(ea, aa, vecPos);
            // 鼠标左键
            if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
                // 如果释放的点和点击的点同一，则发送单击事件发生的位置
                if (m_vecPostion == pos && m_vecPostion != osg::Vec3d(0, 0, 0)) {
                    emit signalPicked(vecPos);
                    emit signalPickedXYZ(pos);
                }
            } else if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
                emit signalRightPicked();
            }
             */
            //x = ea.getX();
            //y = ea.getY();
            //cerr<<"x,y r"<<x<<"\t"<<y<<endl;
            break;
        }
    }
    return false;
}

/*
osg::Vec3d PickHandle::getPos(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa, osg::Vec3d &pos) {
    pos = osg::Vec3d(0, 0, 0);
    osgViewer::Viewer *pViewer = dynamic_cast<osgViewer::Viewer *>(&aa);
    if (pViewer == NULL) {
        return osg::Vec3d(0, 0, 0);
    }
    // 获取当前点
    osgUtil::LineSegmentIntersector::Intersections intersection;
    double x = ea.getX();
    double y = ea.getY();
    pViewer->computeIntersections(ea.getX(), ea.getY(), intersection);
    osgUtil::LineSegmentIntersector::Intersections::iterator iter = intersection.begin();
    if (iter != intersection.end()) {
        //m_pEllipsoid->convertXYZToLatLongHeight(iter->getWorldIntersectPoint().x(), iter->getWorldIntersectPoint().y(),
        //                                        iter->getWorldIntersectPoint().z(), pos.y(), pos.x(), pos.z());
        pos.x() = osg::RadiansToDegrees(pos.x());
        pos.y() = osg::RadiansToDegrees(pos.y());
        return iter->getWorldIntersectPoint();
    }
    return osg::Vec3d(0, 0, 0);
}
 */
