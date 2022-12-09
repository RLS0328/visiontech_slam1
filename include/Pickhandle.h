//#include <osgGA>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>
#include <osgViewer/Viewer>
//class Viewer;
class PickHandle : public osgGA::GUIEventHandler
{
public:
PickHandle();//const osgEarth::SpatialReference* srs);
~PickHandle();
    static bool PICK;
    double GetX(){return x;};
    double GetY(){return y;};
protected:
    virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
//private:
//    osg::Vec3d getPos(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa, osg::Vec3d& pos);
/*
signals: 
// 经纬度信息 
void signalPicked(osg::Vec3d pos); 
void signalMoving(osg::Vec3d pos); 
// 世界坐标信息 
void signalPickedXYZ(osg::Vec3d pos); 
void signalMovingXYZ(osg::Vec3d pos); 
void signalRightPicked();
*/
private:
    double x,y;
//osg::Vec3d m_vecPostion;
//const osg::EllipsoidModel* m_pEllipsoid;
};
