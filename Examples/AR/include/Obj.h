#ifndef OBJ_H
#define OBJ_H

#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <string>
using namespace std;
//USE_OSGPLUGIN(ive)

class Obj
{
public:
	osg::ref_ptr<osg::PositionAttitudeTransform> pat;

	bool visible;

	Obj(const string filename,const float px,const float py,const float pz,const float ax,const float ay,const float az,const float scale);
};

class Pic
{
public:
	bool visible;
	osg::ref_ptr<osg::Geode> geode;
	osg::ref_ptr<osg::PositionAttitudeTransform> trans;
	Pic(const string file,
		const float x1,const float y1,const float z1,
		const float x2,const float y2,const float z2,
		const float x3,const float y3,const float z3,
		const float x4,const float y4,const float z4);
};

osg::ref_ptr<osg::PositionAttitudeTransform> makeCoordinate();
osg::ref_ptr<osg::PositionAttitudeTransform> makeSurface();

#endif
