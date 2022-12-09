#ifndef LOADER_H
#define LOADER_H

#include <opencv2/opencv.hpp>
#include <osg/Group>
#include <string>
#include <vector>
#include "Obj.h"
#include <osg/Texture2D>
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
//USE_OSGPLUGIN(ive);
using namespace std;
class Loader
{
public:

string YAML_PATH;

osg::ref_ptr<osg::LightSource> lightSource;

vector<Obj*> objs;

int size_objs;

vector<Pic*> pics;

int size_pics;

Loader(string path);

cv::FileStorage* load_file(const string& s);

cv::FileNode* load_file_node(const cv::FileStorage* file,const string& s);

void add2group(osg::ref_ptr<osg::Group>& group);

osg::ref_ptr<osg::Group> createLight(osg::ref_ptr<osg::Node>node);

osg::ref_ptr<osg::PositionAttitudeTransform> plane;
};

#endif
