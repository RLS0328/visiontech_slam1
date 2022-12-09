#include "Obj.h"

#include <osg/Geometry>
#include <osg/Node>
#include <osg/ShapeDrawable>
#include <osg/PolygonOffset>

#include <osgDB/ReadFile>

#include <osgViewer/ViewerEventHandlers>

#include <iostream>
#include <osgAnimation/BasicAnimationManager>

	Obj::Obj(const string filename,const float px,const float py,const float pz,const float ax,const float ay,const float az,const float scale)
	{
		visible = true;
		osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(filename);	//读入节点文件
		if(node == NULL){
			cout<<"no model"<<endl;
		}
		else
		{
			cout<<"have model"<<endl;
		}
		pat = new osg::PositionAttitudeTransform();	//位姿变换
		pat->setPosition(osg::Vec3(px,py,pz));	//位置
		pat->setScale(osg::Vec3(scale,scale,scale));	//缩放
		//pat->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::ON);
		osg::Vec3 x(1,0,0),y(0,1,0),z(0,0,1);
		pat->setAttitude(osg::Quat(ax,x)*osg::Quat(ay,y)*osg::Quat(az,z)*osg::Quat(1.57,x));	//角度
		pat->addChild(node.get());	//节点加到位姿中
		pat->setDataVariance(osg::Object::DYNAMIC);
		//sw = new osg::Switch();	//开关
		//sw->addChild(pat.get());	//位姿加到开关中
		cout<<"Create "<<filename<<" at ["<<px<<","<<py<<","<<pz<<"]"<<endl;
	}

  Pic::Pic(const string file,
		const float x1,const float y1,const float z1,
		const float x2,const float y2,const float z2,
		const float x3,const float y3,const float z3,
		const float x4,const float y4,const float z4)
	{
		visible = true;
		trans = new osg::PositionAttitudeTransform();
		trans->addChild(geode);
		geode = new osg::Geode();
		osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
		geode->addDrawable(geometry);
		//光照模式关闭，这样从各个方向看到的图片才是一样的
		geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
		//指定几何体的顶点坐标
		osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
		v->push_back(osg::Vec3(x1, y1, z1));
		v->push_back(osg::Vec3(x2, y2, z2));
		v->push_back(osg::Vec3(x3, y3, z3));
		v->push_back(osg::Vec3(x4, y4, z4));
		geometry->setVertexArray(v.get());

		//指定几何体的法向量坐标
		osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array;
		normal->push_back(osg::Y_AXIS);
		geometry->setNormalArray(normal.get());
		geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);

		//指定几何体的纹理坐标
		osg::ref_ptr<osg::Vec2Array> tcoords = new osg::Vec2Array();
		tcoords->push_back(osg::Vec2(0.0f,0.0f));
		tcoords->push_back(osg::Vec2(1.0f,0.0f));
		tcoords->push_back(osg::Vec2(1.0f,1.0f));
		tcoords->push_back(osg::Vec2(0.0f,1.0f));
		geometry->setTexCoordArray(0, tcoords.get());
		//使用图元绘制几何体
		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
		osg::ref_ptr<osg::Texture2D> texture=new osg::Texture2D;
		osg::ref_ptr<osg::Image> image=osgDB::readImageFile(file);
		texture->setImage(image);
		geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);
		cout<<"Create "<<file<<endl;
	}

    osg::ref_ptr<osg::PositionAttitudeTransform> makeCoordinate()
    {
        //创建保存几何信息的对象
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

        //创建四个顶点
        osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array();
        v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
        v->push_back(osg::Vec3(1000.0f, 0.0f, 0.0f));
        v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
        v->push_back(osg::Vec3(0.0f, 1000.0f, 0.0f));
        v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
        v->push_back(osg::Vec3(0.0f, 0.0f, 1000.0f));
        geom->setVertexArray(v.get());

        //为每个顶点指定一种颜色
        osg::ref_ptr<osg::Vec4Array> c = new osg::Vec4Array();
        c->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f)); //坐标原点为红色
        c->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f)); //x red
        c->push_back(osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f)); //坐标原点为绿色
        c->push_back(osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f)); //y green
        c->push_back(osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f)); //坐标原点为蓝色
        c->push_back(osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f)); //z blue
        //如果没指定颜色则会变为黑色
        geom->setColorArray(c.get());
        geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

        //三个轴
        geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2)); //X
        geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES, 2, 2)); //Y
        geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES, 4, 2)); //Z

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        geode->addDrawable(geom.get());

	osg::TessellationHints *hints=new osg::TessellationHints();
	hints->setDetailRatio(0.5);
	//geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.1,0.1,-0.1),0.19),hints));

	osg::ref_ptr<osg::Node> node_xyz = geode.release();
        osg::ref_ptr<osg::PositionAttitudeTransform> pat_xyz =new osg::PositionAttitudeTransform();
        pat_xyz->setPosition(osg::Vec3(0.0f,0.0f,0.0f));
        pat_xyz->setScale(osg::Vec3(1.0f,1.0f,1.0f));
        pat_xyz->setAttitude(osg::Quat(0,osg::Vec3(0.0, 1.0, 0.0)));
        pat_xyz->addChild(node_xyz.get());

        return pat_xyz;
    }

osg::ref_ptr<osg::PositionAttitudeTransform> makeSurface()
{
    osg::ref_ptr<osg::PositionAttitudeTransform> transformGrid = new osg::PositionAttitudeTransform();
    osg::Geode *geode = new osg::Geode();
    osg::Geometry *geom = new osg::Geometry();
    //构造顶点信息
		        osg::Vec3Array *vertices = new osg::Vec3Array();
		        int i;
		        int num = 6;
		        float rad = 0.5;
		        vertices->push_back( osg::Vec3(0.0f,-rad,0.0f) );
		        vertices->push_back( osg::Vec3(0.0f, rad,0.0f) );
		        vertices->push_back( osg::Vec3(-rad,0.0f,0.0f) );
		        vertices->push_back( osg::Vec3( rad,0.0f,0.0f) );
		        for (i = 1; i<=num; i++)
		        {
		            float t = i*rad/num;
		            vertices->push_back( osg::Vec3(t,-rad,0.0f) );
		            vertices->push_back( osg::Vec3(t,rad,0.0f) );
		            vertices->push_back( osg::Vec3(-t,-rad,0.0f) );
		            vertices->push_back( osg::Vec3(-t,rad,0.0f) );
		            vertices->push_back( osg::Vec3(-rad,t,0.0f) );
		            vertices->push_back( osg::Vec3(rad,t,0.0f) );
		            vertices->push_back( osg::Vec3(-rad,-t,0.0f) );
		            vertices->push_back( osg::Vec3(rad,-t,0.0f) );
		        }
		        geom->setVertexArray(vertices);
		        //指定颜色数组
		        osg::Vec4Array* colors = new osg::Vec4Array();
		        colors->push_back( osg::Vec4(0.5f,0.541f,0.539f,1.0f) );
		        colors->push_back( osg::Vec4(0.5f,0.541f,0.539f,1.0f) );
		        geom->setColorArray(colors);
		        geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

		        //指定法线
		        osg::Vec3Array* normals = new osg::Vec3Array;
		        normals->push_back( osg::Vec3(0.0f,-1.0f,0.0f) );
		        geom->setNormalArray(normals);
		        geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

		        //将图元添加至geometry
		        geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES,0,4) );
		        geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::LINES,4,vertices->size() - 4) );
		        geode->addDrawable(geom);
		        osg::StateSet *set = new osg::StateSet();
		        set->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
		        set->setAttributeAndModes(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
		        set->setMode(GL_LINE_SMOOTH,osg::StateAttribute::ON);
		        geode->setStateSet(set);
		        transformGrid->addChild(geode);
		        transformGrid->setName("transformGrid");
		        return transformGrid;
}
