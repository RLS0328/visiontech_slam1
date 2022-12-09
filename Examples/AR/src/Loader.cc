#include "Loader.h"

Loader::Loader(std::string path)
	{
		YAML_PATH = path;
		cv::FileStorage* obj_yaml=load_file("obj.yaml");
		cv::FileNode* obj_files ;
		cv::FileStorage* file;
		string file_name;
		cv::FileNodeIterator it, it_end;

		obj_files = load_file_node(obj_yaml,"obj_files");
		float px,py,pz,ax,ay,az,s;
		it = obj_files->begin();
		it_end = obj_files->end();
		for (; it != it_end; ++it)
		{
			file = load_file((string)(*it));
			(*file)["osg_file"]>>file_name;
			(*file)["px"]>>px;
			(*file)["py"]>>py;
			(*file)["pz"]>>pz;
			(*file)["ax"]>>ax;
			(*file)["ay"]>>ay;
			(*file)["az"]>>az;
			(*file)["scale"]>>s;
			objs.push_back(new Obj(file_name,px,py,pz,ax,ay,az,s));
		}
		size_objs = objs.size();

		obj_files = load_file_node(obj_yaml,"pic_files");
		float x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4;
		it = obj_files->begin();
		it_end = obj_files->end();
		for (; it != it_end; ++it)
		{
			file = load_file((string)(*it));
			(*file)["pic_file"]>>file_name;
			(*file)["x1"]>>x1;	(*file)["y1"]>>y1;	(*file)["z1"]>>z1;
			(*file)["x2"]>>x2;	(*file)["y2"]>>y2;	(*file)["z2"]>>z2;
			(*file)["x3"]>>x3;	(*file)["y3"]>>y3;	(*file)["z3"]>>z3;
			(*file)["x4"]>>x4;	(*file)["y4"]>>y4;	(*file)["z4"]>>z4;
			pics.push_back(new Pic(file_name,x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4));
		}
		size_pics = pics.size();
	}

cv::FileStorage* Loader::load_file(const string& s)
	{
		cv::FileStorage* file = new cv::FileStorage();
		file->open(YAML_PATH+s, cv::FileStorage::READ);
		if(!file->isOpened())
		{
			cout << "load_file failed\t" << s << endl;
			exit(-1);
		}
		cout << "load_file succeed\t" << s << endl;
		return file;
	}

cv::FileNode* Loader::load_file_node(const cv::FileStorage* file,const string& s)
	{
		cv::FileNode* file_node = new cv::FileNode((*file)[s]);
		if (!file_node->isSeq())
		{
			cout << "load_file_node failed\t"<< s << endl;
			exit(-1);
		}
		cout << "load_file_node succeed\t"<< s << endl;
		return file_node;
	}

void Loader::add2group(osg::ref_ptr<osg::Group>& group)
	{
		for (int i =0; i < size_objs; i ++)
		{

			group->addChild(createLight(objs[i]->pat.get()));
		}
		for (int i =0; i < size_pics; i ++)
		{
			//group->addChild(pics[i]->trans.get());
		}

		//group->addChild(makeCoordinate());
		plane = makeSurface();
		group->addChild(plane);
	}
osg::ref_ptr<osg::Group> Loader::createLight(osg::ref_ptr<osg::Node>node)
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
