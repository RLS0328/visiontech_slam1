# 环境准备

## 1.安装编译软件cmake： sudo apt-get install cmake

## 2.安装pangolin库：依赖项
    sudo apt-get install libglew-dev
    sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
    sudo apt-get install libpython2.7-dev
    sudo apt-get install build-essential
    将Thirdparty中的pangolin放在合适位置解压，然后进入pangolin文件夹编译：
    mkdir build
    cd build
    cmake -DCPP11_NO_BOOST=1 ..
    make
    sudo make install

## 3.安装Eigen库：使用ubuntu软件源更新   
    sudo apt-get install libeigen3-dev

##4.安装依赖库BLAS and LAPACK库：
    sudo apt-get install libblas-dev
    sudo apt-get install liblapack-dev

## 5.安装boost库：sudo apt-get install libboost-dev

## 6.安装opencv：依赖项
    sudo apt-get install build-essential   (和上面重复了)
    sudo apt-get install libgtk2.0-dev pkg-config libavcodec-dev 
    sudo apt-get install libavformat-dev libswscale-dev
    sudo apt-get install libvtk5-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev libtbb-dev
    sudo apt-get install libv4l-dev
    将Thirdparty中的opencv放在合适位置解压，然后进入opencv文件夹编译：
    mkdir release
    cd release
    cmake -D CMAKE_BUILD_TYPE=RELEASE –D CMAKE_INSTALL_PREFTX=/usr/local ..
    make
    sudo make install 

## 7.安裝OSG库：依赖项
    sudo apt-get build-dep openscenegraph
    sudo apt-get install mesa-common-dev freeglut3 freeglut3-dev
    将Thirdparty中的OSG放在合适位置解压，然后进入OSG文件夹编译：
    ./configure （如果提示须 sudo make install_ld_conf， 则运行这个命令）
    make
    sudo make install
    ldconfig （如果提示没有权限，用sudo ldconfig）

    osg数据下载：www.osgchina.org/index.php?option=com_content&view=article&id=109&Itemid=482

    **环境变量配置**
    进入home文件夹按ctrl+h找到.bashrc文件 在文件的最后加上（文件路径地址根据你放的位置）

    export PATH=${PATH}:/wangchenyu/project/OpenSceneGraph/bin (osg安装路径bin)
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/wangchenyu/OpenSceneGraph/lib（osg安装路径）
    export OSG_FILE_PATH=/home/wangchenyu/OpenSceneGraph-Data:/home/wangchenyu/OpenSceneGraph-Data/Images （osg数据路径-3D模型位置）

    重启电脑，通过osgviewer cow.osg来验证配置是否正确。

# 二、运行程序

## 1.更改配置文件中的设定路径：CMakeLists.txt中set(OSG_INCLUDE OSG安装路径/include)
    set(OSG_LIB OSG安装路径/lib)
    Examples/AR/setting中的Conf.yaml  改最后的YAML_PATH
## 2.编译  ./build.sh
    运行建图模式  ./exe_map.sh
    运行加载地图模式  ./exe_loc.sh
