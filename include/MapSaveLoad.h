#ifndef MAPSAVELOAD
#define MAPSAVELOAD

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <string>
#include <vector>
//using namespace std;


namespace ORB_SLAM2 {

	class Map;
	class KeyFrame;
	class MapPoint;
	class KeyFrameDatabase;

    //******************************SystemSetting*************************************//
    class SystemSetting{

        //Load camera parameters from setting file
    public:

        SystemSetting(ORBVocabulary* pVoc);
        //SystemSetting::SystemSetting(ORBVocabulary* pVoc, KeyFrameDatabase* pKFDB );

        bool LoadSystemSetting(const std::string strSettingPath);

    public:
        //The Vocabulary and KeyFrameDatabase
        ORBVocabulary* pVocabulary;
        //KeyFrameDatabase* pKeyFrameDatabase;


        //Camera parameters
        float width;
        float height;
        float fx;
        float fy;
        float cx;
        float cy;
        float invfx;
        float invfy;
        float bf;
        float b;
        float fps;
        cv::Mat K;
        cv::Mat DistCoef;
        bool initialized;

        //Camera RGB parameters
        int nRGB;

        //ORB feature parameters
        int nFeatures;
        float fScaleFactor;
        int nLevels;
        float fIniThFAST;
        float fMinThFAST;

        //other parameters
        float ThDepth = -1;
        float DepthMapFactor = -1;

    };
    //*****************************InitKeyFrame******************************//
    class InitKeyFrame
    {
    public:
        InitKeyFrame(SystemSetting &SS);

        void UndistortKeyPoints();
        bool PosInGrid(const cv::KeyPoint& kp, int &posX, int &posY);
        void AssignFeaturesToGrid();

    public:

        ORBVocabulary* pVocabulary;
        //KeyFrameDatabase* pKeyFrameDatabase;

        unsigned int nId;
        double TimeStamp;

        float fGridElementWidthInv;
        float fGridElementHeightInv;
        std::vector<std::size_t> vGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        float fx;
        float fy;
        float cx;
        float cy;
        float invfx;
        float invfy;
        float bf;
        float b;
        float ThDepth;
        int N;
        std::vector<cv::KeyPoint> vKps;
        std::vector<cv::KeyPoint> vKpsUn;
        cv::Mat Descriptors;

        //it's zero for mono
        std::vector<float> vRight;
        std::vector<float> vDepth;

        DBoW2::BowVector BowVec;
        DBoW2::FeatureVector FeatVec;

        int nScaleLevels;
        float fScaleFactor;
        float fLogScaleFactor;
        std::vector<float> vScaleFactors;
        std::vector<float> vLevelSigma2;
        std::vector<float> vInvLevelSigma2;
        std::vector<float> vInvScaleFactors;

        int nMinX;
        int nMinY;
        int nMaxX;
        int nMaxY;
        cv::Mat K;
        cv::Mat DistCoef;
    };
    //************************************MapSaveLoad*****************************************//
	class MapSaveLoad {
	public:
		MapSaveLoad();
        MapSaveLoad(Map* map);

		void GetMapPointIdx(vector<MapPoint *> mpMapPoints);
		void GetIdxMapPoint(vector<MapPoint *> mpMapPoints);

		void Save(const string &filename);

		void
		Load(const string &filename, SystemSetting *mSystemSetting, KeyFrameDatabase *mKeyFrameDatabase);

		void SaveMapPoints(ofstream &fmp, MapPoint *mp);

		void SaveKeyFrames(ofstream &fkf, KeyFrame *kf);

		MapPoint *LoadMapPoints(ifstream &fmp);

		KeyFrame *LoadKeyFrames(ifstream &fkf, SystemSetting *mSystemSetting, KeyFrameDatabase* mKeyFrameDatabase);

        static cv::Mat ModelMatrix;
	private:
		std::map<MapPoint *, unsigned int> mmpnMapPointsIdx;
		std::map<unsigned int, MapPoint* > mmpnIdxMapPoints;
        unsigned int MaxMPIdx;
        unsigned int MaxKFIdx;
        Map* mMap;
	};


}
#endif