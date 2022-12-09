#include"MapSaveLoad.h"
#include <iostream>
namespace ORB_SLAM2
{
    //************************MapSaveLoad*******************************//
    cv::Mat MapSaveLoad::ModelMatrix = cv::Mat::ones(4,4,CV_32F);
    MapSaveLoad::MapSaveLoad():MaxMPIdx(0),MaxKFIdx(0),mMap(NULL){};
    MapSaveLoad::MapSaveLoad(Map *map):MaxMPIdx(0),MaxKFIdx(0),mMap(map){};

    void MapSaveLoad::GetMapPointIdx(vector<MapPoint*> mpMapPoints){
        for(auto mp:mpMapPoints){
            mmpnMapPointsIdx[mp] = mp->mnId;
        }
    }
    void MapSaveLoad::GetIdxMapPoint(vector<MapPoint *> mpMapPoints) {
        for(auto mp:mpMapPoints){
            mmpnIdxMapPoints[mp->mnId] = mp;
        }
    }
    void MapSaveLoad::Save(const string &filename) {
        static ofstream save("./save.log");
        cerr << "start save map ..." << endl;
        ofstream f_save(filename, ios::binary | ios::out);

        if(!f_save){
            cerr<<"can't open :"<<filename<<endl;
            return;
        }

        //存地图点数量 和 地图点
	    vector<MapPoint*> mMapPoints = mMap->GetAllMapPoints();
        unsigned int mps = mMapPoints.size();
        cerr << "the number of mappoints : " << mps << endl;
        f_save.write((char *) &mps, sizeof(mps));
        for (auto mp:mMapPoints) {
            //MaxMPIdx = max(MaxMPIdx,mp->mnId);
            SaveMapPoints(f_save, mp);
        }
        //f_save.write((char*) &MaxMPIdx, sizeof(MaxMPIdx));
        //cerr << "save mappoints" << endl;

        GetMapPointIdx(mMapPoints);

        //存关键帧数量 和 关键帧
	vector<KeyFrame*> mKeyFrames = mMap->GetAllKeyFrames();
        unsigned int kfs = mKeyFrames.size();
        cerr << "the number of keyframes : " << kfs << endl;
        f_save.write((char *) &kfs, sizeof(kfs));
        for (auto kf:mKeyFrames) {
            //MaxKFIdx = max(MaxKFIdx,kf->mnId);
            SaveKeyFrames(f_save, kf);
        }
        //f_save.write((char*) &MaxKFIdx, sizeof(MaxKFIdx));
        //cerr << "save keyframes" << endl;

        //存父节点和covisibility图的边和节点
        for(auto kf:mKeyFrames){
            save<<"kf id"<<kf->mnId<<endl;
            KeyFrame* parent = kf->GetParent();
            unsigned int parentIdx = UINT_MAX;
            if(parent){
                parentIdx = parent->mnId;
                save<<"parent id"<<parentIdx<<endl;
            }
            f_save.write((char*)&parentIdx, sizeof(parentIdx));

            set<KeyFrame *> conkf = kf->GetConnectedKeyFrames();
            unsigned int conIdx = conkf.size();
            f_save.write((char*)&conIdx, sizeof(conIdx));
            save<<"connection size"<<conIdx<<endl;
            for(auto ckf:conkf){
                save<<"connection id"<<ckf->mnId<<"\t";
                int weight = kf->GetWeight(ckf);
                f_save.write((char*)&ckf->mnId, sizeof(ckf->mnId));
                f_save.write((char*)&weight, sizeof(weight));
            }
            save<<endl;
        }

        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                f_save.write((char*) &ModelMatrix.at<float>(i,j), sizeof(float));
            }
        }
        f_save.close();
        cerr<<"Map saving finished!"<<endl;
    }

    void MapSaveLoad::Load(const string& filename,SystemSetting* mSystemSetting,KeyFrameDatabase* mKeyFrameDatabase){
        static ofstream load("./load.log");
        cerr<<"Start load map ..."<<endl;
        ifstream f_load(filename,ios::binary | ios::in);

        if(!f_load){
            cerr<<"can't open :"<<filename<<endl;
            return;
        }

        //读取地图点数量 和 地图点
        unsigned int mps;
        f_load.read((char*)&mps, sizeof(mps));
        cerr << "the number of mappoints : " << mps << endl;
        for(unsigned int i=0;i<mps;i++){
            MapPoint* mp = LoadMapPoints(f_load);
            MaxMPIdx = max(MaxMPIdx,mp->mnId);
            mMap->AddMapPoint(mp);
        }
        //f_load.read((char*)&MaxMPIdx, sizeof(MaxMPIdx));
        if (MaxMPIdx > 0) {
            //MapPoint *temp_MapPoint = new MapPoint();  //使用MapPoint的构造函数来解决id更新的问题
            MapPoint::nNextId = MaxMPIdx+1;
            //delete temp_MapPoint;
            //temp_MapPoint = NULL;
        }

        vector<MapPoint*> vmp = mMap->GetAllMapPoints();
        GetIdxMapPoint(vmp);
        //读取关键帧数量 和 关键帧
        unsigned int kfs;
        f_load.read((char*)&kfs, sizeof(kfs));
        cerr << "the number of keyframes : " << kfs <<endl;
        vector<KeyFrame*> kf_order;
        for(unsigned int i=0; i<kfs; i++){
            KeyFrame* kf = LoadKeyFrames(f_load,mSystemSetting,mKeyFrameDatabase);
            MaxKFIdx = max(MaxKFIdx,kf->mnId);
            mMap->AddKeyFrame(kf);
            kf_order.push_back(kf);
            mKeyFrameDatabase->add(kf);
            //cerr<<"order"<<kf->mnId<<endl;
        }
        //f_load.read((char*)&MaxKFIdx, sizeof(MaxKFIdx));

        if(MaxKFIdx>0){
            //Frame temp_frame = Frame(mnMaxKFid);
            //kf_order.front()->updateID(mnMaxKFid);
            //KeyFrame* temp_KeyFrame = new KeyFrame();
            Frame::nNextId = MaxKFIdx + 1;
            KeyFrame::nNextId = MaxKFIdx +1;
            //delete temp_KeyFrame;
        }

        vector<KeyFrame*> vkf = mMap->GetAllKeyFrames();

        //读取父节点 和 邻居节点 及其 权重
        map<unsigned int, KeyFrame*> kf_id;
        for(auto kf:vkf){
            //cerr<<"!"<<kf->mnId<<endl;
            kf_id[kf->mnId] = kf;
        }

        for(auto kf:kf_order){
            load<<"kf id"<<kf->mnId<<endl;
            unsigned int parent_id;
            f_load.read((char*)&parent_id, sizeof(parent_id));

            if(parent_id!=UINT_MAX){
                load<<"parent id"<<parent_id<<endl;
                kf->ChangeParent(kf_id[parent_id]);
            }

            unsigned int con_ids;
            f_load.read((char*)&con_ids, sizeof(con_ids));
            load<<"connection id"<<con_ids<<endl;
            for(unsigned int i=0; i<con_ids; i++){
                unsigned int id;
                int weight;
                f_load.read((char*)&id, sizeof(id));
                f_load.read((char*)&weight, sizeof(weight));
                load<<"con id"<<id<<"\t";
                if(kf_id[id]!= NULL ) {
                    kf->AddConnection(kf_id[id], weight);
                }
                else{
                    cerr<<"the connection is null!!!"<<endl;
                }
            }
            cerr<<endl;
        }
        //cerr<<"!!!"<<endl;
        //重计算地图点数据
        for(auto mp:vmp){
            if(mp){
                mp->ComputeDistinctiveDescriptors();
                mp->UpdateNormalAndDepth();
            }
        }

        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                f_load.read((char*) &ModelMatrix.at<float>(i,j), sizeof(float));
            }
        }
        f_load.close();
        cerr<<"map loading finished!"<<endl;
        return;
    }

    void MapSaveLoad::SaveKeyFrames(ofstream &fkf, KeyFrame *kf) {
        //保存keyframe的ID，时间戳
        fkf.write((char *) &kf->mnId, sizeof(kf->mnId));
        fkf.write((char *) &kf->mTimeStamp, sizeof(kf->mTimeStamp));

        //保存关键帧的位姿矩阵
        cv::Mat mTcw = kf->GetPose();
        
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                fkf.write((char *) &mTcw.at<float>(i, j), sizeof(float));
            }
        }
         /*
        std::vector<float> Quat = Converter::toQuaternion(mTcw);
        for ( int i = 0; i < 4; i ++ )
            fkf.write((char*)&Quat[i],sizeof(float));
        //Save the translation matrix
        for ( int i = 0; i < 3; i ++ )
            fkf.write((char*)&mTcw.at<float>(i,3),sizeof(float));
*/
        //存关键帧的特征点数 和 特征点 和 特征点对应的地图点
        fkf.write((char *) &kf->N, sizeof(kf->N));
        for (int i = 0; i < kf->N; i++) {
            cv::KeyPoint kp = kf->mvKeys[i];
            fkf.write((char *) &kp.pt.x, sizeof(kp.pt.x));
            fkf.write((char *) &kp.pt.y, sizeof(kp.pt.y));
            fkf.write((char *) &kp.size, sizeof(kp.size));
            fkf.write((char *) &kp.angle, sizeof(kp.angle));
            fkf.write((char *) &kp.response, sizeof(kp.response));
            fkf.write((char *) &kp.octave, sizeof(kp.octave));


            fkf.write((char*)&kf->mDescriptors.cols, sizeof(kf->mDescriptors.cols));
            for (int j = 0; j < kf->mDescriptors.cols; j++) {
                fkf.write((char *) &kf->mDescriptors.at<unsigned char>(i, j), sizeof(char));
            }

            unsigned int mnIdx;
            MapPoint *mp = kf->GetMapPoint(i);
            if (mp == NULL) {
                mnIdx = UINT_MAX;
            } else {
                mnIdx = mmpnMapPointsIdx[mp];
            }
            fkf.write((char *) &mnIdx, sizeof(mnIdx));

        }
    }

    KeyFrame* MapSaveLoad::LoadKeyFrames(ifstream& fkf, SystemSetting* mSystemSetting, KeyFrameDatabase* mKeyFrameDatabase) {

        InitKeyFrame initkf(*mSystemSetting);

        //读取关键帧的ID和时间戳
        fkf.read((char*)&initkf.nId, sizeof(initkf.nId));
        fkf.read((char*)&initkf.TimeStamp, sizeof(initkf.TimeStamp));

        //读取关键帧的位姿矩阵
/*
        cv::Mat T = cv::Mat::zeros(4,4,CV_32F);
        std::vector<float> Quat(4);
        //Quat.reserve(4);
        for ( int i = 0; i < 4; i ++ )
            fkf.read((char*)&Quat[i],sizeof(float));
        cv::Mat R = Converter::toCvMat(Quat);
        for ( int i = 0; i < 3; i ++ )
            fkf.read((char*)&T.at<float>(i,3),sizeof(float));
        for ( int i = 0; i < 3; i ++ )
            for ( int j = 0; j < 3; j ++ )
                T.at<float>(i,j) = R.at<float>(i,j);
        T.at<float>(3,3) = 1;
        */
        cv::Mat T = cv::Mat::zeros(4,4,CV_32F);
        for(int i=0; i<4; i++){
            for(int j=0; j<4; j++){
                fkf.read((char*)&T.at<float>(i,j), sizeof(float));
            }
        }
         

        //读取关键帧特征点数目 和 特征点
        fkf.read((char*)&initkf.N, sizeof(initkf.N));
        initkf.vKps.reserve(initkf.N);
        initkf.Descriptors.create(initkf.N,32,CV_8UC1);
        vector<MapPoint*> vpMapPoints;
        vpMapPoints = vector<MapPoint*>(initkf.N,static_cast<MapPoint*>(NULL));

        vector<MapPoint*> vmp = mMap->GetAllMapPoints();
        for(int i=0; i<initkf.N; i++){
            //读取一个特征点
            cv::KeyPoint kp;
            fkf.read((char*)&kp.pt.x, sizeof(kp.pt.x));
            fkf.read((char*)&kp.pt.y, sizeof(kp.pt.y));
            fkf.read((char*)&kp.size, sizeof(kp.size));
            fkf.read((char*)&kp.angle, sizeof(kp.angle));
            fkf.read((char*)&kp.response, sizeof(kp.response));
            fkf.read((char*)&kp.octave, sizeof(kp.octave));

            initkf.vKps.push_back(kp);

            //读取该特征点的描述符
            fkf.read((char*)&initkf.Descriptors.cols, sizeof(initkf.Descriptors.cols));
            for(int j=0;j<initkf.Descriptors.cols;j++){
                fkf.read((char*)&initkf.Descriptors.at<unsigned char>(i,j), sizeof(char));
            }

            //找对应的地图点ID
            unsigned int mpid;
            fkf.read((char*)&mpid, sizeof(mpid));

            if(mpid!=UINT_MAX){
                vpMapPoints[i] = mmpnIdxMapPoints[mpid];
            }
            else{
                vpMapPoints[i] = NULL;
            }
        }

        initkf.vRight = vector<float>(initkf.N,-1);
        initkf.vDepth = vector<float>(initkf.N,-1);
        initkf.UndistortKeyPoints();
        initkf.AssignFeaturesToGrid();

        KeyFrame* kf = new KeyFrame(initkf, mMap, mKeyFrameDatabase, vpMapPoints);
        kf->mnId = initkf.nId;
        kf->SetPose(T);
        kf->ComputeBoW();

        for(int i=0; i<initkf.N; i++){
            if(vpMapPoints[i]){
                vpMapPoints[i]->AddObservation(kf,i);
                if(!vpMapPoints[i]->GetReferenceKeyFrame()){
                    vpMapPoints[i]->SetReferenceKeyFrame(kf);
                }
            }
        }
        return kf;
    }
    void MapSaveLoad::SaveMapPoints(ofstream &fmp,MapPoint* mp) {
        //保存mappoint的ID和世界坐标
        fmp.write((char *) &mp->mnId, sizeof(mp->mnId));
        cv::Mat mpWorldPos = mp->GetWorldPos();
        fmp.write((char *) &mpWorldPos.at<float>(0), sizeof(float));
        fmp.write((char *) &mpWorldPos.at<float>(1), sizeof(float));
        fmp.write((char *) &mpWorldPos.at<float>(2), sizeof(float));
    }

    MapPoint* MapSaveLoad::LoadMapPoints(ifstream& fmp) {
        unsigned int id;
        fmp.read((char *) &id, sizeof(id));
        cv::Mat Pos(3,1,CV_32F);
        fmp.read((char *) &Pos.at<float>(0), sizeof(float));
        fmp.read((char *) &Pos.at<float>(1), sizeof(float));
        fmp.read((char *) &Pos.at<float>(2), sizeof(float));
        MapPoint* mp = new MapPoint(Pos,mMap);
        mp->mnId = id;
        mp->SetWorldPos(Pos);
        return mp;
    }
    //************************InitKeyFrame*******************************//
    InitKeyFrame::InitKeyFrame(SystemSetting &SS):pVocabulary(SS.pVocabulary)//, pKeyFrameDatabase(SS.pKeyFrameDatabase)
    {
        fx = SS.fx;
        fy = SS.fy;
        cx = SS.cx;
        cy = SS.cy;
        invfx = SS.invfx;
        invfy = SS.invfy;
        bf = SS.bf;
        b  = SS.b;
        ThDepth = SS.ThDepth;

        nScaleLevels = SS.nLevels;
        fScaleFactor = SS.fScaleFactor;
        fLogScaleFactor = log(SS.fScaleFactor);
        vScaleFactors.resize(nScaleLevels);
        vLevelSigma2.resize(nScaleLevels);
        vScaleFactors[0] = 1.0f;
        vLevelSigma2[0]  = 1.0f;
        for ( int i = 1; i < nScaleLevels; i ++ )
        {
            vScaleFactors[i] = vScaleFactors[i-1]*fScaleFactor;
            vLevelSigma2[i]  = vScaleFactors[i]*vScaleFactors[i];
        }

        vInvScaleFactors.resize(nScaleLevels);
        vInvLevelSigma2.resize(nScaleLevels);
        for ( int i = 0; i < nScaleLevels; i ++ )
        {
            vInvScaleFactors[i] = 1.0f/vScaleFactors[i];
            vInvLevelSigma2[i]  = 1.0f/vLevelSigma2[i];
        }

        K = SS.K;

        DistCoef = SS.DistCoef;

        if( SS.DistCoef.at<float>(0)!=0.0)
        {
            cv::Mat mat(4,2,CV_32F);
            mat.at<float>(0,0) = 0.0;
            mat.at<float>(0,1) = 0.0;
            mat.at<float>(1,0) = SS.width;
            mat.at<float>(1,1) = 0.0;
            mat.at<float>(2,0) = 0.0;
            mat.at<float>(2,1) = SS.height;
            mat.at<float>(3,0) = SS.width;
            mat.at<float>(3,1) = SS.height;

            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, SS.K, SS.DistCoef, cv::Mat(), SS.K);
            mat = mat.reshape(1);

            nMinX = min(mat.at<float>(0,0), mat.at<float>(2,0));
            nMaxX = max(mat.at<float>(1,0), mat.at<float>(3,0));
            nMinY = min(mat.at<float>(0,1), mat.at<float>(1,1));
            nMaxY = max(mat.at<float>(2,1), mat.at<float>(3,1));
        }
        else
        {
            nMinX = 0.0f;
            nMaxX = SS.width;
            nMinY = 0.0f;
            nMaxY = SS.height;
        }

        fGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(nMaxX-nMinX);
        fGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(nMaxY-nMinY);

    }

    void InitKeyFrame::UndistortKeyPoints()
    {
        if( DistCoef.at<float>(0) == 0.0)
        {
            vKpsUn = vKps;
            return;
        }

        cv::Mat mat(N,2,CV_32F);
        for ( int i = 0; i < N; i ++ )
        {
            mat.at<float>(i,0) = vKps[i].pt.x;
            mat.at<float>(i,1) = vKps[i].pt.y;
        }

        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, K, DistCoef, cv::Mat(), K );
        mat = mat.reshape(1);

        vKpsUn.resize(N);
        for( int i = 0; i < N; i ++ )
        {
            cv::KeyPoint kp = vKps[i];
            kp.pt.x = mat.at<float>(i,0);
            kp.pt.y = mat.at<float>(i,1);
            vKpsUn[i] = kp;
        }
    }

    void InitKeyFrame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for ( unsigned int i = 0; i < FRAME_GRID_COLS; i ++ )
        {
            for ( unsigned int j = 0; j < FRAME_GRID_ROWS; j ++)
                vGrid[i][j].reserve(nReserve);
        }

        for ( int i = 0; i < N; i ++ )
        {
            const cv::KeyPoint& kp = vKpsUn[i];
            int nGridPosX, nGridPosY;
            if( PosInGrid(kp, nGridPosX, nGridPosY))
                vGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    bool InitKeyFrame::PosInGrid(const cv::KeyPoint &kp, int &posX,  int &posY)
    {
        posX = round((kp.pt.x-nMinX)*fGridElementWidthInv);
        posY = round((kp.pt.y-nMinY)*fGridElementHeightInv);

        if(posX<0 || posX>=FRAME_GRID_COLS ||posY<0 || posY>=FRAME_GRID_ROWS)
            return false;
        return true;
    }
    //************************SystemSetting*******************************//
    SystemSetting::SystemSetting(ORBVocabulary* pVoc):
            pVocabulary(pVoc)
    {
    }

    //SystemSetting::SystemSetting(ORBVocabulary* pVoc, KeyFrameDatabase* pKFDB):
    //    pVocabulary(pVoc), pKeyFrameDatabase(pKFDB)
    //    {
    //    }


    bool SystemSetting::LoadSystemSetting(const std::string strSettingPath){
        cout<<endl<<"Loading System Parameters form:"<<strSettingPath<<endl;
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        width  = fSettings["Camera.width"];
        height = fSettings["Camera.height"];
        fx     = fSettings["Camera.fx"];
        fy     = fSettings["Camera.fy"];
        cx     = fSettings["Camera.cx"];
        cy     = fSettings["Camera.cy"];

        cv::Mat tmpK = cv::Mat::eye(3,3,CV_32F);
        tmpK.at<float>(0,0) = fx;
        tmpK.at<float>(1,1) = fy;
        tmpK.at<float>(0,2) = cx;
        tmpK.at<float>(1,2) = cy;
        tmpK.copyTo(K);

        cv::Mat tmpDistCoef(4,1,CV_32F);
        /*
        tmpDistCoef.at<float>(0) = fSettings["Camera.k1"];
        tmpDistCoef.at<float>(1) = fSettings["Camera.k2"];
        tmpDistCoef.at<float>(2) = fSettings["Camera.p1"];
        tmpDistCoef.at<float>(3) = fSettings["Camera.p2"];
        */
        tmpDistCoef.at<float>(0) = 0;
        tmpDistCoef.at<float>(1) = 0;
        tmpDistCoef.at<float>(2) = 0;
        tmpDistCoef.at<float>(3) = 0;
        const float k3 = fSettings["Camera.k3"];
        if( k3!=0 )
        {
            tmpDistCoef.resize(5);
            tmpDistCoef.at<float>(4) = k3;
        }
        tmpDistCoef.copyTo( DistCoef );

        bf = fSettings["Camera.bf"];
        fps= fSettings["Camera.fps"];

        invfx = 1.0f/fx;
        invfy = 1.0f/fy;
        b     = bf  /fx;
        initialized = true;

        cout<<"- size:"<<width<<"x"<<height<<endl;
        cout<<"- fx:"  <<fx<<endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if(DistCoef.rows==5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- bf: " << bf << endl;

        //Load RGB parameter
        nRGB = fSettings["Camera.RGB"];

        //Load ORB feature parameters
        nFeatures = fSettings["ORBextractor.nFeatures"];
        fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        nLevels = fSettings["ORBextractor.nLevels"];
        fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        fMinThFAST = fSettings["ORBextractor.minThFAST"];

        cout << endl  << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        //Load others parameters, if the sensor is MONOCULAR, the parameters is zero;
        //ThDepth = fSettings["ThDepth"];
        //DepthMapFactor = fSettings["DepthMapFactor"];
        fSettings.release();
        return true;
    }
    //***********************************new function*********************************//
    KeyFrame::KeyFrame(InitKeyFrame &initkf, Map *pMap, KeyFrameDatabase *pKFDB, vector<MapPoint *> &vpMapPoints) :
            mnFrameId(0), mTimeStamp(initkf.TimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
            mfGridElementWidthInv(initkf.fGridElementWidthInv), mfGridElementHeightInv(initkf.fGridElementHeightInv),
            mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
            mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
            fx(initkf.fx), fy(initkf.fy), cx(initkf.cx), cy(initkf.cy), invfx(initkf.invfx),
            invfy(initkf.invfy), mbf(initkf.bf), mb(initkf.b), mThDepth(initkf.ThDepth), N(initkf.N),
            mvKeys(initkf.vKps), mvKeysUn(initkf.vKpsUn), mvuRight(initkf.vRight), mvDepth(initkf.vDepth),
            mDescriptors(initkf.Descriptors.clone()), mBowVec(initkf.BowVec), mFeatVec(initkf.FeatVec),
            mnScaleLevels(initkf.nScaleLevels), mfScaleFactor(initkf.fScaleFactor),
            mfLogScaleFactor(initkf.fLogScaleFactor),
            mvScaleFactors(initkf.vScaleFactors), mvLevelSigma2(initkf.vLevelSigma2),
            mvInvLevelSigma2(initkf.vInvLevelSigma2),
            mnMinX(initkf.nMinX), mnMinY(initkf.nMinY), mnMaxX(initkf.nMaxX), mnMaxY(initkf.nMaxY), mK(initkf.K),
            mvpMapPoints(vpMapPoints), mpKeyFrameDB(pKFDB), mpORBvocabulary(initkf.pVocabulary),
            mbFirstConnection(true), mpParent(NULL), mbNotErase(false), mbToBeErased(false), mbBad(false),
            mHalfBaseline(initkf.b / 2), mpMap(pMap) {
        //nNextId ++;
        mGrid.resize(mnGridCols);
        for (int i = 0; i < mnGridCols; i++) {
            mGrid[i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mGrid[i][j] = initkf.vGrid[i][j];
        }
    }

    void MapPoint::SetReferenceKeyFrame(KeyFrame *mKeyFrame) {
        unique_lock<mutex> lock(mMutexFeatures);
        mpRefKF = mKeyFrame;
    }

    MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap) :
            mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame *>(NULL)), mnVisible(1),
            mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap) {
        Pos.copyTo(mWorldPos);
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId = nNextId++;
        mbTrackInView = false;
    }


}
