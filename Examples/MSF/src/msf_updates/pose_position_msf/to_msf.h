#include "my_types.h"
#include <opencv2/core/eigen.hpp>



class to_msf{
private:
    uint32_t  m_cnt=0;
    cv::Mat to_Tcw;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Meas::Ptr_mea> to_measurements;
    BlockQueue_t<cv::Mat>* to_mpTcw_buffer;
    Eigen::Vector3d grav;



    to_msf(const std::string vicon_path);
    ~to_msf() {;}



    void to_setGravity(const Eigen::Vector3d &gravity);
    cv::Mat to_getTcw_msf();
    void to_MSF_data();
    void to_MSF_init();
    void to_MSF_process();
};




