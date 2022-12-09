#include <opencv2/core/core.hpp>  
#include <cv.h>

using namespace cv;

class mFrustum
{
public:
	void init(const Vec3f& pObj , const Vec3f& pView , const float c , const float a , const float b);

	mFrustum(const float Obj[] , const float View[] , const float c , const float a , const float b);

	void surf();

	void cpCol(const int col,const Vec3f& v);

	bool isVisible(const Vec3f& pCam);

	bool isVisible(const float Cam[]);

	bool inFrustum(const Vec3f& n,const int a,const int b,const int c);

public:

	Vec3f mpObj;

	Matx<float, 3, 8> vec;

};









	

