#include "mFrustum.h"

#include <opencv2/core/core.hpp> 
#include <cv.h>
#include <pangolin/pangolin.h>
#include <iostream>
using namespace std;
using namespace cv;

mFrustum::mFrustum(const float Obj[] , const float View[] , const float c , const float a , const float b)
{
	Vec3f pObj = Vec3f(Obj[0] , Obj[1] , Obj[2]);
	Vec3f pView = Vec3f(View[0] , View[1] ,View[2]);
	init(pObj,pView,c,a,b);
}

void mFrustum::init(const Vec3f& pObj , const Vec3f& pView , const float c , const float a , const float b)
{
	mpObj=pObj;
	//cout.precision(2);
	//cout<<"object position:"<<endl<<pObj<<endl;

	Vec3f nc = pView - pObj;
	//cout<<"nc:"<<endl<<nc<<endl;
	float lc = norm(nc);
	//cout<<"lc:"<<endl<<lc<<endl;
	Vec3f ec = normalize(nc);
	//cout<<"ec:"<<endl<<ec<<endl;

	Vec3f na = Vec3f(nc(2),0,-nc(0));
	float la = norm(na);
	Vec3f ea = normalize(na);

	Vec3f nb = Vec3f(nc(2),-nc(1),0);
	float lb = norm(nb);
	Vec3f eb = normalize(nb);

	if(la<0.001)
	{
		ea = eb.cross(ec);
		//cout<<"ea:"<<endl<<ea<<endl;
	}

	if(lb<0.001)
	{
		eb = ec.cross(ea);
		//cout<<"eb:"<<endl<<eb<<endl;
	}

	na=ea*a;
	nb=eb*b;

	float l = (lc - c)/lc;
	cpCol(0,nc + na + nb);
	cpCol(1,nc + na - nb);
	cpCol(2,nc - na + nb);
	cpCol(3,nc - na - nb);
	cpCol(4,l*(nc + na + nb));
	cpCol(5,l*(nc + na - nb));
	cpCol(6,l*(nc - na + nb));
	cpCol(7,l*(nc - na - nb));

	//cout<<"construct"<<endl<<vec<<endl;
}

void mFrustum::surf()
{
	//cout<<"surf"<<endl<<vec<<endl;
	const int pnts[6*4] = {
		0,1,2,3,// FRONT
		4,5,6,7,// BACK
		0,1,4,5,// LEFT
		2,3,6,7,// RIGHT
		0,2,4,6,// TOP
		1,3,5,7// BOTTOM
	};
	//cout.precision(2);
	GLfloat verts[6*4*3]={0.0f};
	for(int i=0;i<6*4;i++)
	{		
		verts[i*3+0]=vec(0,pnts[i])+mpObj(0);	//cout<<verts[i*3+0]<<"\t";
		verts[i*3+1]=vec(1,pnts[i])+mpObj(1);	//cout<<verts[i*3+1]<<"\t";
		verts[i*3+2]=vec(2,pnts[i])+mpObj(2);	//cout<<verts[i*3+2]<<endl;
	}
	glPushMatrix();
    
	glVertexPointer(3, GL_FLOAT, 0, verts);
	glEnableClientState(GL_VERTEX_ARRAY);
    
	glColor4f(1.0f, 0.0f, 0.0f, 0.1f);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
    
	glColor4f(0.0f, 1.0f, 0.0f, 0.1f);
	glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
	glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);
    
	glColor4f(0.0f, 0.0f, 1.0f, 0.1f);
	glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
	glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);
    
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

void mFrustum::cpCol(const int col,const Vec3f& v)
{
	vec(0,col)=v(0);
	vec(1,col)=v(1);
	vec(2,col)=v(2);
}


bool mFrustum::inFrustum(const Vec3f& n,const int a,const int b,const int c)
{
	Matx33f M;
	M(0,0)=vec(0,a);	M(0,1)=vec(0,b);	M(0,2)=vec(0,c);
	M(1,0)=vec(1,a);	M(1,1)=vec(1,b);	M(1,2)=vec(1,c);
	M(2,0)=vec(2,a);	M(2,1)=vec(2,b);	M(2,2)=vec(2,c);
	//cout<<"M0"<<M<<endl;
	double D = determinant(M);
	//cout<<"D"<<D<<endl;
	M(0,0)=n(0);	M(0,1)=vec(0,b);	M(0,2)=vec(0,c);
	M(1,0)=n(1);	M(1,1)=vec(1,b);	M(1,2)=vec(1,c);
	M(2,0)=n(2);	M(2,1)=vec(2,b);	M(2,2)=vec(2,c);
	//cout<<"M1"<<M<<endl;
	double d1 = determinant(M) / D;
	//cout<<"d1\t"<<d1<<endl;
	M(0,0)=vec(0,a);	M(0,1)=n(0);	M(0,2)=vec(0,c);
	M(1,0)=vec(1,a);	M(1,1)=n(1);	M(1,2)=vec(1,c);
	M(2,0)=vec(2,a);	M(2,1)=n(2);	M(2,2)=vec(2,c);
	//cout<<"M2"<<M<<endl;
	double d2 = determinant(M) / D;
	//cout<<"d2\t"<<d2<<endl;
	M(0,0)=vec(0,a);	M(0,1)=vec(0,b);	M(0,2)=n(0);
	M(1,0)=vec(1,a);	M(1,1)=vec(1,b);	M(1,2)=n(1);
	M(2,0)=vec(2,a);	M(2,1)=vec(2,b);	M(2,2)=n(2);
	//cout<<"M3"<<M<<endl;
	double d3 = determinant(M) / D;
	//cout<<"d3\t"<<d3<<endl;

#define ZERO 0.001

	return (D>ZERO||D<-ZERO) && d1>-ZERO && d2>-ZERO && d3>-ZERO && d1+d2+d3<1;
	/*Vec3f k;
	bool ok = solve(M,n,k);
	cout<<ok<<"\t"<<k<<endl;
	return ok && k(0)>0 && k(1)>0 && k(2)>0 && k(0)+k(1)+k(2)<1;*/
}

bool mFrustum::isVisible(const Vec3f& pCam)
{
	Vec3f n = pCam - mpObj;
	//cout<<pCam<<endl<<mpObj<<endl;
	float f0 = inFrustum(n,0,1,2);
	float f1 = inFrustum(n,1,2,3);
	float n0 = inFrustum(n,4,5,6);
	float n1 = inFrustum(n,5,6,7);
	//cout<<f0<<"\t"<<n0<<"\t"<<f1<<"\t"<<n1<<endl;
	return (f0&&!n0) || (f1&&!n1);
}

bool mFrustum::isVisible(const float Cam[])
{
	Vec3f pCam = Vec3f(Cam[0] , Cam[1] , Cam[2]);
	return isVisible(pCam);
}


