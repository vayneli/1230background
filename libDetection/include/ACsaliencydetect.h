#ifndef _ACSALIENCYDETECT_H
#define _ACSALIENCYDETECT_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

//相机内参结构
struct CAMERA_INRINSIC_PARAMETERS_A{
    double cx,cy,fx,fy,scale;
};

class AC_saliency{
    public:
        AC_saliency();
        ~AC_saliency();
    public:
        Mat saliencyBasedonAC(Mat &src,int MinR2, int MaxR2,int Scale);
        Point3f getXY(Mat& img,Mat& depth);
        Point3f getC_xyz(Point3f& point_p,CAMERA_INRINSIC_PARAMETERS_A& camera);
        
};






#endif
