#ifndef _AIM_H
#define _AIM_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "pnp_solver.h"
#include <time.h>
using namespace cv;
struct CAMERA_INRINSIC_PARAMETERS{
    double cx,cy,fx,fy,scale;
};
class Aim_ball{
    public:
        Aim_ball();
        ~Aim_ball();
    public:  
        Mat setImage(Mat depth);
        bool findTarget(Mat &src, Mat &depth, Point3f &point);
	Point3f getC_xyz(Point3f& point_p,CAMERA_INRINSIC_PARAMETERS& camera);
    private:
        Mat src;
        Mat roi_image;

};
#endif
