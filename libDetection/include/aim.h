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

typedef enum:unsigned char {
    RESULT_ONLY_BALL = 0xa0,
    RESULT_ONLY_UAV = 0xa1,
    RESULT_BOTH_BALL_UAV = 0xa2,
    RESULT_NONE = 0xa3
} TargetFinderResult;

typedef enum:unsigned char {
    TRACKING_BALL = 0xb0,
    TRACKING_UAV = 0xb1,
    TRACKING_IN_TESTING_UAV = 0xb2,
    TRACKING_NONE = 0xb3
} TRACKINGTARGET;

class Aim_ball{
    public:
        Aim_ball();
        ~Aim_ball();
    public:  
        Mat setImage(Mat depth);
        Mat segmentInRange(Mat depth,int min, int max);
        bool contoursFinder_distance(Mat src,Mat depth,vector<vector<Point> > &contours,Point3f &point,int &count);//使用最近距离优先挑选
        bool contoursFinder_area(Mat src,Mat depth,vector<vector<Point> > &contours,Point3f &point,int &count);//使用最大面积优先挑选
        bool ballChecker(vector<Point> contour,Mat mask, Point2f center, float radius);
        TargetFinderResult targetSaprate(Mat src,Mat depth);
        TRACKINGTARGET setTarget(Mat &src,Mat depth,Point3f &drone,Point3f &ball);
        void controlLogic(Point3f &point);
        bool findTarget(Mat &src, Mat &depth, Point3f &point);
        
	    Point3f getC_xyz(Point3f& point_p,CAMERA_INRINSIC_PARAMETERS& camera);
        float P2Pdistance(Point2d point1,Point2d point2);
    private:
        Mat src;
        Mat roi_image;
        TargetFinderResult target_contain = RESULT_NONE;
        vector<Point> drone_contours;
        vector<Point> ball_contours;
        Point3d drone = Point3d(-1,-1,-1);
        Point3d ball = Point3d(-1,-1,-1);
};
#endif
