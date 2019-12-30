#include "opencv2/opencv.hpp"
#include "sys/time.h"
#include "time.h"

#ifndef SENTRYDEMO_IMAGE_TOOL_H
#define SENTRYDEMO_IMAGE_TOOL_H

class ImageTool{
public:
    static struct timeval start,end;
    static clock_t currentClock;
public:
    static void showRGBSplitImg(cv::Mat img);
    static void showHSVInRangeImg(cv::Mat img);
    static void drawRotatedRect(cv::Mat &img,cv::RotatedRect r);
    static void draw4Point4f(cv::Mat &img,cv::Point2f point2fs[4]);
    static float calc2PointDistance(cv::Point2f point1,cv::Point2f point2);
    static float calc2PointAngle(cv::Point2f point1,cv::Point2f point2);
    static float calcTriangleInnerAngle(cv::Point2f vertexPoint,cv::Point2f point1,cv::Point2f point2);
    static void timeInit();
    static void timeCout();
    static void timeInitClock();
    static void timeCoutClock();
    static double timeGet();


};


#endif //SENTRYDEMO_IMAGE_TOOL_H

