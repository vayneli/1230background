#ifndef AIM_PREDICT_H
#define AIM_PREDICT_H
#include "kalman_filter_by_opencv.h"
#include "opencv2/opencv.hpp"
#include <numeric>


class ErrAdder{
    public:
        ErrAdder();
        ~ErrAdder();
    public:
        void modelInit();
        void resetPostState(Mat statepost);
        Mat predict(cv::Mat measurement,float dt);
    private:
        Kalman_filter kf;
};


#endif