#ifndef REALSENSE_INTERFACE_H
#define REALSENSE_INTERFACE_H
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include "base_thread.h"
using namespace std;
using namespace cv;

class RealsenseInterface:public BaseThread{
    public:
        RealsenseInterface();
        ~RealsenseInterface();

    private:
        //内部初始化方法
        int init();

        //读取彩色图片
        int readColorImg();

        //读取深度图片
        int readDepthImg();

        int readImg();
        void run();

    public:
        /** 相机初始化函数(不包括相机参数初始化)
         *  @param: int width,相机分辨率的宽度值。
         *  @param: int height,相机分辨率的高度值。
         *  @return: int ,错误码，0代表无错误。
         */
        int init(int width , int height);

        /** 彩色图片获取函数
         *  @param: Mat &img,　Mat类的引用，目标图像存放。
         *  @return: int ,错误码，0代表无错误。
         */
        int getColorImg(Mat &img);

        /** 深度图片获取函数
         *  @param: Mat &img,　Mat类的引用，目标图像存放。
         *  @return: int ,错误码，0代表无错误。
         */
        int getDepthImg(Mat &img);

        float get_depth_scale(rs2::device dev);

        bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

    private:
        rs2::spatial_filter spat;
        rs2::frameset data;
        rs2::frameset frameset;
        rs2::disparity_transform depth2disparity;
        rs2::decimation_filter dec;
       // rs2::pipeline pipe;
       // rs2::pipeline pipe_color;
       // rs2::pipeline profile;
        float depth_scale;
        rs2_stream align_to=RS2_STREAM_COLOR;
        bool isOpen;
        
        pthread_mutex_t imgMutex = PTHREAD_MUTEX_INITIALIZER;//互斥锁
        int color_img_width;
        int depth_img_width;
        int color_img_height;
        int depth_img_height;
        public:
        bool isColorImgUpdate;
        bool isDepthImgUpdate;
        Mat color_img;
        Mat depth_img;


};
#endif