#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include "realsense.h"
#include <librealsense2/rs_advanced_mode.hpp>
RealsenseInterface::RealsenseInterface(){
    isOpen = false;

};
RealsenseInterface::~RealsenseInterface(){};
//rs2::align align_to(RS2_STREAM_DEPTH);
int RealsenseInterface::init(){
    rs2::context ctx;
    auto devices = ctx.query_devices();
    size_t device_count = devices.size();
    if (!device_count)
    {
        cout <<"No device detected. Is it plugged in?\n";
        return -1;
    }
    auto dev = devices[0];
    if (dev.is<rs400::advanced_mode>())
    {
        
        auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
        // Check if advanced-mode is enabled
        if (!advanced_mode_dev.is_enabled())
        {
            // Enable advanced-mode
            advanced_mode_dev.toggle_advanced_mode(true);
        }
    }
    else
    {
        cout << "Current device doesn't support advanced-mode!\n";
        return EXIT_FAILURE;
    }

    // rs2::device selected_device = dev;
    // auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    // depth_sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET,rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    //Create a configuration for configuring the pipeline with a non default profile
    // rs2::config cfg;
    // //cfg.enable_stream(RS2_STREAM_INFRARED,  color_img_width, color_img_height, RS2_FORMAT_Y8, 60);
    // //cfg.enable_stream(RS2_STREAM_DEPTH,     color_img_width, color_img_height, RS2_FORMAT_Z16, 60);

    // //Instruct pipeline to start streaming with the requested configuration
    // //cfg.enable_stream(RS2_STREAM_COLOR, color_img_width, color_img_height, RS2_FORMAT_BGR8, 60);   
    // auto profile = pipe.start(cfg);
    // auto sensor = profile.get_device().first<rs2::depth_sensor>();
    // sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET,rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    // pipe.stop();
    // cfg.enable_stream(RS2_STREAM_INFRARED,  color_img_width, color_img_height, RS2_FORMAT_Y8, 30);
    // //cfg.enable_stream(RS2_STREAM_INFRARED,  depth_img_width, depth_img_height, RS2_FORMAT_Y8, 60);
    // //cfg.enable_stream(RS2_STREAM_COLOR,  color_img_width, color_img_height, RS2_FORMAT_BGR8, 60);              
    // cfg.enable_stream(RS2_STREAM_COLOR,  color_img_width, color_img_height, RS2_FORMAT_RGB8, 30);
    // cfg.enable_stream(RS2_STREAM_DEPTH,  depth_img_width, depth_img_height, RS2_FORMAT_Z16, 30);   
    // pipe.start(cfg);

    
    // // spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    
    
    // // // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
    // // // but you can also increase the following parameter to decimate depth more (reducing quality)
    // // dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // // // Define transformations from and to Disparity domain
    //     for(int i = 0; i < 5; i++)
    // {
    //     //Wait for all configured streams to produce a frame
    //     auto data = pipe.wait_for_frames();
    // }

    return 0;
}

int RealsenseInterface::init(int width, int height){
    color_img_height = height;
    color_img_width = width;
    depth_img_height = height;
    depth_img_width = width;
    //depth_img_height = 720;
    //depth_img_width = 1280;
    int statue  = init();
    isOpen = true;
    if(statue == 0) start();
    return  statue; 
}

// int RealsenseInterface::readImg(){
//     try{
//         data = pipe.wait_for_frames();
//         rs2::frame color_infrared = data.first(RS2_STREAM_INFRARED);
//         rs2::frame color = data.get_color_frame();
//         rs2::frame depth = data.get_depth_frame();
//         const int w_c = color.as<rs2::video_frame>().get_width();
//         const int h_c = color.as<rs2::video_frame>().get_height();
//         const int w = depth.as<rs2::video_frame>().get_width();
//         const int h = depth.as<rs2::video_frame>().get_height();
//         //cout<<"color width:"<<w_c<<" "<<"color height:"<<h_c<<endl;
        
//         Mat color_tmp(Size(w_c, h_c),CV_8UC3,(void*)color.get_data(),Mat::AUTO_STEP);
//         //Mat color_tmp(Size(color_img_width, color_img_height),CV_8UC3,(void*)color.get_data(),Mat::AUTO_STEP);
//         Mat depth_tmp(Size(w, h),CV_16UC1,(void*)depth.get_data(),Mat::AUTO_STEP);
//         pthread_mutex_lock(&imgMutex);
//         //resize(color_tmp,color_tmp,Size(640,480));
//         color_tmp.copyTo(color_img);//写入color_img,加锁
//         depth_tmp.copyTo(depth_img);
//         pthread_mutex_unlock(&imgMutex);
//         isColorImgUpdate = true;
//         isDepthImgUpdate = true;
//         return EXIT_SUCCESS;
//     }
//     catch (const rs2::error & e)
//     {
//         std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//         return EXIT_FAILURE;
//     }
//     catch (const std::exception& e)
//     {
//         std::cerr << e.what() << std::endl;
//         return EXIT_FAILURE;
//     }
// }

//获取深度像素对应长度单位转换
float RealsenseInterface::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
 
//检查摄像头数据管道设置是否改变
bool RealsenseInterface::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}
 
int RealsenseInterface::readImg() {
    try
{
    //创建数据管道
    rs2::pipeline pipe;
    rs2::config pipe_config;
    //pipe.stop();
    //pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,60);
    //pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,60);
    //Calling pipeline's start() without any additional parameters will start the first device
    //直接start()，不添加配置参数，则默认打开第一个设备
    // with its default streams.
    //以及以默认的配置进行流输出
    //The start function returns the pipeline profile which the pipeline used to start the device
    //start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET,rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    //sensor.set_option(RS2_OPTION_VISUAL_PRESET,4);
    //profile = pipe.start(pipe_config);
    pipe.stop();

    pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,60);
    pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,60);
    profile = pipe.start(pipe_config);
    // Each depth camera might have different units for depth pixels, so we get it here
    //每个深度摄像头有不同单元的像素，我们这里获取
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    //使用数据管道的profile获取深度图像像素对应于长度单位（米）的转换比例
    float depth_scale = get_depth_scale(profile.get_device());
    cout<<"depth_scale:"<<depth_scale<<endl;
    //Pipeline could choose a device that does not have a color stream
    //数据管道可以选择一个没有彩色图像数据流的设备
    //If there is no color stream, choose to align depth to another stream
    //选择彩色图像数据流来作为对齐对象6
 
    rs2_stream align_to = RS2_STREAM_COLOR;//find_stream_to_align(profile.get_stream());
 
    /*
     @这里的对齐是改变深度图，而不改变color图
    */
    //The "align_to" is the stream type to which we plan to align depth frames.
    // "align_to"是我们打算用深度图像对齐的图像流
    rs2::align align(align_to);
 
    // Define a variable for controlling the distance to clip
    //定义一个变量去转换深度到距离
    float depth_clipping_distance = 1.f;
    //深度图像颜色map
    rs2::colorizer c;                          // Helper to colorize depth images
    //helper用于渲染图片
    //texture renderer;                     // Helper for renderig images
 
    // Create a pipeline to easily configure and start the camera
    
    //while (cvGetWindowHandle(depth_win)&&cvGetWindowHandle(color_win)) // Application still alive?
    //{
    while(exist){
        // Using the align object, we block the application until a frameset is available
        //堵塞程序直到新的一帧捕获
        rs2::frameset frameset = pipe.wait_for_frames();
 
        // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
        // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
        //因为rs2::align 正在对齐深度图像到其他图像流，我们要确保对齐的图像流不发生改变
        //  after the call to wait_for_frames();
        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            //如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
            profile = pipe.get_active_profile();
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }
 
        //Get processed aligned frame
        //获取对齐后的帧
        auto processed = align.process(frameset);
 
        // Trying to get both other and aligned depth frames
        //尝试获取对齐后的深度图像帧和其他帧
        rs2::frame aligned_color_frame = processed.get_color_frame();//processed.first(align_to);
        //rs2::frame aligned_depth_frame = processed.get_depth_frame().apply_filter(c);
        rs2::frame aligned_depth_frame = processed.get_depth_frame();
 
        //获取对齐之前的color图像
      //rs2::frame before_depth_frame=frameset.get_depth_frame().apply_filter(c);
        //获取宽高
        const int depth_w=aligned_depth_frame.as<rs2::video_frame>().get_width();
        const int depth_h=aligned_depth_frame.as<rs2::video_frame>().get_height();
        const int color_w=aligned_color_frame.as<rs2::video_frame>().get_width();
        const int color_h=aligned_color_frame.as<rs2::video_frame>().get_height();
        //const int b_color_w=before_depth_frame.as<rs2::video_frame>().get_width();
        //const int b_color_h=before_depth_frame.as<rs2::video_frame>().get_height();
        //If one of them is unavailable, continue iteration
        // if (!aligned_depth_frame || !aligned_color_frame)
        // {
        //     continue;
        // }
        //创建OPENCV类型 并传入数据
        Mat aligned_depth_image(Size(depth_w,depth_h),CV_16UC1,(void*)aligned_depth_frame.get_data(),Mat::AUTO_STEP);
        Mat aligned_color_image(Size(color_w,color_h),CV_8UC3,(void*)aligned_color_frame.get_data(),Mat::AUTO_STEP);
        //Mat before_color_image(Size(b_color_w,b_color_h),CV_8UC3,(void*)before_depth_frame.get_data(),Mat::AUTO_STEP);
        //显示
        // imshow(depth_win,aligned_depth_image);
        // imshow(color_win,aligned_color_image);
        // imshow("before aligned",before_color_image);
        // waitKey(10);
        pthread_mutex_lock(&imgMutex);
        aligned_color_image.copyTo(color_img);//写入color_img,加锁
        aligned_depth_image.copyTo(depth_img);
        pthread_mutex_unlock(&imgMutex);
        isColorImgUpdate = true;
        isDepthImgUpdate = true;
        
    }
    std::cout<<"?????????????"<<std::endl;
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
}

int RealsenseInterface::getColorImg(Mat &img){
    if(!isColorImgUpdate){
        //等待100ms
        int timeCounter=0;
        while(!isColorImgUpdate&&timeCounter<100){
            usleep(1000);//1ms等待
            timeCounter++;
        }
        if(!isColorImgUpdate){
            return -3;//更新超时
        }
    }
    pthread_mutex_lock(&imgMutex);
    color_img.copyTo(img);//读mImg,加锁
    pthread_mutex_unlock(&imgMutex);
    if(!img.empty()&&(img.cols==color_img_width)&&(img.rows==color_img_height)){
        isColorImgUpdate= false;
        return 0;
    } else{
        return -1;
    }

}

int RealsenseInterface::getDepthImg(Mat &img){
    if(!isDepthImgUpdate){
        //等待100ms
        int timeCounter=0;
        while(!isDepthImgUpdate&&timeCounter<100){
            usleep(1000);//1ms等待
            timeCounter++;
        }
        if(!isDepthImgUpdate){
            return -3;//更新超时
        }
    }
    pthread_mutex_lock(&imgMutex);
    depth_img.copyTo(img);//读mImg,加锁
    pthread_mutex_unlock(&imgMutex);
    if(!img.empty()&&(img.cols==depth_img_width)&&(img.rows==depth_img_height)){
        isDepthImgUpdate= false;
        return 0;
    } else{
        return -1;
    }

}

void RealsenseInterface::run(){
    //while(true){
        static bool flag=false;
        if(flag)
            return;
        flag=true;
        readImg();
    //}
}

 

 

