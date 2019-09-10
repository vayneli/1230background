////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC MBZIRC     Model Code for UAV
///ALL RIGHTS RESERVED
///@file:control_model.cpp
///@brief: UAV 控制模型，包含对所有应用的管理，创建应用，并改变应用
/// 
///@vesion 1.0
///@author: PC
///@email: 694977655@qq.com
///@date: 18-11-6
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "control_model.h"
#include "basic_tool.h"
#include "usb_capture_with_thread.h"
#include "string"
#include "aim.h"
#include "ACsaliencydetect.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace cv;
using namespace std;

ControlModel::ControlModel(){}

ControlModel::~ControlModel(){}
//static VideoWriter writer;
static VideoWriter writer_color,writer_depth;
void ControlModel::init(RobotModel* robotModel){
    pRobotModel=robotModel;
    //配置文件
    usleep(1000000);//等待1s，等摄像头稳定
    //初始模式初始化
    mSetMode=ROBOT_MODE_TRACKBALL;
    //writer.open("log.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),0);
    writer_color.open("log_color.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),1);
    writer_depth.open("log_depth.avi",CV_FOURCC('M','P','4','2'),25,Size(640,480),0);
}

//串口数据接收处理入口
void ControlModel::serialListenDataProcess(SerialPacket recvPacket) {
//复杂自定义数据包，需要自定义析单独处理
    pRobotModel->DataUpdate(recvPacket);
}

void ControlModel::processFSM(){
    //模式切换预处理
    if(mSetMode!=pRobotModel->getCurrentMode()){
        pRobotModel->setCurrentMode(mSetMode);
        switch (mSetMode){
            case ROBOT_MODE_TRACKBALL:{
                pid_x.init(0.01,0,0.0001,0.015f,-3,3,AUTOMATIC,DIRECT);
                pid_y.init(0.00,0.000,0,0.015f,-3,3,AUTOMATIC,DIRECT);
                pid_z.init(0.005,0,0,0.015f,-3,3,AUTOMATIC,DIRECT);
                pid_x.PIDSetpointSet(0);
                pid_y.PIDSetpointSet(0);
                pid_z.PIDSetpointSet(500);
                cout<<"[control model mode ]:Switch to BALL TRACKING Mode!"<<endl;
                break;
            }
            case ROBOT_MODE_RETURN:{
                cout<<"[control model mode ]:Switch to RETURN Mode!"<<endl;
                break;
            }
            case ROBOT_MODE_EMPTY:{
                cout<<"[control model mode ]:Do Nothing"<<endl;
                break;
            }
        }
    }

    switch(pRobotModel->getCurrentMode()){
        case ROBOT_MODE_TRACKBALL:{
            trackBall();
            break;
        }
        case ROBOT_MODE_RETURN:{

            break;
        }
        
    }

}

void ControlModel::trackBall(){
    RealsenseInterface* cap = pRobotModel->getRealsenseCpature();
    SerialInterface* interface = pRobotModel->getpSerialInterface();
    Mat src;
    Mat ori;
    //相机内参
    CAMERA_INRINSIC_PARAMETERS camera;
    //后面补充；
    AC_saliency ac_saliency;
    Eigen::Matrix3f Rotation_matrix;
    Eigen::Vector3f pcv,pav;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    Mat result;
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    Rect temp;
    cap->getColorImg(ori);
    Point3f pe,pc,pa;
    String str1,str2; 
    float velocity_y,velocity_x,velocity_z;
    //VideoWriter writer;
    //writer.open("log.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),0);
    //distance.x = distance .y = distance.z = 0xff;
    time_t c=time(NULL);
    while(std::difftime(time(NULL),c)<60){
    if(cap->getDepthImg(src) == 0){
        //distance = ball_aim.getDistance(src);
      //if(ball_aim.isdetect(ori,distance)==1){
          Mat result=ac_saliency.saliencyBasedonAC(ori,ori.rows/8,ori.rows/2,3);
            threshold(result, result, 40, 255, THRESH_BINARY);
            // //cout<<result.rows<<endl;
            morphologyEx(result,result,MORPH_CLOSE,element);
            findContours(result, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for(size_t i=0;i<contours.size();i++){
                if(contours[i].size()>5){
                    temp = boundingRect(contours[i]);
                    if(temp.area()>2000) continue;
                    if(temp.area()<200) continue;
                    rectangle(ori,temp,Scalar(0,0,255));
                    pe.x=int(temp.y+temp.height/2);
                    pe.y=int(temp.x+temp.width/2);
                    pe.z=src.at<ushort>(pe.x,pe.y);
                    cout<<"where pe:"<<pe<<endl;
                    pc=ac_saliency.getC_xyz(pe,camera);

                    Rotation_matrix=pRobotModel->getRotation_matrix(pc);
                    pcv={pc.x,pc.y,pc.z};
                    pav=(Rotation_matrix.inverse())*pcv;

                    pa={pav[0],pav[1],pav[2]};

                }
            }
        // std::cout<<"distance.x:"<<distance.y<<"\tdistance.z"<<distance.z<<endl;
        // circle(ori,Point(distance.x+320,distance.y+240),5,Scalar(0,0,255));
        pid_x.PIDInputSet(pa.x);
        pid_y.PIDInputSet(pa.y);
        pid_z.PIDInputSet(pa.z);
        if(pid_x.PIDCompute()==false)std::cout<<"error pid_x"<<std::endl;//fix
        if(pid_y.PIDCompute()==false)std::cout<<"error pid_y"<<std::endl;
        if(pid_z.PIDCompute()==false)std::cout<<"error pid_z"<<std::endl;
        velocity_y = pid_x.PIDOutputGet();
        velocity_x = pid_y.PIDOutputGet();
        //velocity_z = pid_z.PIDOutputGet();
        //}else{
            velocity_x=velocity_y=velocity_z=0;
        //}
        //float velocity_z = pid_z.PIDOutputGet();
	    std::cout<<"pid_y:"<<velocity_y<<std::endl;
        // if(abs(distance.x)<50&&abs(distance.y)<50)
        str1 = "velocity_x: "+to_string(velocity_x);
        str2="distancez:"+to_string(pa.z);
        putText(ori,str1,Point(10,30),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),4,8);
        putText(ori,str2,Point(10,60),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),4,8);
        
        writer_color.write(ori);
        //writer_color.release();
	    src.convertTo(src,CV_8UC1);
        writer_depth.write(src);
        //writer_depth.release();
        //if (char(waitKey(1)) == 'q') break;

             interface->movebyVelocity(0,0,0,0);
        // else interface->movebyVelocity(velocity_x,velocity_y,0,0);
        // if(distance.z<800)  cout<<"收网!!!!"<<endl;
    }
        else cout<<"error"<<endl;
        if (char(waitKey(1)) == 'q') break;
    }   
        cout<<"one minutes"<<endl;
        writer_color.release();
        writer_depth.release();
} 
