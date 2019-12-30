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
#include <fstream>
#include <string>
using namespace cv;
using namespace std;
//#define COLLECT_DATA

//#define IMAGE_OUTPUT
//#define SHOW_IMG
ControlModel::ControlModel(){}

ControlModel::~ControlModel(){}
static VideoWriter writer_depth;
static VideoWriter writer_color;
static  string log_name = "log.txt";
//writer_color.open("color.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),1);
//writer_depth.open("depth.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),0);
void ControlModel::init(RobotModel* robotModel){
    pRobotModel=robotModel;
    cap = pRobotModel->getRealsenseCpature();
    interface = pRobotModel->getpSerialInterface();
    radio = pRobotModel->getpRadioInterface();
    pRobotModel->setOriginPoint();
    //配置文件
    usleep(5000000);//等待1s，等摄像头稳定


    //初始模式初始化
    mSetMode=ROBOT_MODE_AUTO_TAKEOFF;
    #ifdef COLLECT_DATA
        writer_color.open("color.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),1);
        writer_depth.open("depth.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),1);
    #endif
    
    c.fx=617;
    c.fy=618;
    c.cx=321;
    c.cy=235;
    c.scale=1000;
    erroradder.modelInit();
    is_predict = true;
}

//串口数据接收处理入口
void ControlModel::serialListenDataProcess(SerialPacket recvPacket) {
//复杂自定义数据包，需要自定义析单独处理
    pRobotModel->DataUpdate(recvPacket);
}

void ControlModel::radioListenDataProcess(RadioPacket recvPacket) {
//复杂自定义数据包，需要自定义析单独处理
    unsigned char CMD= recvPacket.getCMD();
    if(CMD==CMD_PID_UPDATE){
        pRobotModel->PIDUpdate(recvPacket.getFloatInBuffer(2),recvPacket.getFloatInBuffer(6),recvPacket.getFloatInBuffer(10));
        cout<<"get"<<endl;
	
    }
    else if(CMD==CMD_PID_SAVE){
        cv::FileStorage f("../res/parameters.yaml", cv::FileStorage::WRITE);
        f << "PID_P" << PID.x;
        f << "PID_I" << PID.y;
        f << "PID_D" << PID.z;
        f.release();
	cout<<"save success"<<endl;
    }
    else if(CMD==CMD_REGET_FLIGHT_CONTROL){
        interface->init(pRobotModel->getLinuxSetup());
    }

}

void ControlModel::basketMonitor(bool is_ball_contain){
    pRobotModel->ballStateUpdate(is_ball_contain);
}

void ControlModel::processFSM(){
    //模式切换预处理
    if(mSetMode!=pRobotModel->getCurrentMode()){
        pRobotModel->setCurrentMode(mSetMode);
        switch (mSetMode){
            case ROBOT_MODE_TRACKBALL:{
		PID = pRobotModel->getPIDParam();
                pid_x.init(1.5,0,0.0001,0.015f,-5,5,AUTOMATIC,DIRECT);
                pid_y.init(1.5,0.000,0,0.015f,-5,5,AUTOMATIC,DIRECT);
                pid_z.init(0.005,0,0,0.015f,-3,3,AUTOMATIC,DIRECT);
                pid_x.PIDSetpointSet(0);
                pid_y.PIDSetpointSet(0);
                pid_z.PIDSetpointSet(1500);
                cout<<"[control model mode ]:Switch to BALL TRACKING Mode!"<<endl;
		cout<<"PID: "<<PID<<endl;
                break;
            }
            case ROBOT_MODE_AUTO_TAKEOFF:{
		cout<<"auto take off"<<endl;
		interface->Takeoff();
		cout<<"!!!!"<<endl;
            interface->moveByPositionOffset_block(0,0,5,0,0,0);

                cout<<"end take off"<<endl;
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
        case ROBOT_MODE_AUTO_TAKEOFF:{
    //        interface->Takeoff();
  //          interface->moveByPositionOffset(0,0,3,0);
//            usleep(5000000);
            mSetMode = ROBOT_MODE_TRACKBALL;

            break;
        }
        
    }

}

void ControlModel::trackBall(){
    int start = basic_tool_.currentTimeMsGet();
    Mat depth;
    Mat color;

    float kp,ki,kd;
    cap->getDepthImg(depth);
    cap->getColorImg(color);
    //int k = basic_tool_.currentTimeMsGet();
    //cout<<"ti: "<<k-start<<endl;
   #ifdef IMAGE_OUTPUT
        imshow("color",color);
        waitKey(1);
    #endif
    float velocity_x=0;
    float velocity_y=0;
    float velocity_z=0;
    Point3f point,point_c;

    Mat mask(Size(640,480),CV_8UC1);
    mask=ball_aim.setImage(depth);
   // int k = basic_tool_.currentTimeMsGet();
    //cout<<"ti: "<<k-start<<endl;
    if(ball_aim.findTarget(mask,depth,point)){
        int end = basic_tool_.currentTimeMsGet();
	    if(point.x<=100&&point.y<=100)
                close_count++;
        else close_count = 0;
        if(close_count>=150) pid_z.PIDSetpointSet(500);
        #ifdef SHOW_IMG
        Mat showImg;	
	    cvtColor(mask,showImg,CV_GRAY2BGR);
        circle(showImg,Point2i(point.x,point.y),10,Scalar(0,0,255),4,CV_AA);
	    imshow("showImg",showImg);
	    waitKey(1);
        #endif
        #ifdef COLLECT_DATA
        Mat depth_write;
        cvtColor(mask,depth_write,CV_GRAY2BGR);
        circle(depth_write,Point2i(point.x,point.y),10,Scalar(0,0,255),4,CV_AA);
        writer_color.write(color);
        writer_depth.write(depth_write);
        #endif
        PID = pRobotModel->getPIDParam();
        pid_x.PIDTuningsSet(PID.x,PID.y,PID.z);
        pid_y.PIDTuningsSet(PID.x,PID.y,PID.z);
	    cout << "----------pid_x: " << PID.x << endl;
        point_c=ball_aim.getC_xyz(point,c);
        cout<<"point_c:"<<point_c<<endl;
	Point3f vel=interface->getVelocity();
        Point3f pos = interface->getTransformedPosition(pRobotModel->getOriginPoint(),interface->getGPSposition());
        if(abs(point_c.x)>3&&point_c.x>0) point_c.x=3;
        if(abs(point_c.x)>3&&point_c.x<0) point_c.x=-3;
        if(abs(point_c.y)>3&&point_c.y>0) point_c.y=3;
        if(abs(point_c.y)>3&&point_c.y<0) point_c.y=-3;
//        if(abs(point_c.z)>3&&point_c.z>0) point_c.z=3;
//        if(abs(point_c.z)>3&&point_c.z<0) point_c.z=-3;
	if(is_predict){
            count++;
	    cout<<"point: "<<point_c<<endl;

            measurement.at<float>(0) = pos.x+point_c.y;
            measurement.at<float>(1) = pos.y+point_c.x;
            //measurement.at<float>(2) = vel.x;
            //measurement.at<float>(3) = vel.y;
	    //cout<<measurement<endl;
            if(count==1){
                Mat state_post=(Mat_<float>(4, 1) << pos.x+point_c.y,pos.y+point_c.x,0,0);
                erroradder.modelInit();
                erroradder.resetPostState(state_post);

            
	    }
	    
            Mat predict = erroradder.predict(measurement,(end-start)*0.001);
	    cout<<"x: "<<predict.at<float>(0)<<" y: "<<predict.at<float>(1)<<" vx: "<<predict.at<float>(2)<<" vy: "<<predict.at<float>(3)<<endl;
            Point2d predict_point;
            predict_point.x = predict.at<float>(2)*23*(end-start)*0.001;
            predict_point.y = predict.at<float>(3)*23*(end-start)*0.001;
	  //  if(predict.at<float>(2)*4*(end-start)<=0.00002 || predict.at<float>(2)*4*(end-start)>3){
	//	    cout<<"!!!!!!!!!!!!!!"<<endl;
	  //  	predict_point.x = point_c.y;
	   // }
	   // if(predict.at<float>(3)*4*(end-start)<=0.000002 || predict.at<float>(3)*4*(end-start)>3){
             //   predict_point.y = point_c.x;
	//	cout<<"@@@@@@@@@"<<endl;
          //  }
	    cout<<"pre: "<<predict_point<<endl;
            interface->moveByPositionOffset(point_c.y+predict_point.x,point_c.x+predict_point.y,pos.z + point_c.z-4,0);
            cout<<"time_delay: "<<end-start<<endl;

        }else{
            //interface->moveByPositionOffset(point_c.y,point_c.x,pos.z + point_c.z-6,0);
            interface->moveByPositionOffset(point_c.y,point_c.x,pos.z+point_c.z-4,0);
        }
       cout<<"x:"<<point_c.x<<" y:"<<point_c.y<<" z:"<<point_c.z<<endl;

	    cout<<"speed_x: "<<vel.x<< "	" << "speed_y: "<<vel.y << endl;
        cout << "pos_x: " << pos.x << " pos_y: " << pos.y << " pos_z: " << pos.z << endl;
	    //string logs = to_string(pos.x) + " " + to_string(pos.y) + " " 
              //      + to_string(pos.z) + " " + to_string(point_c.y) + " " 
                //    + to_string(point_c.x) + " " + to_string(velocity_x) + " " 
                 //   +  to_string(velocity_y) + " " + to_string(vel.x) + " " +  to_string(vel.y) + "\n";

        //ofstream log(log_name, ios::app);
	    //log.open( log_name);
	  //  log << logs; 
	   // log.close();
    }else{
    count = 0;
    }

} 
