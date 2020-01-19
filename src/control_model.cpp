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
    
    c.fx=380;
    c.fy=380;
    c.cx=320;
    c.cy=239;
    c.scale=1000;
    erroradder.modelInit();
    is_predict = false;
}

//串口数据接收处理入口
void ControlModel::serialListenDataProcess(SerialPacket recvPacket) {
//复杂自定义数据包，需要自定义析单独处理
    pRobotModel->DataUpdate(recvPacket);
}

void ControlModel::radioListenDataProcess(RadioPacket recvPacket) {
//复杂自定义数据包，需要自定义析单独处理
    unsigned char CMD= recvPacket.getCMD();
    if(CMD==CMD_SWITCH_MESSAGE_UPDATE){
        unsigned char switch_1 = recvPacket.getUncharInBuffer(3);
        unsigned char existance = recvPacket.getUncharInBuffer(14);
        if(switch_1==0x01&&existance==0x00){
            mSetMode = ROBOT_MODE_WAITING;
	    cout<<"catching!!!!!!!!!!!"<<endl;
        }
        if(existance == 0x01){
            mSetMode = ROBOT_MODE_RETURN;
        }

        
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
                cout<<"[control model mode ]:Switch to BALL TRACKING Mode!"<<endl;
                is_up_now = false;
                need_reset_statepost = false;
                catching_step_open = false;
                count = 0;
                up_count = 0;
                start_flag = 0;
                cout<<"[control model mode ]:BALL TRACKING Mode IS OK!"<<endl;
		        
                break;
            }
            case ROBOT_MODE_AUTO_TAKEOFF:{
		        cout<<"auto take off"<<endl;
		        interface->Takeoff();
		        cout<<"!!!!"<<endl;
                interface->moveByPositionOffset_block(0,0,5,0,0.5,0);
                cout<<"end take off"<<endl;
		break;
            }
            case ROBOT_MODE_RETURN:{
                cout<<"[control model mode ]:Switch to RETURN Mode!"<<endl;
                Point3f pos = interface->getTransformedPosition(pRobotModel->getOriginPoint(),interface->getGPSposition());
                interface->moveByPositionOffset_block(-pos.x,-pos.y,0,0,1,0);
                interface->Land();
                break;
            }
            case ROBOT_MODE_EMPTY:{
                cout<<"[control model mode ]:Do Nothing"<<endl;
                break;
            }

            case ROBOT_MODE_WAITING:{
                cout<<"[control model mode ]: waiting the ball checking"<<endl;
                start_time_in_waiting_check = basic_tool_.currentTimeMsGet();
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
            mSetMode = ROBOT_MODE_TRACKBALL;

            break;
        }
        case ROBOT_MODE_WAITING:{
            Point3f pos = interface->getTransformedPosition(pRobotModel->getOriginPoint(),interface->getGPSposition());
            int end_time = basic_tool_.currentTimeMsGet();
	    if(end_time-start_time_in_waiting_check >= 1500){
	        interface->moveByPositionOffset(0,0,5.5-pos.z,0);
	    }
            //interface->moveByPositionOffset(0,0,5.5-pos.z,0);
            if(end_time-start_time_in_waiting_check >= 3000){
                mSetMode = ROBOT_MODE_TRACKBALL;
                second_time_tracking = true;
            }
            break;
        }
    }

}

void ControlModel::trackBall(){
    int start = basic_tool_.currentTimeMsGet();
    Mat depth;
    Mat color;
    Point3f ball = Point3f(-1,-1,-1);//获取图像
    Point3f drone = Point3f(-1,-1,-1);
    cap->getDepthImg(depth);
    cap->getColorImg(color);
    
    #ifdef IMAGE_OUTPUT
        imshow("color",color);
        waitKey(1);
    #endif
    float velocity_x=0;
    float velocity_y=0;
    float velocity_z=0;
    Point3f point,point_c,ball_point,drone_point;
    Mat mask(Size(640,480),CV_8UC1);
    mask=ball_aim.setImage(depth);//初始化图像
    //imshow("mask",mask);
    TargetFinderResult result;
    result = ball_aim.targetSaprate(mask,depth);//寻找目标，并进行目标分割
    TRACKINGTARGET target_aim;
    Point3f pos = interface->getTransformedPosition(pRobotModel->getOriginPoint(),interface->getGPSposition());
    if(result!=RESULT_NONE){
        count_pulse ++;
        target_aim = ball_aim.setTarget(color,depth,drone,ball);//设定跟踪目标
	   // cout<<"ddd: "<<drone<<endl;
	    cout<<"bbb: "<<ball<<endl;
       if(drone.z == 0){
           drone.z = last_drone_height;
       }else{
      	    last_drone_height = drone.z;
       } 
        if(stable_count>=100) is_up_now = true;
        else is_up_now = false;                 //稳定跟踪200帧后开始抓球
       // is_up_now = false;
        if(!catching_step_open) {
            drone_distance = 4;
            ball_distance = 2.5;
        }

        if(last_tracking_mode == target_aim) pulse_count=0;
        //防止目标区分检测出现跳变
        if(last_tracking_mode == TRACKING_BALL && target_aim==TRACKING_UAV){
            if(fabs(last_ball_position.z-drone.z)<=500 && pulse_count<=5){
                target_aim = TRACKING_BALL;
                ball = drone;
            }
            pulse_count++;
        }

        if(last_tracking_mode == TRACKING_UAV && target_aim==TRACKING_BALL){
            if(fabs(last_ball_position.z-drone.z)<=500 && pulse_count<=5){
                target_aim = TRACKING_UAV;
                drone = ball;
            }
            pulse_count++;
            
        }
        
       cout<<"pos: "<<pos<<endl;
        switch(target_aim){
            case TRACKING_UAV:{
                if((last_tracking_mode == TRACKING_NONE && count_lost>=5)){
                    //need_reset_statepost = true;
                }else{
		            need_reset_statepost = false;
		        }
		        cout<<"UAV"<<endl;
                Point3f temp;
                point = drone;
                if(point.z == 0){
		            point.z = ball.z;
		            if(point.z==-1);
		    	        point.z = 0;
		        }
                drone_distance = 4.0;
                ball_distance = 2.5;
                break;
            }
            case TRACKING_BALL:{
	            cout<<"BALL"<<endl;
                if(last_tracking_mode != TRACKING_BALL){
                    //need_reset_statepost = true;
                }else{
			        need_reset_statepost = false;
		        }
                Point3f temp;
                temp=ball_aim.getC_xyz(ball,c);
                if(fabs(temp.x)<0.2&&fabs(temp.y)<0.2&is_up_now){
                    if(ball.z<=1000){
                        ball_distance -= 0.2;
			if(ball.z<700){
			 add_speed_up = true;
			}
			
                         
                    }else{
                        ball_distance -= 0.04;
                    }
                    if(ball_distance < 0.1 ){
                        ball_distance = 0;
                    }
                }
		if((fabs(temp.x)>0.5||fabs(temp.y)>0.5)&&is_up_now&&ball.z<=1000){
		ball_distance+=0.04;
		if(ball_distance>2.5) ball_distance = 2.5;
		}
                drone = ball;
                drone.z = ball.z+1600;
                point = ball;
                break;
            }
            case TRACKING_IN_TESTING_UAV:{
		        cout<<"UAV BALL"<<endl;				 
                if((last_tracking_mode == TRACKING_NONE&&count_lost>=5) || last_tracking_mode == TRACKING_BALL){
                    //need_reset_statepost = true;
                }else{
			        need_reset_statepost = false;
		        }
                catching_step_open = true;
                Point3f temp;
                temp=ball_aim.getC_xyz(ball,c);
                if(fabs(temp.x)<0.2&&fabs(temp.y)<0.2&&is_up_now){
                    if(ball.z<=1000){
			    
                        ball_distance-=0.2;
			if(ball.z<=700){
			add_speed_up = true;
			}
			
                    }else{
                        ball_distance -= 0.04;
                    }
                    if(ball_distance<0.1){
                        ball_distance = 0.1; 
                    }
                }
		if((fabs(temp.x)>0.5||fabs(temp.y)>0.5)&&is_up_now&&ball.z<1000){
		    ball_distance+=0.04;
		    if(ball_distance>2.5) ball_distance = 2.5;
		}
		if(ball.z<=1500){
		    point = ball;
		}else{
                    point = drone;
		}

                break;
            }
        }

	    count_lost = 0;
	    //cout<<"distance: "<<distance<<endl;
        if(distance<=1.70) up_count++;
        if(up_count >200) {
            distance = 4.0;
            up_count = 0 ;
            ball_check_second_switch = true;
        }else{
            ball_check_second_switch = false;
        }

        int end = basic_tool_.currentTimeMsGet();
        
        #ifdef SHOW_IMG
        Mat showImg;	
	    cvtColor(mask,showImg,CV_GRAY2BGR);
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
	    //cout<<"input point: "<<point<<endl;
        point_c=ball_aim.getC_xyz(point,c);
        ball_point = ball_aim.getC_xyz(ball,c);
        drone_point = ball_aim.getC_xyz(drone,c);
        cout<<"point_c:   "<<point_c<<endl;
        count++;
        
        //延时3秒开启运动补偿，防止飞机初次锁定目标时由于补偿震荡
        if(count==1) start_tracing_time = basic_tool_.currentTimeMsGet();
        start_delay_count = basic_tool_.currentTimeMsGet();
        if(start_delay_count-start_tracing_time>=3000) {
            is_predict = true;
            start_flag ++;
        }

        //记录开始进行稳定跟踪时刻
        if(start_flag >=1 ){
            Point3f temp = ball_aim.getC_xyz(drone,c);
            if(fabs(temp.x)<1&&fabs(temp.y)<1){
                stable_count++;
            }else{
                //stable_count--;
            }
            
        }

        if(is_predict){
            measurement.at<float>(0) = point_c.y+pos.x;
            measurement.at<float>(1) = point_c.x+pos.y;
            if(need_reset_statepost||start_flag==1){
                Mat state_post;
                state_post=(Mat_<float>(4, 1) << point_c.y+pos.x,point_c.x+pos.y,0,0);
                erroradder.modelInit();
                erroradder.resetPostState(state_post);
                need_reset_statepost = false;
                using_last_speed = true;
	        }
	    
            Mat predict = erroradder.predict(measurement,(end-start)*0.001);
	        cout<<"x: "<<predict.at<float>(0)<<" y: "<<predict.at<float>(1)<<" vx: "<<predict.at<float>(2)<<" vy: "<<predict.at<float>(3)<<endl;
            Point2d predict_point;
            if(using_last_speed){
                predict_point.x = last_speed_x*18*(end-start)*0.001;
                predict_point.y = last_speed_y*18*(end-start)*0.001;
                using_last_speed = false;
            }else{
                predict_point.x = predict.at<float>(2)*18*(end-start)*0.001;
                predict_point.y = predict.at<float>(3)*18*(end-start)*0.001;
		if(ball.z<1000){
		    predict_point.x = predict.at<float>(2)*20*(end-start)*0.001;
		    predict_point.y = predict.at<float>(3)*20*(end-start)*0.001;
		}
            }
            last_speed_x = predict.at<float>(2);
            last_speed_y = predict.at<float>(3);
	        cout<<"point_c: "<<point_c<<endl;
	        float x = point_c.y+predict_point.x;
	        float y = point_c.x+predict_point.y;
            cout<<"after:  x: "<<x<<" y: "<<y<<endl;
	        if(x>5) x=5;
	        if(x<-5)x=-5;
	        if(y>5)y=5;
	        if(y<-5)y=-5;

            if(target_aim==TRACKING_UAV){    
                interface->moveByPositionOffset(x,y,pos.z + point_c.z-4.5,0);
		cout<<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<endl;
            }else{
                if(!add_speed_up){
                    interface->moveByPositionOffset(x,y,pos.z + ball.z/1000-ball_distance,0);
                }else{
	                interface->moveByPositionOffset(x,y,pos.z+4*ball.z/1000+0.7,0);
		            add_speed_up=false;
                }
            }
        }else{
            if(point_c.y>3)point_c.y=3;
	        if(point_c.y<-3) point_c.y=-3;
	        if(point_c.x>3) point_c.x=3;
	        if(point_c.x<-3) point_c.x=-3;	
            if(point_c.y!=0&&point_c.x!=0&&count_pulse>=2)
                interface->moveByPositionOffset(point_c.y,point_c.x,pos.z + point_c.z-4.5,0);
        }
        last_drone_position = drone;
        last_ball_position = ball;
        if(target_aim == TRACKING_UAV){
            no_ball_count++;
            if(no_ball_count>100&&point_c.z>=3.5){
               // mSetMode = ROBOT_MODE_RETURN;
            }
        }else{
            no_ball_count=0;
        }
        

    }else{
        target_aim = TRACKING_NONE;
        count = 0;
        start_flag = 0;
        count_pulse=0;
	//interface->moveByPositionOffset(0,0,5.5-pos.z,0);
    }
    last_tracking_mode = target_aim;
    //imshow("src",color);
    //waitKey(1);
} 
