#ifndef RMDEMO_CONTROL_MODEL_H
#define RMDEMO_CONTROL_MODEL_H
#include "opencv2/opencv.hpp"
#include "robot_model.h"
#include "aim.h"
#include "pid.h"
#include "errorAdder.h"
#include "basic_tool.h"
#include"findballon.h"
class ControlModel{
public:
    ControlModel();
    ~ControlModel();

public:
    void init(RobotModel* pRobotModel);
    //串口监听数据处理函数接口
    void serialListenDataProcess(SerialPacket recvPacket);
    void radioListenDataProcess(RadioPacket recvPackert);
    void basketMonitor(bool is_ball_contain);
    void processFSM();

private:
    void trackBall();
    //机器人临时模式变量
    RobotMode mSetMode;
    RealsenseInterface* cap;
    SerialInterface* interface;
    RadioInterface* radio;
    Findballon findball;
    Point3f PID;
    VideoWriter writer_color,writer_depth;
    Aim_ball ball_aim;
    PIDControl pid_x;
    PIDControl pid_y;
    PIDControl pid_z;
    int close_count = 0;
    CAMERA_INRINSIC_PARAMETERS c;
    ErrAdder erroradder;
    Mat measurement = Mat::zeros(2, 1, CV_32F); 
    int count = 0;
    bool is_predict;
    float distance = 4;
    float ball_distance = 2.5;
    float drone_distance = 4;
    float target = 1.4;
private:
    RobotModel* pRobotModel;
    BasicTool basic_tool_;
    TRACKINGTARGET last_tracking_mode = TRACKING_NONE;
    bool need_reset_statepost = false;
    bool catching_step_open = false;
    bool using_last_speed = false;;
    int up_count = 0;
    bool is_up_now = false;
    float last_speed_x = 0;
    float last_speed_y = 0;
    float last_dis_x = 0;
    float last_dis_y = 0;
    int count_lost = 0;
    int pulse_count = 0;
    float last_drone_height = 0;
    Point3f last_drone_position;
    Point3f last_ball_position;
    //相关临时变量
private:
    int stable_count = 0;
    int start_delay_count;
    int start_tracing_time;
    int start_catch_delay;
    int start_flag = 0;
    bool ball_check_second_switch = false;
    int no_ball_count = 0;
    int start_time_in_waiting_check;
    int count_pulse = 0;
    bool second_time_tracking = false;
    bool add_speed_up = false;
    
};

#endif //RMDEMO_CONTROL_MODEL_H
