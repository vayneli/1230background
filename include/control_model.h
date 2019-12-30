#ifndef RMDEMO_CONTROL_MODEL_H
#define RMDEMO_CONTROL_MODEL_H
#include "opencv2/opencv.hpp"
#include "robot_model.h"
#include "aim.h"
#include "pid.h"
#include "errorAdder.h"
#include "basic_tool.h"
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
private:
    RobotModel* pRobotModel;
    BasicTool basic_tool_;
    //相关临时变量
private:

};

#endif //RMDEMO_CONTROL_MODEL_H
