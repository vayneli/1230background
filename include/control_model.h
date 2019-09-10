#ifndef RMDEMO_CONTROL_MODEL_H
#define RMDEMO_CONTROL_MODEL_H
#include "opencv2/opencv.hpp"
#include "robot_model.h"
#include "aim.h"
#include "pid.h"

class ControlModel{
public:
    ControlModel();
    ~ControlModel();

public:
    void init(RobotModel* pRobotModel);
    //串口监听数据处理函数接口
    void serialListenDataProcess(SerialPacket recvPacket);
    void processFSM();

private:
    void trackBall();
    //机器人临时模式变量
    RobotMode mSetMode;

    Aim_ball ball_aim;
    PIDControl pid_x;
    PIDControl pid_y;
    PIDControl pid_z;
private:
    RobotModel* pRobotModel;
    //相关临时变量
private:

};

#endif //RMDEMO_CONTROL_MODEL_H
