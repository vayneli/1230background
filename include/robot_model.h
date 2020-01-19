#ifndef RMDEMO_ROBOT_MODEL_H
#define RMDEMO_ROBOT_MODEL_H

#include "serial_packet.h"
#include "serial_interface.h"
#include "radio_interface.h"
#include "radio_packet.h"
#include "usb_capture_with_thread.h"
#include "basic_tool.h"
// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>
#include "realsense.h"
#include <Eigen/Core>
#include <string>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

typedef enum :unsigned char {
    ROBOT_MODE_EMPTY = 0x00,
    ROBOT_MODE_TRACKBALL=0x01,
    ROBOT_MODE_RETURN=0x02,
    ROBOT_MODE_WAITING = 0x03,
    ROBOT_MODE_AUTO_TAKEOFF = 0xa0
    
} RobotMode;
class RobotModel {
public:
    RobotModel();
    ~RobotModel();
public:
    int init(LinuxSetup* setup);

private://硬件资源
    UsbCaptureWithThread mUsbCapture;
    RealsenseInterface mRealsense;
    Vehicle* mRvehicle;
    SerialInterface mSerialInterface;
    RadioInterface mRadio;
    LinuxSetup* mSetup;
    bool is_ball_contain = false;
    
    float kp,ki,kd;

private://机器人数据模型
    pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER; //互斥变量锁
    unsigned char mRobotId;//机器人id
    RobotMode mCurrentMode;//机器人当前运行模式
    float mCurrentPitch=0;//当前云台pitch角度
    float mCurrentYaw=0;//当前云台yaw角度
    TypeMap<TOPIC_STATUS_FLIGHT>::type     mFlightStatus;
    TypeMap<TOPIC_GPS_FUSED>::type         mLatLon;
    TypeMap<TOPIC_ALTITUDE_FUSIONED>::type mAltitude;
    TypeMap<TOPIC_RC>::type                mRc;
    TypeMap<TOPIC_VELOCITY>::type          mVelocity;
    TypeMap<TOPIC_QUATERNION>::type        mQuaternion;
    BasicTool basictool;
    Telemetry::GlobalPosition  mStartPoint;
    string log;
public://硬件资源获取函数接口
    UsbCaptureWithThread* getpUsbCapture();
    SerialInterface* getpSerialInterface();
    RealsenseInterface* getRealsenseCpature();
    RadioInterface* getpRadioInterface();
    LinuxSetup* getLinuxSetup();
    
public://机器人具体数据读写函数接口
    /*******系统框架调用接口，机器人数据更新*****/
    void mcuDataUpdate(float pitch,float yaw);
    void DataUpdate(SerialPacket recv_packet);
    void PIDUpdate(float Kp,float Ki, float Kd);
    Point3f getPIDParam();
   void release(); 
    /**************用户接口****************/
    Point3f getCurrentVelocity();
    Point3f getCurrentAngle();
    Eigen::Matrix3f getRotation_matrix(Point3f CurrentAngle);
    unsigned char getRobotId();
    RobotMode getCurrentMode();
    void setCurrentMode(RobotMode robotMode);
    void ballStateUpdate(bool is_ok);
    bool getBallState();
    void setOriginPoint();
    Telemetry::GlobalPosition getOriginPoint();
    void setLog(string str);
    string getLog();
};

#endif //RMDEMO_ROBOT_MODEL_H
