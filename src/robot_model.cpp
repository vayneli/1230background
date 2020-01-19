////////////////////////////////////////////////////////////////////////////////
///Copyright(c)      UESTC MBZIRC     Model Code for UAV
///ALL RIGHTS RESERVED
///@file:control_model.cpp
///@brief: UAV 模型，包含对所有消息的管理，负责与无人机进行信息交互
/// 
///@vesion 1.0
///@author: PC
///@email: 694977655@qq.com
///@date: 18-11-6
///修订历史：
////////////////////////////////////////////////////////////////////////////////
#include <unistd.h>
#include <robot_model.h>
#include <basic_tool.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace cv;
using namespace std;

RobotModel::RobotModel() {

}
RobotModel::~RobotModel() {

}
void RobotModel::release(){
	mRealsense.join();
	mSetup->~LinuxSetup();
}
int RobotModel::init(LinuxSetup* setup){
    mSetup = setup;
    string serialPath,videoPath;
    cv::FileStorage f("../res/parameters.yaml", cv::FileStorage::READ);
    f["PID_P"] >> kp;
    f["PID_I"] >> ki;
    f["PID_D"] >> kd;
    f.release();
    //cout<<kp<<endl;
    
    //配置文件 error when build in TX2
    //
    //f["robot_id"] >> mRobotId;//机器人id
    //f["serial_path"] >> serialPath;//机器人串口路径
    //f["capture_path"] >> videoPath;//机器人摄像头路径
    //f.release();
    //初始化串口
    
   if(mRadio.init("/dev/ttyUSB0")==0){
       cout<<"[robot model init ]: RobotRadioInterface init successed!"<<endl;
    }
    if(mSerialInterface.init(mSetup)==0) {
        cout<<"[robot model init ]: RobotSerialInterface init successed!"<<endl;
        //mSerialPort.ShowParam();
    } else{
        cout<<"[robot model init ]: RobotSerialInterface init failed!"<<endl;
    }
    if(mRealsense.init(640,480) == 0){
        cout << "[robot model init ]:RealsenseCapture init successed!" <<endl;
        usleep(5000000);
    }
     //初始化摄像头
    // if(mUsbCapture.init("/dev/Video",1280,720)==0){
    //     cout << "[robot model init ]:UsbCapture init successed!" <<endl;
    //     usleep(1000000);//等待1s
    //     //mUsbCapture.infoPrint();
    // }else{
    //     cout << "[robot model init ]:UsbCapture init failed! No UsbCapture" <<endl;
    // }
    mCurrentMode=ROBOT_MODE_EMPTY;

}

unsigned char RobotModel::getRobotId(){
    return mRobotId;
};

LinuxSetup* RobotModel::getLinuxSetup(){
    return mSetup;
}


UsbCaptureWithThread* RobotModel::getpUsbCapture() {
    return &mUsbCapture;
}

RealsenseInterface* RobotModel::getRealsenseCpature(){
    return &mRealsense;
}

RadioInterface* RobotModel::getpRadioInterface(){
    return &mRadio;
}

void RobotModel::setCurrentMode(RobotMode robotMode) {
    mCurrentMode=robotMode;
}
SerialInterface* RobotModel::getpSerialInterface(){//add define
    return &mSerialInterface;
}


RobotMode RobotModel::getCurrentMode() {
    return mCurrentMode;
}

void RobotModel::ballStateUpdate(bool is_ok){
    is_ball_contain = is_ok;
}


//机器人数据接口
void RobotModel::DataUpdate(SerialPacket recv_packet){
    pthread_mutex_lock(&dataMutex);
    mFlightStatus = recv_packet.flightStatus;
    mLatLon = recv_packet.latLon;
    mAltitude = recv_packet.altitude;
    mRc = recv_packet.rc;
    mVelocity = recv_packet.velocity;
    mQuaternion = recv_packet.quaternion;
    pthread_mutex_unlock(&dataMutex);
}

void RobotModel::PIDUpdate(float Kp,float Ki,float Kd){
    
    pthread_mutex_lock(&dataMutex);
    kp =Kp;
    ki = Ki;
    kd = Kd;
    pthread_mutex_unlock(&dataMutex);

}
Point3f RobotModel::getPIDParam(){
    Point3f tmp;
    pthread_mutex_lock(&dataMutex);//加锁
    tmp.x = kp;
    tmp.y = ki;
    tmp.z = kd;
    pthread_mutex_unlock(&dataMutex);
    return tmp;
}
Point3f RobotModel::getCurrentVelocity(){
    Point3f tmp;
    pthread_mutex_lock(&dataMutex);//加锁
    tmp.x = mVelocity.data.x;
    tmp.y = mVelocity.data.y;
    tmp.z = mVelocity.data.z;
    pthread_mutex_unlock(&dataMutex);
}
Point3f RobotModel::getCurrentAngle(){
    Telemetry::Vector3f tmp_;
    Point3f tmp;
    pthread_mutex_lock(&dataMutex);
    tmp_ = basictool.toEulerAngle(mQuaternion);
    tmp.x = tmp_.x;
    tmp.y = tmp_.y;
    tmp.z = tmp_.z;
    pthread_mutex_unlock(&dataMutex);

    return tmp;
}

Eigen::Matrix3f RobotModel::getRotation_matrix(Point3f CurrentAngle){

    Eigen::Vector3f Rotation_vector(CurrentAngle.z,CurrentAngle.y,CurrentAngle.x);
    Eigen::AngleAxisf rotation_vector;
    rotation_vector=Eigen::AngleAxisf(Rotation_vector[0],Eigen::Vector3f::UnitZ())*
                    Eigen::AngleAxisf(Rotation_vector[1],Eigen::Vector3f::UnitY())*
                    Eigen::AngleAxisf(Rotation_vector[2],Eigen::Vector3f::UnitX());

    Eigen::Matrix3f rotation_matrix=rotation_vector.matrix();

    return rotation_matrix;
}

bool RobotModel::getBallState(){
    return is_ball_contain;
}

void RobotModel::setOriginPoint(){
    mStartPoint = mSerialInterface.getGPSposition();
}

Telemetry::GlobalPosition RobotModel::getOriginPoint(){
    return mStartPoint;
}

void RobotModel::setLog(string mLog)
{
    pthread_mutex_lock(&dataMutex);
    log = mLog;
    pthread_mutex_unlock(&dataMutex);
}

string RobotModel::getLog()
{
    pthread_mutex_lock(&dataMutex);
    string mLog = log;
    pthread_mutex_unlock(&dataMutex);
    return mLog;
}
