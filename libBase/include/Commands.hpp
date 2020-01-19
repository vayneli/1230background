#include <string>
#include <vector>


struct Control_command
{
    float speed_x, speed_y, speed_z, yaw_rate;
    bool key_1;
    bool key_2;
    bool key_3;
    int num;

    // Control_command(float x,float y,float z,float w):
    // speed_x(x),speed_y(y), speed_z(z), yaw_rate(w){};

};


struct PID_command
{
    float kp;
    float ki;
    float kd;
    float SampleTime;
    int num;
};


class Commands
{

public:
    int num_v = 4; //一条指令有n-1个变量
    int num_c = 20;//一个字符串中最多有20条指令
    char type;
public:
    Control_command control_command;//一条命令
    PID_command pid;
    std::string pid_param = "#p4 0 0 0 0 $";
    std::string ground_data = "#c4 0 0 0 0 $";
    std::vector<Control_command> myCommand;//多条命令m 


public:
    Commands();
    ~Commands();
    int getCommand(const char *data);
    int clearCommand();
    int fuckCommandC(std::string tmp, int begin);//解析无人开机控制命令，并存在control_command 中
    int fuckCommandS();//解析请求发送
    int fuckCommandX();//发送姿态位置
    int fuckCommandP(std::string tmp, int begin);//解析pid调参命令
    
    
};



