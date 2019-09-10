
#include <string>
#include <vector>


struct Control_command
{
    float pitch, roll, yaw, yaw_rate;
    bool key_1;
    bool key_2;
    bool key_3;
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
    std::vector<Control_command> myCommand;//多条命令


public:
    Commands();
    ~Commands();
    int getCommand(const char *data);
    int clearCommand();
    int fuckCommandC(std::string tmp, int begin);//解析无人开机控制命令，并存在control_command 中
    
};



