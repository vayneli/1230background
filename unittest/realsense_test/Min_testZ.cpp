#include <opencv2/opencv.hpp>
#include "realsense.h"
#include "pid.h"
#include "aim.h"
#include "socket.hpp"
#include "ACsaliencydetect.h"
//#include <curses.h>
#include <time.h>  
#include <sys/time.h>
#include <deque>   
#include "Commands.hpp"
#include <mutex>
#include <string>
mutex lo;
//
void *threadFunction(void *args)
 
{    
    string askstr="#x $";
    Commands *commands = (Commands*)args;
    while(true)
    {
        Socket client1;
        client1.client_init("127.0.0.1");
        client1.send_data(askstr.c_str());
        client1.receive_data();
        cout <<"receive sussfully !"<<client1.receive_buff << endl;
        lo.lock();
        commands->getCommand(client1.receive_buff); 
        lo.unlock();
        close(client1.sock_fd);
    }
}

int main(){

    RealsenseInterface realsense_getxyz;
    Aim_ball ball_aim;
    AC_saliency ac;
    CAMERA_INRINSIC_PARAMETERS c;
    Commands commands;
    string askstr="#x $";
    //创建线程，将数据发送到地面站，并接受地面站的pid参数
    // pthread_t threadID;//
    // pthread_create(&threadID,NULL,threadFunction,&commands);

    //记录数据，格式为（）
    time_t totalseconds = time(NULL);//获取总妙数
    struct tm *st = localtime(&totalseconds);
    string log_name = to_string(st->tm_year + 1900) + "-" + to_string(st->tm_mon + 1) + "-" + 
    to_string(st->tm_mday) + "-" + to_string(st->tm_hour) + "-" + to_string(st->tm_min) 
    + "-" + to_string(st->tm_sec) + ".txt";
    ofstream log;
    log.open("./log/" + log_name);
    mutex mut; 
 
    Point3f pc,pa;
    c.fx=617;
    c.fy=618;
    //645.277749 648.004580 331.576464 231.011424 -0.004489 0.002333 0.169973 -0.357225
    c.cx=321;
    c.cy=235;
    c.scale=1000.0;
    float tx=0.1;
    float ty=-0.045;
    float tz=0;
    Point3f T(0.1,-0.045,0);

    if(realsense_getxyz.init(640,480) == 0){
        cout << "RealsenseCapture init successed!" <<endl;
        usleep(10000000);
    }

    Mat depth;
    Mat color;
    static VideoWriter writer_color,writer_depth;
    string color_name="./Z"+to_string(st->tm_year + 1900) + "-" + to_string(st->tm_mon + 1) + "-" + 
    to_string(st->tm_mday) + "-" + to_string(st->tm_hour) + "-" + to_string(st->tm_min) 
    + "-" + to_string(st->tm_sec) + "color.avi";
    string depth_name="./Z"+to_string(st->tm_year + 1900) + "-" + to_string(st->tm_mon + 1) + "-" + 
    to_string(st->tm_mday) + "-" + to_string(st->tm_hour) + "-" + to_string(st->tm_min) 
    + "-" + to_string(st->tm_sec) + "depth.avi";
    writer_color.open(color_name,CV_FOURCC('M','J','P','G'),25,Size(640,480),1);
    writer_depth.open(depth_name,CV_FOURCC('M','J','P','G'),25,Size(640,480),1);
    //writer.open("log.avi",CV_FOURCC('M','J','P','G'),25,Size(640,480),0);
    
    
    Point3f d_d,d_a,d_ac;
    String str_1,str_2;
    float Velocity_x,Velocity_y,Velocity_z;

    //定义并初始化pid对象
    PIDControl pid_x;
    PIDControl pid_y;
    PIDControl pid_z;
    
    pid_x.init(2.0,0,0,0.01f,-10,10,AUTOMATIC,DIRECT);
    pid_y.init(2.0,0,0,0.01f,-10,10,AUTOMATIC,DIRECT);
    pid_z.init(1,0,0,0.01f,-5,5,AUTOMATIC,DIRECT);

    pid_x.PIDSetpointSet(0);
    pid_y.PIDSetpointSet(0); 
    pid_z.PIDSetpointSet(0.5);
    ushort d;
	
    char receive_buff[1024];
    Mat image_tmp(Size(640,480),CV_8UC1); 
    Mat image_tmp1,image_tmp2,image_tmp3;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    Point2d pt; 
    int count=0;
    struct timeval dwStart;
            struct timeval dwEnd;
            unsigned long dwTime=0;
            unsigned long dwTime1=0;
             gettimeofday(&dwStart,NULL);
    
    int nn = 0;
    double time = 0;
    while(true)
    {
        // nn++;
        // gettimeofday(&dwEnd,NULL);
        // dwTime = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);
        // cout << "timedelay:" << dwTime << endl;

        Socket client1;
        client1.client_init("127.0.0.1");
        client1.send_data(askstr.c_str());
        usleep(100);
         
        client1.receive_data();
      
        commands.getCommand(client1.receive_buff);
        cout << "pid param: " << commands.control_command.pitch << " " << 
        commands.control_command.roll << " " <<
        commands.control_command.yaw << " " << endl;        
        pid_x.PIDTuningsSet(commands.control_command.pitch, commands.control_command.roll, 
        commands.control_command.yaw);
        close(client1.sock_fd);


        if(realsense_getxyz.getDepthImg(depth) == 0){
            realsense_getxyz.getColorImg(color);
                
        	Point3f tp=ball_aim.getDistance(depth);
            
            d_ac=ac.getC_xyz(tp,c);
            pid_x.PIDInputSet(-d_ac.x);
            pid_y.PIDInputSet(-d_ac.y);
            pid_z.PIDInputSet(d_ac.z);
            cout<<"xyz_ac:"<<-d_ac<<endl;

            //若之后需要考虑飞机的旋转角，调用robotmodel中的变换矩阵相乘即可

            if(pid_x.PIDCompute()==false)std::cout<<"error pid_x"<<std::endl;//fix
            if(pid_y.PIDCompute()==false)std::cout<<"error pid_y"<<std::endl;
            if(pid_z.PIDCompute()==false)std::cout<<"error pid_z"<<std::endl;
            Velocity_x= -pid_x.PIDOutputGet();
            Velocity_y= -pid_y.PIDOutputGet();
            Velocity_z = pid_z.PIDOutputGet();
          
        cout<<"Velocity_x: "<<Velocity_x<<endl;   
        cout<<"Velocity_y: "<<Velocity_y<<endl;
        cout<<"Velocity_z: "<<Velocity_z<<endl;              
            
    
	    Socket client;
        client.client_init("127.0.0.1");
	    

        client.send_data(Velocity_x,Velocity_y,0);
        // cout << "send successful" << endl;
        close(client.sock_fd);
        
        string str = to_string((float)clock()/CLOCKS_PER_SEC) + " " + to_string(-d_ac.x) + " " + 
        to_string(-d_ac.y) + " " + to_string(d_ac.z) + " " + to_string(Velocity_x) + " " + 
        to_string(Velocity_y) + " " + to_string(Velocity_z);
        // cout << str << endl;
        mut.lock();
        log << str << endl;
        mut.unlock();
        writer_color.write(color);
        depth.convertTo(image_tmp1,CV_8UC1);
        cvtColor(image_tmp1,image_tmp1,CV_GRAY2BGR);
        writer_depth.write(image_tmp1);
        
            //if(waitKey(1)>0) break;
        }else{
             cout<<"error"<<endl;
        }
        //float velocity_z = pid_z.PIDOutputGet();
    
        // gettimeofday(&dwEnd,NULL);
        // dwTime1 = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);
        // time += (dwTime1 - dwTime)/1000.0;
        // cout << "timedelay2: " << time/nn << endl; 
    }
     log.close();
        return 0;
}



  
