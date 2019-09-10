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
    string color_name=to_string(st->tm_year + 1900) + "-" + to_string(st->tm_mon + 1) + "-" + 
    to_string(st->tm_mday) + "-" + to_string(st->tm_hour) + "-" + to_string(st->tm_min) 
    + "-" + to_string(st->tm_sec) + "color.avi";
    string depth_name=to_string(st->tm_year + 1900) + "-" + to_string(st->tm_mon + 1) + "-" + 
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

    int data_length = 9;
    std::deque<float> data_x(data_length);
    std::deque<float> data_y(data_length);

    
    pid_x.init(2.0,0,0,0.01f,-20,20,AUTOMATIC,DIRECT);
    pid_y.init(2.0,0,0,0.01f,-20,20,AUTOMATIC,DIRECT);
    pid_z.init(1,0,0,0.01f,-5,5,AUTOMATIC,DIRECT);

    pid_x.PIDSetpointSet(0);
    pid_y.PIDSetpointSet(0); 
    pid_z.PIDSetpointSet(0.5);


    //time_t c=time(NULL);
    //while(std::difftime(time(NULL),c)<60)
    ushort d;
	
    char receive_buff[1024];
    Mat image_tmp(Size(640,480),CV_8UC1); 
    Mat image_tmp1,image_tmp2,image_tmp3;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    Point2d pt; 
    int count=0;
    //client.client_init("127.0.0.1");
    //VideoCapture cap1("/home/vayneli/test_video/0327/8_depth.avi");
    //VideoCapture cap1("/home/vayneli/test_video/0719/depth.avi");
    //VideoCapture cap2("/home/vayneli/test_video/0719/color.avi");
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
                
        	for(int i=0;i<depth.rows;i++){
                for(int j=0;j<depth.cols;j++){  
                    if(depth.at<ushort>(i,j)>1000)
                        image_tmp.at<uint8_t>(i,j)=255;
                    else
                        image_tmp.at<uint8_t>(i,j)=0;
                }
            } 
	    cvtColor(image_tmp,image_tmp1,CV_GRAY2BGR);
        
            //realsense_getxyz.getColorImg(color);
            //convertTo转换是channel得相同；
            //cout<<"image_tmp.type():"<<image_tmp.type()<<endl;
            //imshow("depth_8",image_tmp);
        threshold(image_tmp, image_tmp2, 20, 255, THRESH_BINARY);
	
	    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));//3*3全1结构元素
	    morphologyEx(image_tmp2, image_tmp3, cv::MORPH_CLOSE, element);
        findContours(image_tmp3, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	    drawContours(image_tmp3,contours,-1,Scalar(0,255,0),1,8,hierarcy);
         
        if(contours.size()>0)
        {
            Rect temp0 = boundingRect(contours[0]);
            for(size_t i=0;i<contours.size();i++){
            
                Rect tempi = boundingRect(contours[i]);   
                if(tempi.area()>temp0.area()){
                    temp0=tempi; 
                    count=(int)i;
                }
            }	
            pt = Point2d(temp0.x+temp0.width/2,temp0.y+temp0.height/2);
        }
        else pt=Point2d(320,240);
	//add update_flag and zhongxin 
	/*for(size_t i=0;i<contours.size();i++){
                Rect temp0 = boundingRect(contours[0]);    
                Rect tempi = boundingRect(contours[i]);   
                if(tempi.area()>temp0.area()){
                    temp0=tempi; 
                    count=(int)i;
                }
                cv::Moments mom = cv::moments(cv::Mat(contours[count]));
                if(mom.m00!=0){
                // 画重心
                    pt = cv::Point(mom.m10 / mom.m00, mom.m01 / mom.m00);	//使用前三个矩m00, m01和m10计算重心	
                }
                else if(update_flag<=10){
                
                    update_flag++;
                }
                else{
                    pt=Point2d(320,240);
                    update_flag=0;
                }
            }*/
	   
	    // circle(color, pt, 10, cv::Scalar(0, 0, 255), 4,8);//画红点
        circle(image_tmp3,pt,10,Scalar(0,0,255),4,8);
	    circle(image_tmp1,pt,10,Scalar(0,0,255),4,8);
	    cout<<"pt:"<<pt<<endl;        
	    if(pt.x==320&&pt.y==240) d=0;
	    else
            d=depth.at<ushort>(pt.y,pt.x);
            d_d=Point3f(pt.x,pt.y,d);
            
	    if(d!=0)
        {
            d_ac=ac.getC_xyz(d_d,c);
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
        }
        else
        {
            Velocity_x=0;
            Velocity_y=0;
            Velocity_z = 0;
        }  
        cout<<"Velocity_x: "<<Velocity_x<<endl;   
        cout<<"Velocity_y: "<<Velocity_y<<endl;        

        //     pc={commands.control_command.pitch,commands.control_command.roll
        //     ,commands.control_command.yaw};
	    // //pc={commands.control_command.pitch,commands.control_command.roll
        //     //,-commands.control_command.yaw};
        //     //Rotation_matrix=getRotation_matrix(pc);
        //     Vec3f eulerAngle={pc.y,pc.x,-pc.z};
	    // Rotation_matrix=eulerAnglesToRotationMatrix(eulerAngle);
            
    
	    Socket client;
        client.client_init("127.0.0.1");
	    
    

        // pcv={d_ac.x+0.1,d_ac.y-0.04,-(d_ac.z+0.045)};
	    // pcv={d_ac.x,d_ac.y,d_ac.z};	
        // pav=(Rotation_matrix)*pcv;
	    // pav=(Rotation_matrix.inverse())*pcv;

        // pa={pav[0],pav[1],pav[2]};

        // cout<<"xyz_pa:"<<pa<<endl;
        //中值滤波
        // d_ac=pa;

        // data_x.pop_front();
        // data_x.push_back(d_ac.x);
        // d_ac.x = midFilter(data_x);
        
        // data_y.pop_front();
        // data_y.push_back(d_ac.y);
        // d_ac.x = midFilter(data_x);
	       
        //摄像机坐标系的XYZ和飞机body坐标系相反。因此需要转换为负的
	        //pid_x.PIDInputSet(-d_ac.x);
            //pid_y.PIDInputSet(-d_ac.y);
            //pid_z.PIDInputSet(d_ac.z);

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



  
