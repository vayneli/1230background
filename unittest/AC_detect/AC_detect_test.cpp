#include <opencv2/opencv.hpp>
#include "realsense.h"
#include "pid.h"
#include "ACsaliencydetect.h"
//#include <curses.h>
#include <time.h>        

int main(){
    RealsenseInterface realsense;
    //Aim_ball ball_aim;
    AC_saliency ac_saliency;
    Point3f p;
    if(realsense.init(640,480) == 0){
        cout << "RealsenseCapture init successed!" <<endl;
        usleep(1000000);
    }
    Mat depth;
    Mat color;
    
    p=ac_saliency.getXY(color,depth);
    cout<<"p"<<endl;
}
