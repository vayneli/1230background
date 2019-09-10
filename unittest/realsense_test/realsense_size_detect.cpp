#include <iostream>
#include <opencv2/opencv.hpp>
#include "realsense.h"
#include <string.h>

using namespace std;
using namespace cv;

int main(int argc,char** argv){

    RealsenseInterface realsense_size;
    if(realsense_size.init(640,480)==0){

        cout<<"Realsense origin color init success"<<" "<<realsense_size.init(640,480)<<endl;
        usleep(5000000);
    }
else cout<<"realsense initilize error"<<endl;

Mat color,depth;
//static VideoWriter writer_color,writer_depth;

//writer_color.open("color.avi",CV_FOURCC('M','J','P','G'),30,Size(640,480),1);
//writer_depth.open("depth.avi",CV_FOURCC('M','J','P','G'),30,Size(640,480),0);
ostringstream os;
int i=0;

while(i<10){
    if(realsense_size.getColorImg(color)==0&&realsense_size.getDepthImg(depth)==0){
//if(realsense_size.isColorImgUpdate){//realsense_size.getDepthImg(depth)==0
        imshow("realsense origin:",color);
        imshow("realsense depth:",depth);
        os<<"depth"<<setw(4)<<setfill('0')<<to_string(i)<<".png"<<endl;
        imwrite(os.str(),depth);
        os.str("");
        os.clear();
        i++;
        //writer_color.write(color);
        //writer_depth.write(depth);
        if(waitKey(1)>0) break;
    }
    else cout<<realsense_size.getDepthImg(depth)<<" "<<"no depth image"<<endl;
}
    return 0;
}


