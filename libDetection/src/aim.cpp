#include "aim.h"
#define RANGE 20000
//#define RANGE 2000
Aim_ball::Aim_ball(){}
Aim_ball::~Aim_ball(){}

Mat Aim_ball::setImage(Mat depth){
   Mat output(Size(640,480),CV_8UC1);
   inRange(depth,500,RANGE,output);
   return output;

}

bool Aim_ball::findTarget(Mat &src,Mat &depth,Point3f &point){
    int count = 0;
    //Mat src;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
   // cout<<"aa"<<endl;
    //cvtColor(src,src,CV_BGR2GRAY);
    Canny(src,src,100,300);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));//3*3全1结构元素
	morphologyEx(src,src, cv::MORPH_CLOSE, element);
    //cout<<"111"<<endl;
    findContours(src, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    imshow("contours:",src);
//    waitKey(1);
    //cout<<"1111"<<endl;
    Point2d pt;
    if(contours.size()>0){
        Rect temp0 = boundingRect(contours[0]);
        for(size_t i=0;i<contours.size();i++){    
            Rect tempi = boundingRect(contours[i]);   
            if(tempi.area()>temp0.area()){
                temp0=tempi; 
                count=(int)i;
            }
        }	
            pt = Point2i(temp0.x+temp0.width/2,temp0.y+temp0.height/2);
    }
    else pt=Point2i(320,240);
    int distance,distance2;
    if(pt.x==320&&pt.y==240) {
        distance=0;
        return false;
    }else{
/*        int sum;
        int pixel_count=0;
        for(int i = pt.x-5;i<pt.x+5 && i<640;i++){
            for(int j = pt.y-5;j<pt.y+5 && j<480;j++ ){
                if(i>=0&&j>=0){
                    sum+=depth.at<ushort>(j,i);
                    pixel_count++;
                }
            }
        }
        if(pixel_count>0){ */
           // distance = sum/pixel_count;
	   // distance=1;
            distance = depth.at<ushort>(pt.y,pt.x);
            //cout<<"distance:"<<distance<<endl;
            //distance2 = depth.at<ushort>(pt.x,pt.y);
            //cout<<"distance2:"<<distance<<endl;
            point=Point3f(pt.x,pt.y,distance);
            return true;
       // }else{
         //   return false;
        }


   // }


}

Point3f Aim_ball::getC_xyz(Point3f& point_p,CAMERA_INRINSIC_PARAMETERS& camera){

    Point3f pc;
    pc.z=point_p.z/camera.scale;
    pc.y=(point_p.y-camera.cy)*pc.z/camera.fy;
    pc.x=(point_p.x-camera.cx)*pc.z/camera.fx;

    return pc;
}
