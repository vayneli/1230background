#include "aim.h"
#define RANGE 7000
//#define RANGE 2000
Aim_ball::Aim_ball(){}
Aim_ball::~Aim_ball(){}

Mat Aim_ball::setImage(Mat depth){
   Mat output(Size(640,480),CV_8UC1);
   inRange(depth,300,RANGE,output);
   return output;
}

Mat Aim_ball::segmentInRange(Mat depth,int min, int max){
    Mat output(Size(640,480),CV_8UC1);
    inRange(depth,min,max,output);
    return output;
}

float Aim_ball::P2Pdistance(Point2d point1,Point2d point2){
    return sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2));
}
bool Aim_ball::contoursFinder_area(Mat src,Mat depth,vector<vector<Point> > &contours,Point3f &point, int &count){
    count = 0;
    vector<Vec4i> hierarcy;
    findContours(src, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if(contours.size()>0){
        Rect temp0 = boundingRect(contours[0]);
        for(size_t i=0;i<contours.size();i++){    
            Rect tempi = boundingRect(contours[i]);   
            if(tempi.area()>temp0.area()){
                temp0=tempi; 
                count = i;
            }
        }	
        Point2f center;
        float radius;
        minEnclosingCircle(contours[count],center,radius);
        point.x = center.x;
        point.y = center.y;
        point.z = radius;
        return true;

    }else return false;
}

bool Aim_ball::contoursFinder_distance(Mat src,Mat depth,vector<vector<Point> > &contours,Point3f &point, int &count){
    count = 0;
    vector<Vec4i> hierarcy;
    findContours(src, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if(contours.size()>0){
        Point2f center;
        float radius;
        minEnclosingCircle(contours[0],center,radius);
        float dis0 = depth.at<ushort>(center.y,center.x);
        for(size_t i=0;i<contours.size();i++){    
            Point2f center1;
            float radius1;
            minEnclosingCircle(contours[i],center1,radius1);  
            float dis1 = depth.at<ushort>(center1.y,center1.x);
            if(dis1<dis0){
                dis0=dis1; 
                count = i;
            }
        }	
        Point2f center2;
        float radius2;
        minEnclosingCircle(contours[count],center2,radius2);
        point.x = center2.x;
        point.y = center2.y;
        point.z = radius2;
        return true;

    }else return false;
}

bool Aim_ball::ballChecker(vector<Point> contour,Mat mask, Point2f center, float radius){
    Rect rec = boundingRect(contour);
    double none_zeor_area = countNonZero(mask(rec));
    double area = contourArea(contour,false);
    double standard_area = radius*radius*CV_PI;
    cout<<"k_n: "<<none_zeor_area/standard_area<<endl;
    cout<<"k: "<<area/standard_area<<endl;
    if(area/standard_area >= 0.75){
        return true;
    }else{
       return false;
    }
}

TargetFinderResult Aim_ball::targetSaprate(Mat src, Mat depth){
    TargetFinderResult result;
    vector<vector<Point> > contours;
    drone_contours.clear();
    ball_contours.clear();
    drone = Point3d(-1,-1,-1);
    ball = Point3d(-1,-1,-1);
    Mat temp1;
    Point3f point_1_step;
    Canny(src,temp1,100,300);
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));//3*3全1结构元素
    Mat element1 = getStructuringElement(MORPH_RECT, Size(7, 7));
	morphologyEx(temp1,temp1, cv::MORPH_CLOSE, element);
    int index_1 = 0;
    bool complete = false;
    bool target_is_ball = false;
    target_contain = RESULT_NONE;
    //先寻找0-20米空域内最大轮廓以确定空中是否有未知目标
    if(contoursFinder_distance(temp1,depth,contours,point_1_step,index_1)){
        Mat temp2,temp3; 
        int max_range;
        max_range = depth.at<ushort>(point_1_step.y,point_1_step.x);
       // if(max_range>1000){
            //优先确定当前检测目标是否是球
	    Mat temp4 = segmentInRange(depth,max_range-300,max_range+300);
            morphologyEx(temp4,temp4, cv::MORPH_CLOSE, element);
            Mat temp = temp4;
            Canny(temp4,temp4,100,300);
            vector<vector<Point> > contours3;
            Point3f point_4_step;
            int index_4;
            if(contoursFinder_distance(temp4,depth,contours3,point_4_step,index_4)){
                if(ballChecker(contours3[index_4],temp,Point2f(point_4_step.x,point_4_step.y),point_4_step.z)){
                    ball_contours = contours3[index_4];
                    ball = point_4_step;
                    target_is_ball = true;
                }else{
                    target_is_ball = false;
                    drone_contours = contours3[index_4];
                    drone = point_4_step;
                }


                //寻找飞机
                if(target_is_ball){

                    //检测目标上方与下方空域
                    if(max_range+3500>RANGE)
                            temp2 = segmentInRange(depth,max_range+1500,RANGE);
                        else
                            temp2 = segmentInRange(depth,max_range+1500,max_range+3500);
                    morphologyEx(temp2,temp2, cv::MORPH_CLOSE, element1);
                    Canny(temp2,temp2,100,300);

                    vector<vector<Point> > contours1;
                    Point3f point_2_step;
                    int index_2;
                    //如果上方空域有目标，则认为该目标是无人机
                    if(contoursFinder_area(temp2,depth,contours1,point_2_step,index_2)){
                        drone_contours = contours1[index_2];
                        drone = point_2_step; 
                    }else{//如果上方无目标且下方发现目标，则认为刚刚球检测错误，将下方目标定义为球，首次检测到的目标定义为无人机
                        if(max_range-3500>=300)                
                            temp3 = segmentInRange(depth,max_range-3500,max_range-1300);
                        else temp3 = segmentInRange(depth,300,max_range-1300);
                        morphologyEx(temp2,temp2, cv::MORPH_CLOSE, element1);
                        Canny(temp2,temp2,100,300);
                        if(contoursFinder_area(temp2,depth,contours1,point_2_step,index_2)){
                            drone = ball;
                            drone_contours = ball_contours;
                            ball_contours = contours1[index_2];
                            ball= point_2_step; 
                        }
                    }
                    //imshow("t2",temp2);
                }else{//寻找球
                    if(max_range-3500>=300)                
                        temp3 = segmentInRange(depth,max_range-3500,max_range-1300);
                    else temp3 = segmentInRange(depth,300,max_range-1300);
                    Canny(temp3,temp3,100,300);
                    morphologyEx(temp3,temp3, cv::MORPH_CLOSE, element);
                    vector<vector<Point> > contours2;
                    Point3f point_3_step;
                    int index_3;
                    if(contoursFinder_area(temp3,depth,contours2,point_3_step,index_3)){
                            ball_contours = contours2[index_3];
                            ball = point_3_step;
                    }else{ 
                        if(max_range+3500>RANGE)
                            temp3 = segmentInRange(depth,max_range+1000,RANGE);
                        else
                            temp3 = segmentInRange(depth,max_range+1000,max_range+3500);

                        morphologyEx(temp3,temp3, cv::MORPH_CLOSE, element1);
                        Canny(temp3,temp3,100,300);
                        if(contoursFinder_area(temp3,depth,contours2,point_3_step,index_3)){
                            ball = drone;
                            ball_contours = drone_contours;
                            drone_contours = contours2[index_3];
                            drone = point_3_step; 
                        }
		            }	
                }
            }   
    }   
    
    cout<<"ball: "<<ball<<endl;
    cout<<"drone: "<<drone<<endl; 
    if(ball.x!=-1&&drone.x!=-1) result = RESULT_BOTH_BALL_UAV;
    else if(ball.x==-1&&drone.x!=-1) result = RESULT_ONLY_UAV;
    else if(ball.x!=-1&&drone.x==-1) result = RESULT_ONLY_BALL;
    else result = RESULT_NONE;
    target_contain = result;
    return result;

}

TRACKINGTARGET Aim_ball::setTarget(Mat &src,Mat depth,Point3f &point1,Point3f &point2){
    TRACKINGTARGET result;

    point1.x = drone.x;
    point1.y = drone.y;
    if(drone.x!=-1)
        point1.z = depth.at<ushort>(drone.y,drone.x);
    
    point2.x = ball.x;
    point2.y = ball.y;
    if(ball.x!=-1)
        point2.z = depth.at<ushort>(ball.y,ball.x);
    if(drone.x!=-1){
        circle(src,Point(drone.x,drone.y),drone.z,Scalar(0,255,255),2);
    }
    if(ball.x!=-1){
        circle(src,Point(ball.x,ball.y),ball.z,Scalar(0,0,255),2);
    }
    switch(target_contain){
        case RESULT_ONLY_BALL:{
            point2.x = ball.x;
            point2.y = ball.y;
            point2.z = depth.at<ushort>(ball.y,ball.x);
	    while(point2.z==0){
		if(ball.x<640){
		    ball.x = ball.x+1;
		}else{
		    break;
		}
	    	point2.z = depth.at<ushort>(ball.y,ball.x);
	    }
            result = TRACKING_BALL;
            
            break;
        }
        case RESULT_ONLY_UAV:{
            point1.x = drone.x;
            point1.y = drone.y;
            point1.z = depth.at<ushort>(drone.y,drone.x);
	    int temp = drone.x;
	     while(point1.z==0){
                if(temp<640){
                    temp = temp+1;
                }else{
                    break;
                }
                point1.z = depth.at<ushort>(drone.y,temp);
            }
            result = TRACKING_UAV;

            break;
        }
        case RESULT_BOTH_BALL_UAV:{
            int uav_height,ball_height;
            Point2d b_p = Point2d(ball.x,ball.y);
            Point2d d_p = Point2d(drone.x,drone.y);
	        int temp = ball.x;
            uav_height = depth.at<ushort>(drone.y,drone.x);
		    while(uav_height==0){
                if(drone.x<640){
                    drone.x = drone.x+1;
                }else{
                    break;
                }
                uav_height = depth.at<ushort>(drone.y,drone.x);
            }
            point1.z = uav_height;
            point2 = ball;
	        point2.z= depth.at<ushort>(ball.y,ball.x);
	        while(point2.z==0){
                if(ball.x<640){
                    ball.x = ball.x+1;
                }else{
                    break;
                }
                point2.z = depth.at<ushort>(ball.y,ball.x);
            }
            result = TRACKING_IN_TESTING_UAV;
            break;
        }
    }
    
    return result;
}



Point3f Aim_ball::getC_xyz(Point3f& point_p,CAMERA_INRINSIC_PARAMETERS& camera){

    Point3f pc;
    pc.z=point_p.z/camera.scale;
    pc.y=(point_p.y-camera.cy)*pc.z/camera.fy;
    pc.x=(point_p.x-camera.cx)*pc.z/camera.fx;

    return pc;
}
