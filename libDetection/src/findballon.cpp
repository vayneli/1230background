#include"findballon.h"
#include "line2Dup.h"
using namespace cv;
cv::Mat Findballon::Denoising(cv::Mat &img){
   GaussianBlur(img,img,Size(3,3),3,3);    
   cvtColor(img,hsv,CV_BGR2HSV);
   inRange(hsv,Scalar(H_L,S_L,V_L),Scalar(H_H,S_H,V_H), dst);
   // 
   cv::erode(dst, dst,structure_element);
   medianBlur(dst,dst,3);
   return dst;
}
std::vector<cv::Point2f> Findballon::getPoint(cv::Mat &img,cv::Mat &src){
   P.clear();
   findContours(dst,contours,hierarchy, RETR_TREE , CV_CHAIN_APPROX_SIMPLE);
   double max_area = 0.0;
   int max_index = -1;
    int indexOfBalloon = 0;
    for(int index = contours.size()-1;index >=0;index--){
        double area = std::fabs(contourArea(contours[index]));
        if(area>max_area){
            max_index = index;
            max_area = area;
        }
    }
    cv::Point2f center;
    float radius;
    if(max_index>=0){
        printf("max_index\n");
        cv::minEnclosingCircle(contours[max_index], center,radius);
        circle(src, center, radius, Scalar(180,60,255),3);
        if(max_area>100)P.push_back(center);
    }

   return P;
}
bool Findballon::GetBoxType(cv::Scalar Color){
	int B= std::abs(Color.val[0]);
	int G=std::abs(Color.val[1]);
	int R=std::abs(Color.val[2]);
	if(B>180&&G>180&&R>180)
    return true;
	else 
    return false;
}
cv::Scalar Findballon::GetColor(cv::Mat imgROI,int size){
	cv::Scalar color(0,0,0);
	int r=std::min(imgROI.cols,imgROI.rows);
	srand(time(NULL));
	for(int i=0;i<size;i++){
		int m=rand()%r-(r/2);
        int n=rand()%r-(r/2);
		for(int i=0;i<3;i++)
			color.val[i]+=imgROI.at<cv::Vec3b>(imgROI.cols/2+m,imgROI.rows/2+n)[i];
	}
	for(int i=0;i<3;i++)
		color.val[i]/=size;
	return color;
}
bool Findballon::scale_test(cv::Mat &img){
    static bool  flag = false;
    static int num_feature =  60;
    static  std::vector<std::string> ids;
    static line2Dup::Detector detector(num_feature, {4, 8});
    R.clear();
    if(!flag){
        ids.push_back("circle");
        ids.push_back("circle1");
        detector.readClasses(ids, "./res/%s_templ.yaml");
        flag = true;
    }
    int stride = 32;
    int n = img.rows/stride;
    int m = img.cols/stride;
    Rect roi(0, 0, stride*m , stride*n);
    cv::Mat matchimg = img(roi).clone();
    auto matches = detector.match(matchimg, 80,ids);
    // std::cout << "matches.size(): " << matches.size() << std::endl;
    size_t top5 = 1;
    bool aa = false;
    if(top5>matches.size()) top5=matches.size();
    for(size_t i=0; i<top5; i++){
        auto match = matches[i];
        auto templ1 = detector.getTemplates("circle",
                                            match.template_id);
        auto templ2 = detector.getTemplates("circle1",
                                            match.template_id);
        double x = -1,y = -1,r = -1;
        if(templ2.size()){
            x =  templ1[0].width/2 + match.x;
            y = templ1[0].height/2 + match.y;
            r = templ1[0].width/2;     
        }else if(templ1.size()){
            x =  templ2[0].width/2 + match.x;
            y = templ2[0].height/2 + match.y;
            r = templ2[0].width/2; 
        }

    }
    if(matches.size()>0)
        return true;
    else
        return false;
    
  
}

