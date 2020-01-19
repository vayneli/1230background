#include "ACsaliencydetect.h"
//#include "robot_model.h"

AC_saliency::AC_saliency(){

}
AC_saliency::~AC_saliency(){

}


Mat AC_saliency::saliencyBasedonAC(Mat& src,int MinR2, int MaxR2,int Scale){
    Mat Lab;
	cvtColor(src, Lab, CV_BGR2Lab); 
 
	int row=src.rows,col=src.cols;
	int Sal_org[row][col];
	memset(Sal_org,0,sizeof(Sal_org));  
    //void *memset(void *s,int c,size_t n)
    //总的作用：将已开辟内存空间 s 的首 n 个字节的值设为值 c
	
	Mat Sal=Mat::zeros(src.size(),CV_8UC1 );
 
	Point3_<uchar>* p;
	Point3_<uchar>* p1;
	int val;
	Mat filter;
 
	int max_v=0;
	int min_v=1<<28;  //为啥左移28位？
	for (int k=0;k<Scale;k++){
		int len=(MaxR2 - MinR2) * k / (Scale - 1) + MinR2;
		blur(Lab, filter, Size(len,len ));
		for (int i=0;i<row;i++){
			for (int j=0;j<col;j++){
				p=Lab.ptr<Point3_<uchar> > (i,j);
				p1=filter.ptr<Point3_<uchar> > (i,j);
				//cout<<(p->x - p1->x)*(p->x - p1->x)+ (p->y - p1->y)*(p->y-p1->y) + (p->z - p1->z)*(p->z - p1->z) <<" ";
				
				val=sqrt( (p->x - p1->x)*(p->x - p1->x)+ (p->y - p1->y)*(p->y-p1->y) + (p->z - p1->z)*(p->z - p1->z) );
				Sal_org[i][j]+=val;
				if(k==Scale-1){
					max_v=max(max_v,Sal_org[i][j]);
					min_v=min(min_v,Sal_org[i][j]);
				}
			}
		}
}
	
	//cout<<max_v<<" "<<min_v<<endl;
	int X,Y;
    for (Y = 0; Y < row; Y++)
    {
        for (X = 0; X < col; X++)
        {
            Sal.at<uchar>(Y,X) = (Sal_org[Y][X] - min_v)*255/(max_v - min_v);        //    计算全图每个像素的显著性
        	//Sal.at<uchar>(Y,X) = (Dist[gray[Y][X]])*255/(max_gray);        //    计算全图每个像素的显著性
        }
    }
    return Sal;
}

Point3f AC_saliency::getXY(Mat& img,Mat& depth){
    Rect roi;
    Point3f p;
    roi.x = 0;
    roi.y = 100;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarcy;
    Mat result;
    //获得帧率
    //int count = 1;
    vector<Rect> unknown;
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
        Rect temp;
        if(img.empty()) return Point3f(0,0,0);
        roi.width = img.cols;
        //cout<<img.cols<<endl;
        //cout<<img.rows<<endl;
        roi.height = 300;
        Mat src = img(roi);
        //result = SalientRegionDetectionBasedonLC(src);
        //result = SalientRegionDetectionBasedonFT(src);
        result=saliencyBasedonAC(src,src.rows/8,src.rows/2,3);
        threshold(result, result, 40, 255, THRESH_BINARY);
        morphologyEx(result,result,MORPH_CLOSE,element);
        findContours(result, contours, hierarcy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        for(size_t i=0;i<contours.size();i++){
            if(contours[i].size()>5){
                temp = boundingRect(contours[i]);
                temp.y+=roi.y;
                if(temp.area()>2000) continue;
                if(temp.area()<200) continue;
                //rectangle(img,temp,Scalar(255,255,0));
                p.x=int(temp.y+temp.height/2);
                p.y=int(temp.x+temp.width/2);
                p.z=depth.at<ushort>(p.x,p.y);
            }
        }
        return p;
}

Point3f AC_saliency::getC_xyz(Point3f& point_p,CAMERA_INRINSIC_PARAMETERS_A& camera){

    Point3f pc;
    pc.z=point_p.z/camera.scale;
    pc.x=(point_p.x-camera.cy)*pc.z/camera.fy;
    pc.y=(point_p.y-camera.cx)*pc.z/camera.fx;

    return pc;

}

