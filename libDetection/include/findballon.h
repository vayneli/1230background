#include <opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;
class Findballon{
public:
	cv::Mat hsv,dst,gray;
	vector<vector<Point> > contours;
	RotatedRect contours_Rect;
	Mat structure_element = getStructuringElement(MORPH_RECT, Size(3,3));
	vector<Vec4i> hierarchy;
	cv::Point2f vertex[4];
	std::vector<cv::Point2f> P;
	std::vector<cv::Point2f> R;
	int H_L=0;
	int S_L=0;
	int V_L=169;
	int H_H=180;
	int S_H=30;
	int V_H=255;
	// cv::Scalar low=(0,0,221);
	// cv::Scalar high=(180,30,255);
    cv::Mat Denoising(cv::Mat &img);
    /**
     * 
     * 
	 * @param:  
	 * @return:　
     *
     */
	std::vector<cv::Point2f> getPoint(cv::Mat &img,cv::Mat &src);
	/**
	 * 
	 * @param:  
	 * @return:　
	 * 
	 */
	bool scale_test(cv::Mat &img);
	/**
	 * 
	 * @param:  
	 * @return:　
	 * 
	 *
	 */

    private:
	bool GetBoxType(cv::Scalar Color);
	/**
	 * 
	 * @param:  
	 * @return:　
	 * 
	 *
	 */
	cv::Scalar GetColor(cv::Mat imgROI,int size=100);
	/**
	 * 
	 * @param:  
	 * @return:　
	 * 
	 *
	 */
	  
};