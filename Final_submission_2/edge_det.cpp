/************************
This code uses sobel derivatives to get the edge detected out of the original image.
So plugin in the origianl image here and execute it, then directly run the final_task_2.cpp file.
************************/



#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp" 
#include<iostream>
#include<math.h>
using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
	Mat b;
	Mat source=imread(argv[1],1);
	cvtColor(source,b,CV_BGR2GRAY);
	Mat a(b.rows,b.cols,CV_8UC1,Scalar(0));
	Mat c=Mat::zeros(b.size(),CV_8UC1);
	Mat d=Mat::zeros(b.size(),CV_8UC1);
	namedWindow("Edge_Detection",WINDOW_AUTOSIZE);
	int s=1,w=1,t=1;
	createTrackbar("Edge_Refinement_X","Edge_Detection",&s,255);
	createTrackbar("Edge_Refinement_Y","Edge_Detection",&w,255);
	createTrackbar("Edge_Refinement_Equivalent","Edge_Detection",&t,255);
	while(1)
	{
		for(int i=1;i<b.rows-1;i++)
		{
			for(int j=1;j<b.cols-1;j++)
			{	
				int p=((b.at<uchar>(i+1,j)+2*b.at<uchar>(i,j+1)+b.at<uchar>(i+1,j+1))-(b.at<uchar>(i-1,j-1)+2*b.at<uchar>(i,j-1)+b.at<uchar>(i+1,j-1)))/4;
				int q=((b.at<uchar>(i+1,j+1)+2*b.at<uchar>(i+1,j)+b.at<uchar>(i+1,j-1))-(b.at<uchar>(i-1,j-1)+2*b.at<uchar>(i-1,j)+b.at<uchar>(i-1,j+1)))/4;
				int r=sqrt(p*p+q*q);
				if(abs(p)>=s&&abs(q)>=w&&abs(r)>=t)
					a.at<uchar>(i,j)=abs(r);
				else
					a.at<uchar>(i,j)=0;
			}
		}
	imshow("Edge_Detection",a);
	imwrite("original.jpeg",source);
	imwrite("sobel.jpeg",a);
	waitKey(50);
	}
}

