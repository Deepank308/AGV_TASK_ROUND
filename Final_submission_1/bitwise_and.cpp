#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp" 
#include<iostream>
#include<math.h>
#include<queue>
#include<vector>
#include<bits/stdc++.h>
using namespace std;
using namespace cv;

Mat image1,image2,image3,image4;

int main()
{
	Mat restricted;
	image1=imread("0.png",0);
	//threshold(image1,image1,0,255,THRESH_BINARY|THRESH_OTSU);
	restricted=image1.clone();
	bitwise_and(restricted,image1,restricted);
	image2=imread("45.png",0);
	threshold(image2,image2,0,255,THRESH_BINARY|THRESH_OTSU);
	bitwise_and(restricted,image2,restricted);
	image3=imread("90.png",0);
	threshold(image3,image3,0,255,THRESH_BINARY|THRESH_OTSU);
	bitwise_and(restricted,image3,restricted);
	image4=imread("135.png",0);
	threshold(image4,image4,0,255,THRESH_BINARY|THRESH_OTSU);
	bitwise_and(restricted,image4,restricted);
	Mat element = getStructuringElement(MORPH_RECT,Size(2*1+1,2*1+1),Point(-1,-1));
    dilate(restricted,restricted,element);
    erode(restricted,restricted,element);
	imwrite("restricted.png",restricted);
	imshow("restricted",restricted);
	waitKey(0);
}