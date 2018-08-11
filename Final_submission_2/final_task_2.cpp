/******************************
Before running this code make sure you exectued the edge_det.cpp file supplied.
That code generates two image : 1) The original Image 2) Sobel filtered image

This code might take some time to run.
My laptop(regular one) lags when the code is under process
******************************/






#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp" 
#include<iostream>
#include<math.h>
#include<string>
#include<bits/stdc++.h>
using namespace std;
using namespace cv;


vector<vector<vector<Point> > > vote(10000,vector<vector<Point> >(10000));
vector<int> vote_row_sum(10000);
vector<vector<Vec4i> > lines_generated(20),voted_lines(20);
int subsection_count=0;


Mat lane_extraction(Mat road)    // A formality function 
{     
    Mat gray;
    cvtColor(road,gray, CV_BGR2GRAY);
    threshold(gray,gray,0,255,THRESH_BINARY|THRESH_OTSU);       
    return gray;
}



vector<Mat> road_section(Mat road)
{
	vector<Mat> subsections;
	road=lane_extraction(road);
	for(int i=0,k=1;i<road.rows;k++)
	{
		int width_iterative=(k*road.rows)/20,width;
		if(width_iterative+i<road.rows)
			width=width_iterative;
		else
			width=road.rows-i;
		Rect roi = Rect(0,i,road.cols,width);
       	subsections.push_back( road(roi));
       	i+=width;
	}
	return subsections;
}


vector<Mat> road_section_org(Mat road)
{
	vector<Mat> subsections;
	for(int i=0,k=1;i<road.rows;k++)
	{
		int width_iterative=(k*road.rows)/20,width;
		if(width_iterative+i<road.rows)
			width=width_iterative;
		else
			width=road.rows-i;
		Rect roi = Rect(0,i,road.cols,width);
       	subsections.push_back( road(roi));
       	i+=width;
	}
	return subsections;
}


Mat join_subsections(Mat final,Mat section,int section_number)
{
	int initial_coord=(section_number*(section_number+1))*final.rows/40;
	int final_coord=(section_number+1)*final.rows/20;
	for(int i=initial_coord,k=0;k<section.rows&&i<final.rows;i++,k++)
	{
		for(int j=0;j<final.cols;j++)
		{
			final.at<Vec3b>(i,j)[0]=section.at<Vec3b>(k,j)[0];
			final.at<Vec3b>(i,j)[1]=section.at<Vec3b>(k,j)[1];
			final.at<Vec3b>(i,j)[2]=section.at<Vec3b>(k,j)[2];
		}
	}
	return final;
}

int subsection_index_in_vote_as_y=0;


void voting_intersection(int x,int y,int line_1,int line_2,float length_1,float length_2)
{
 	int shift_x=x+5000,shift_y=y+5000;
 	if((shift_y>=0&&shift_x>=0&&shift_y<10000&&shift_x<10000))
 	{
 		Point u;
 		u.x=length_2+length_1;
 		u.y=-1;
 		if(vote[shift_y][shift_x].size())
 		{
 			vote[shift_y][shift_x][0].x++;    // I am not weighing the votes
 			vote[shift_y][shift_x][0].y=-1;
 		}
  		else
 			vote[shift_y][shift_x].push_back(u);
 		u.x=line_1;
 		u.y=subsection_index_in_vote_as_y;
 		vote[shift_y][shift_x].push_back(u);
 		u.x=line_2;
 		vote[shift_y][shift_x].push_back(u);
 		vote_row_sum[shift_y]+=length_1+length_2;
 	}
}


int vote_count()
{
	cout<<"Counting the votes"<<endl;
	int max_vote=0,sum_three_rows=0,max_vote_row_index=0;
	for(int i=0;i+4<10000;i++)
	{
		sum_three_rows=vote_row_sum[i]+vote_row_sum[i+1]+vote_row_sum[i+2];               
		if(max_vote<sum_three_rows)
		{
			max_vote=sum_three_rows;
			max_vote_row_index=i;
		}
	}
	cout<<"max_vote_row_index "<<max_vote_row_index<<endl;
	return max_vote_row_index;
}	


void lines_intersection(vector<Vec4i> lines)
{
	cout<<"Voting in progress"<<endl;
	int x_intersection,y_intersection,count=0;
	float slope_line_i=0,slope_line_j=0,length_i,length_j;
	for(size_t i=0;i<lines.size();i++)
	{
		slope_line_i=(lines[i][3]-lines[i][1])/(float)(lines[i][2]-lines[i][0]);
		length_i=sqrt(pow(lines[i][3]-lines[i][1],2)+pow(lines[i][2]-lines[i][0],2));
		for(size_t j=i+1;j<lines.size();j++)
		{
			slope_line_j=(lines[j][3]-lines[j][1])/(float)(lines[j][2]-lines[j][0]);
			length_j=sqrt(pow(lines[j][3]-lines[j][1],2)+pow(lines[j][2]-lines[j][0],2));
			x_intersection=(lines[j][1]-lines[i][1]+slope_line_i*lines[i][0]-slope_line_j*lines[j][0])/(slope_line_i-slope_line_j);
			y_intersection=(slope_line_i*slope_line_j*(lines[i][0]-lines[j][0])+slope_line_i*lines[j][1]-slope_line_j*lines[i][1])/(slope_line_i-slope_line_j);
			voting_intersection(x_intersection,y_intersection,i,j,length_i,length_j);
		}
	}
	cout<<"Done"<<endl;
}


void get_lanes_after_vote(int horizon)
{
	cout<<"Getting the lanes"<<endl;
	int index_line;
	for(int j=horizon;j<=horizon+2;j++)
	{
		for(int column=0;column<10000;column++)
		{
			subsection_count=0;
			for(size_t i=1;i<vote[j][column].size();i++)
			{
				index_line=vote[j][column][i].x;
				subsection_count=vote[j][column][i].y;

				if(find(voted_lines[subsection_count].begin(), voted_lines[subsection_count].end(),
					lines_generated[subsection_count][index_line])!= voted_lines[subsection_count].end()) continue;

				if(abs((lines_generated[subsection_count][index_line][3]-lines_generated[subsection_count][index_line][1])
					/(float)(lines_generated[subsection_count][index_line][2]-lines_generated[subsection_count][index_line][0]))<0.2f) continue;

				voted_lines[subsection_count].push_back(lines_generated[subsection_count][index_line]);
			}
		}
	}
	cout<<"Done";
}


void hough_transform_probabilistic(Mat road)
{
	cout<<"Detecting the lines"<<endl;
	int horizon_row_index;
	vector<Mat> road_sections,road_sections_org;
	road_sections=road_section(road);
	cout<<"Divided into subsections"<<endl;
    Mat road_lanes_detected=Mat::zeros(road.size(),CV_8UC3);
    for(int i=0;i<road_sections.size();i++)
    {
    	Mat road_org;
    	cvtColor(road_sections[i],road_org, CV_GRAY2BGR );
    	HoughLinesP(road_sections[i], lines_generated[i], 2, CV_PI/180, 50, 20,30 );
    	cout<<"Finding the line intersections"<<endl;
    	lines_intersection(lines_generated[i]);
    	subsection_index_in_vote_as_y++;
    }
   	horizon_row_index=vote_count();
   	get_lanes_after_vote(horizon_row_index);
   	road_sections_org=road_section_org(imread("original.jpeg",1));               // Here I am reading in the original image created by edge_det.cpp
   	for(int i=0;i<road_sections_org.size();i++)
   	{
   		Mat road_org_l=road_sections_org[i].clone();
   		for( size_t j = 0; j < voted_lines[i].size(); j++ )
    		{
        		line( road_org_l, Point(voted_lines[i][j][0],voted_lines[i][j][1]),
            		Point(voted_lines[i][j][2],voted_lines[i][j][3]), Scalar(0,0,255), 2, 8 );
    		}
   		road_lanes_detected=join_subsections(road_lanes_detected,road_org_l,i);
    	}
   	namedWindow("Detected Lines", WINDOW_NORMAL );
  	imshow("Detected Lines", road_lanes_detected );
	imwrite("Output_image.png",road_lanes_detected);
   	waitKey(0);
}


int main(int argc, char** argv)
{
    Mat road;
  	road=imread("sobel.jpeg",1);                                                     //Here I am reading the sobel image created by edge_det.cpp
  	cout<<"Starting"<<endl;
    hough_transform_probabilistic(road);
    waitKey(0);
    return 0;
}
