#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp" 
#include<iostream>
#include<math.h>
#include<queue>
#include<vector>
#include<bits/stdc++.h>
#include<string>
using namespace std;
using namespace cv;

Mat final,source;
int angle;

int IsValid(int n_x,int n_y,int p_x,int p_y)
	{
		if(n_x>=final.cols||n_x<0||n_y<0||n_y>=final.rows||(n_x==p_x&&n_y==p_y))
			return 0;
		else 
			return 1;
	}

void rotate_bot(int x,int y,int theta)
{
RotatedRect rRect = RotatedRect(Point(x,y), Size2f(20,10),180-theta);

Point2f vertices[4];
rRect.points(vertices);
for (int i = 0; i < 4; i++)
    line(final, vertices[i], vertices[(i+1)%4], Scalar(255,255,255));

//Rect brect = rRect.boundingRect();
//rectangle(final, brect, Scalar(255,0,0));
namedWindow("Instant",WINDOW_NORMAL);
imshow("Instant",final);
waitKey(1);
}


void config(Mat source)
{
	int done;
	for(int y=0;y<source.rows;y++)
	{
		for(int x=0;x<source.cols;x++)
		{
			done=0;
			for(int neigh_x=x-1;neigh_x<=x+1;neigh_x++)
			{
				//if(done) continue;
				for(int neigh_y=y-1;neigh_y<=y+1;neigh_y++)
				{
					if(abs(neigh_y-y)+abs(neigh_x-x)!=1) continue;
					if(!IsValid(neigh_x,neigh_y,x,y)) continue;
					if(IsValid(2*x-neigh_x,2*y-neigh_y,x,y))
					{
						done=1;
						if(source.at<Vec3b>(neigh_y,neigh_x)[0]>200&&source.at<Vec3b>(2*y-neigh_y,2*x-neigh_x)[0]<200&&source.at<Vec3b>(y,x)[0]<200)
							rotate_bot(x,y,angle);
							//bot_movement(angle_func(Point(x,y),Point(neigh_x,neigh_y)),Point(x,y));
					}
					else
					{	done=1;
						if(source.at<Vec3b>(neigh_y,neigh_x)[0]>200&&source.at<Vec3b>(y,x)[0]<200)
							rotate_bot(x,y,angle);
							//bot_movement(angle_func(Point(x,y),Point(neigh_x,neigh_y)),Point(x,y));
					}


				}
			}
			if(done) continue;
			for(int neigh_x=x-1;neigh_x<=x+1;neigh_x++)
			{
				for(int neigh_y=y-1;neigh_y<=y+1;neigh_y++)
				{
					if(abs(neigh_y-y)+abs(neigh_x-x)!=2) continue;
					if(!IsValid(neigh_x,neigh_y,x,y)) continue;
					if(IsValid(2*x-neigh_x,2*y-neigh_y,x,y))
					{
						if(source.at<Vec3b>(neigh_y,neigh_x)[0]>230&&source.at<Vec3b>(2*y-neigh_y,2*x-neigh_x)[0]==0&&source.at<Vec3b>(y,x)[0]<200)
							rotate_bot(x,y,angle);
							//bot_movement(angle_func(Point(x,y),Point(neigh_x,neigh_y)),Point(x,y));
					}
					else
					{
						if(source.at<Vec3b>(neigh_y,neigh_x)[0]>230&&source.at<Vec3b>(y,x)[0]<200)
							rotate_bot(x,y,angle);
							//bot_movement(angle_func(Point(x,y),Point(neigh_x,neigh_y)),Point(x,y));
					}


				}
			}
		}
	}
}

int main(int argc, char** argv)
{
	source=imread(argv[1],1);
	angle=atoi(argv[2]);
	final=source.clone();
	config(source);
	imwrite(to_string(angle)+".png",final);
	return 0;
}
