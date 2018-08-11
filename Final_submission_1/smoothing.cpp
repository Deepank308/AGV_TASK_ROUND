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

Point dest;
struct for_queue
{
	int x,y;
	int dist;
};

typedef bool (*comp) (for_queue,for_queue);
 
struct parent_child
{
	int x,y;
	int index_parent;
};

int heur_euler(for_queue a)
	{return sqrt(pow(abs(a.x-dest.x),2)+pow(abs(a.y-dest.y),2))+a.dist;}

int heur_manhattan(for_queue a)
	{return abs(a.x-dest.x)+abs(a.y-dest.y)+a.dist;}

bool compare(for_queue a,for_queue b)
{
	if(heur_euler(a)>heur_euler(b))
		return heur_euler(a)>heur_euler(b);
	if(heur_manhattan(a)>heur_manhattan(b))
		return heur_manhattan(a)>heur_manhattan(b);
	return 0;
}

class PathPlanner
{
 private:
 	struct return_data
 	{
 		float length;
 		int total_points;
 	};
	Point start_org,dest_org;
	Mat obstacle_map;
	Mat region_org,region_for_bot_movement,refined_path;
	int points_traversed;
	float path_length;
	vector<Point> final_path;
	vector<Point> final_path_reverse;

	vector<for_queue> neigh_update_dist;

	parent_child element;
	vector<parent_child> Parent_Child;

	int isValid(int y,int x,int r,int c)
	{
		if(x>=c||x<0||y<0||y>=r)
			return 0;
		else
			return 1;
	}

	int neigh_dist_update(int x,int y)
	{
		int min=10000;
		for(int i=0;i<neigh_update_dist.size();i++)
		{
			if(neigh_update_dist[i].x==x&&neigh_update_dist[i].y==y)
				if(min>neigh_update_dist[i].dist)
					min=neigh_update_dist[i].dist;
		}
		return min;
	}

	int check_for_parent(int x,int y)
	{
		for(int i=0;i<Parent_Child.size();i++)
		{
			if(Parent_Child[i].x==x&&Parent_Child[i].y==y)
				return i;
		}
		return -1;
	}
	void inflated_erosion()
 	{
 		int i=10;
    	Mat element = getStructuringElement(MORPH_RECT,Size(2*1+1,2*1+1),Point(-1,-1));
    	while(i--)
        	dilate(obstacle_map,obstacle_map,element);
 	}


	int angle_func(Point current,Point target)
	{
    	float path_angle; 
    	if (target.x-current.x==0)
        	if (target.y<current.y)//finding initial path angle
            	path_angle = 3.14159 / 2;
          	else
            	path_angle = -3.14159 / 2;
        else
            path_angle = -atan((target.y - current.y)*1.0 / (target.x - current.x));//finding the path angle

        if (target.x - current.x >= 0)
        {
            if (path_angle<0)
                path_angle += 2 * 3.14159;
        }
        else
            path_angle += 3.14159;//as atan returns -ve value

        path_angle *= 180 / 3.14159;
        if(path_angle>=180)
        	return path_angle-180;
        return (int)path_angle;
	}
	void bot_movement(int theta,Point current)
	{
		Mat orientation=region_for_bot_movement.clone();
		Mat bot(30,30,CV_8UC3,Scalar(0,0,0));
		for(int i=10;i<20;i++)
		{
			for(int j=5;j<25;j++)
			{
				bot.at<Vec3b>(j,i)[0]=255;
			//bot.at<Vec3b>(j,i)[1]=255;
			//bot.at<Vec3b>(j,i)[2]=255;
			}
		}
		Mat M=getRotationMatrix2D(Point(bot.cols/2-1,bot.rows/2-1),theta-90,1);
    	Mat dst;
    	warpAffine(bot,dst,M, Size(30,30));
    	for(int i=0,k=current.y-15;i<30;i++,k++)
    	{
    		for(int j=0,l=current.x-15;j<30;j++,l++)
    		{
    			if(orientation.at<Vec3b>(k,l)[0]<50&&orientation.at<Vec3b>(k,l)[1]<50&&orientation.at<Vec3b>(k,l)[2]<50)
    				orientation.at<Vec3b>(k,l)=dst.at<Vec3b>(i,j);
    		}
    	}
   		namedWindow("Final movement",WINDOW_AUTOSIZE);
    	imshow("Final movement",orientation);
    	waitKey(50);
	}

	void orientation_consideration()
	{
		for(int i=0;i<final_path_reverse.size()-1;i++)
			bot_movement(angle_func(final_path_reverse[i],final_path_reverse[i+1]),final_path_reverse[i]);	
	}


	void path_refinement()
	{
		//cout<<"E2"<<endl;
		//cout<<"final_path.size() "<<final_path.size()<<endl;
		
		for(int i=final_path.size()-1;i>-1;i--)
		{
			final_path_reverse.push_back(final_path[i]);
		}
    	for(int i=0;i<final_path_reverse.size()-1;i++)
    	{
    		//cout<<"E4"<<endl;
        	if(final_path_reverse[i].x==0||final_path_reverse[i+1].x==0||final_path_reverse[i+2].x==0) continue;
        	int bot_angle=angle_func(final_path_reverse[i+1],final_path_reverse[i]);
        	int dest_angle=angle_func(final_path_reverse[i+1],final_path_reverse[i+2]);
        	int dist_pixel=abs(final_path_reverse[i].x-final_path_reverse[i+1].x)+abs(final_path_reverse[i].y-final_path_reverse[i+1].y);
        	//cout<<dist_pixel<<endl;
        	//cout<<"E5"<<endl;
        	//cout<<"Bot "<<bot_angle<<" "<<"Dest "<<dest_angle<<endl;
        	if(abs(bot_angle-dest_angle)>=27/*||(abs(bot_angle-dest_angle)>=135&&abs(bot_angle-dest_angle)<180))*/&&dist_pixel<=3)
        	{
        	//	cout<<"E6"<<endl;
            	final_path_reverse.erase(final_path_reverse.begin()+i+1);
            	i--;
            	//cout<<i<<endl;
        	}
    	}
    	//cout<<"E3"<<endl;
	}


	void make_refined_path()
	{
		//cout<<"E"<<endl;
		path_refinement();
		//cout<<"E1"<<endl;
		for(int i=0;i<final_path_reverse.size()-1;i++)
		{
			refined_path.at<Vec3b>(final_path_reverse[i].y,final_path_reverse[i].x)[0]=255;
		}
		namedWindow("Refined",WINDOW_NORMAL);	
		imshow("Refined",refined_path);
		waitKey(100);
		orientation_consideration();
	}

	void make_path()
	{
		int p=0,flag=1;
		final_path.push_back(dest_org);
		while(flag)
		{
			for(int i=0;i<Parent_Child.size();i++)
			{
				if(Parent_Child[i].x==final_path[p].x&&Parent_Child[i].y==final_path[p].y)
				{
					p++;
					Point s;
					s.x=Parent_Child[Parent_Child[i].index_parent].x;
					s.y=Parent_Child[Parent_Child[i].index_parent].y;
					final_path.push_back(s);
					path_length+=abs(s.x-final_path[p-1].x)+abs(s.y-final_path[p-1].y);
					region_org.at<Vec3b>(final_path[p-1].y,final_path[p-1].x)[0]=255;
					if(Parent_Child[Parent_Child[i].index_parent].x==start_org.x&&Parent_Child[Parent_Child[i].index_parent].y==start_org.y)
					{
						flag=0;
						break;
					}
					else
						continue;
				}
			}
		}
		make_refined_path();
		//orientation_consideration();
		region_org.at<Vec3b>(start_org.y,start_org.x)[0]=255;
		namedWindow("Path",WINDOW_NORMAL);
		imshow("Path",region_org);
		waitKey(0);
	}
 public:

	PathPlanner(Point start,Point dest,Mat region)
	{
		start_org.x=start.x;
		start_org.y=start.y;
		dest_org.x=dest.x;
		dest_org.y=dest.y;
		points_traversed=0;
		path_length=0;
		obstacle_map=region.clone();
		inflated_erosion();
		region_org=region.clone();
		refined_path=region.clone();
		region_for_bot_movement=region.clone();
	}

	return_data return_parameters()
	{
		return_data temp;
		temp.length=path_length;
		temp.total_points=points_traversed;
		return temp;
	}

	void get_path()
	{
 		int flag=1;
		element.x=start_org.x;
		element.y=start_org.y;
		element.index_parent=0;
		Parent_Child.push_back(element);
		element.index_parent=-1;
		for_queue start_queue;
		start_queue.x=start_org.x;
		start_queue.y=start_org.y;
		start_queue.dist=0;
		priority_queue<for_queue,vector<for_queue>,comp> queue_nodes{compare};
		queue_nodes.push(start_queue);
		while((!(queue_nodes.empty())&&flag))
		{
			for_queue u=queue_nodes.top();
			queue_nodes.pop();
			if(obstacle_map.at<Vec3b>(u.y,u.x)[0]==255)
				continue;
			obstacle_map.at<Vec3b>(u.y,u.x)[0]=255;  // visited
			if(u.y==dest_org.y&&u.x==dest_org.x)
			{
				flag=0;
				make_path();
				break;
			}
			points_traversed++;
			//namedWindow("instant",	WINDOW_NORMAL);
			//imshow("instant",obstacle_map);
			//waitKey(1);
			for(int i=u.y-1;i<=u.y+1;i++)
			{
				for(int j=u.x-1;j<=u.x+1;j++)
				{
					if(!isValid(i,j,obstacle_map.rows,obstacle_map.cols)) continue;  // within the image
					if(obstacle_map.at<Vec3b>(i,j)[0]>200) continue;  // already visited or obstacle
				//if(sqrt(pow(abs(dest.x-u.x),2)+pow(abs(dest.y-u.y),2))<sqrt(pow(abs(dest.x-j),2)+pow(abs(dest.y-i),2))) continue;
					for_queue neigh;
					neigh.x=j;
					neigh.y=i;
					for_queue current;
					current.x=u.x;
					current.y=u.y;
					current.dist=u.dist;
					neigh.dist=neigh_dist_update(neigh.x,neigh.y);
					if(neigh.dist>current.dist+sqrt(pow(abs(current.x-neigh.x),2)+pow(abs(current.y-neigh.y),2)))
					{
						neigh.dist=current.dist+sqrt(pow(abs(current.x-neigh.x),2)+pow(abs(current.y-neigh.y),2));
						neigh_update_dist.push_back(neigh);
						queue_nodes.push(neigh);
						element.x=neigh.x;
						element.y=neigh.y;
						element.index_parent=check_for_parent(current.x,current.y);
						if(check_for_parent(element.x,element.y)!=-1)
							Parent_Child[check_for_parent(element.x,element.y)].index_parent=check_for_parent(current.x,current.y);	
						else
							Parent_Child.push_back(element);
					}
					/*if(i==dest_org.y&&j==dest_org.x)
					{
						flag=0;
						make_path();
						break;
					}*/
				}
				if(!flag) break;
			}
		}	
	}
};


Point contour_center(Mat thresholded)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Point center;
    findContours(thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    Mat drawing = Mat::zeros(thresholded.size(), CV_8UC3);
    for( int i = 0; i< contours.size(); i++ )
    {
    	if(contourArea(contours[i])>10000) continue;
        drawContours( drawing, contours, i,Scalar(255,255,255), 2, 8, hierarchy, 0, Point() );
	Rect rct=boundingRect(Mat(contours[i]));
        center.x=(rct.tl().x+rct.br().x)/2;
        center.y=(rct.tl().y+rct.br().y)/2;
        drawing.at<Vec3b>(center.y,center.x)=255;
    }; 
	return center;
}


Point start_contour(Mat region)
{
	Mat region_ext=Mat::zeros(region.size(),CV_8UC1);
	for(int i=0;i<region.rows;i++)
	{
		for(int j=0;j<region.cols;j++)
		{
			if(region.at<Vec3b>(i,j)[1]>175&&region.at<Vec3b>(i,j)[0]<100)
				region_ext.at<uchar>(i,j)=255;
			else
				region_ext.at<uchar>(i,j)=0;
		}
	}
	return contour_center(region_ext);
}


Point dest_contour(Mat region)
{
	Mat region_ext=Mat::zeros(region.size(),CV_8UC1);
	for(int i=0;i<region.rows;i++)
	{
		for(int j=0;j<region.cols;j++)
		{
			if(region.at<Vec3b>(i,j)[2]>200&&region.at<Vec3b>(i,j)[0]<50)
				region_ext.at<uchar>(i,j)=255;
			else
				region_ext.at<uchar>(i,j)=0;
		}
	}
	return contour_center(region_ext);
}

int main(int argc,char ** argv)
{
	Mat region=imread(argv[1],1);
	Point start=start_contour(region);
	dest=dest_contour(region);
	PathPlanner A_Path(start,dest,region);
	A_Path.get_path();
	cout<<"Points Traversed : "<<A_Path.return_parameters().total_points<<endl;
	cout<<"Length of Path : "<<A_Path.return_parameters().length<<" pixels"<<endl;
}














































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































