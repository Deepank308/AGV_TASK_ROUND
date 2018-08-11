#include<iostream>
#include<fstream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
int main()
{
	MatrixXd R(2,2);
	R << 0.1,   0,
	       0, 0.1;
	MatrixXd H(2,4);
	H << 1, 0, 0, 0,
	     0, 1, 0, 0;
	float timestep;
	cout<< "Enter the time step:"<<endl;
	cin>> timestep;
	MatrixXd F(4,4);
	F << 1, 0, timestep,        0,
	     0, 1,        0, timestep,
	     0, 0,        1,        0,
	     0, 0,        0,        1;
	float x_initial,y_initial,vx_initial=0,vy_initial=0;
	cout<<"Enter the initial positions of bot (x,y):"<<endl;
	cin>> x_initial;
	cin>> y_initial;
	//cout<<"e1"<<endl;
	MatrixXd X(4,1);
	X << 	 x_initial,
		 y_initial,
		 vx_initial, 
		 vy_initial;
		// cout<<"e2"<<endl;
	MatrixXd P(4,4);
	P << 1000,    0,     0,    0,
	        0, 1000,     0,    0,
		0,     0, 1000,    0,
		0,     0,    0, 1000;
		 //cout<<"e"<<endl;
	cout<<endl;
	cout<<"The Layout of matrix is : "<<endl;
	cout<<"The positions and velocity matrix : "<<endl;
	cout<<"px"<<endl<<"py"<<endl<<"vx"<<endl<<"vy"<<endl;
	fstream ifs;
	ifs.open("landmarks.txt",ios::in);

    while(1)
    {
    	float x_gps=-1,y_gps=-1;
    	char c;
    	ifs>>x_gps;
    	ifs>>c;
    	ifs>>y_gps;
    	//ifs>>"\n";
    	if(x_gps==-1||y_gps==-1) break;
	cout<<endl;
	cout<<"At "<<"X="<<x_gps<<" Y="<<y_gps<<endl;
    	MatrixXd Z(2,1);
    	Z << x_gps,
	     y_gps;
    	X=F*X;
    	P=F*P*(F.transpose());
    	MatrixXd K(4,2);
    	MatrixXd to_inverse(2,2);
    	to_inverse=H*P*(H.transpose())+R;
    	K=P*(H.transpose())*(to_inverse.inverse());
    	X=X+K*(Z-H*X);
    	P=P-K*H*P;
    	cout<<"The positions and velocity matrix:"<<endl;
    	cout<<X<<endl<<endl;
    	cout<<"The uncertainity matrix:"<<endl;
    	cout<<P<<endl<<endl<<endl;
    }

}
