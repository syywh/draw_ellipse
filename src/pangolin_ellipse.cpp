#include <pangolin/pangolin.h>
#include <thread>
#include <ros/ros.h>

void run();

int main(int argc, char** argv){
	
  	ros::init(argc, argv, "pangolin_ellipse");
	ros::start();
	ros::NodeHandle nh;
	
	std::thread* viewer;
	viewer = new std::thread(&run);
	
	ros::Rate r(1);
	while(ros::ok()){
		std::cout << "22" << std::endl;
		r.sleep();
	}
	
}

void run()
{
	ros::NodeHandle n;
	ros::Rate rr(1);
	while(ros::ok()){
		std::cout << "11" << std::endl;
		rr.sleep();
	}
}
