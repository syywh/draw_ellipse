// 高斯分布测试

#include <iostream>
#include <string>
#include <random>
#include <boost/math/distributions/normal.hpp>
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gaussian");
	ros::start();

	ros::NodeHandle nh;
	
  const int nrolls=10000;  // number of experiments
  const int nstars=100;    // maximum number of stars to distribute

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(3.0,2.0);

  int p[10]={};

  for (int i=0; i<nrolls; ++i) {
    double number = distribution(generator);
    if ((number>=0.0)&&(number<10.0)) ++p[int(number)];
  }

  std::cout << "normal_distribution (5.0,2.0):" << std::endl;

  for (int i=0; i<10; ++i) {
    std::cout << i << "-" << (i+1) << ": ";
    std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
  }
  
  ros::Time t1= ros::Time::now();
  cerr<<t1<<endl;
  boost::math::normal_distribution<double> distribution_boost(0,sqrt(5)); //均值，sigma
  cerr<<distribution_boost.mean()<<"  "<<distribution_boost.scale()<<endl;
  
  cerr<<boost::math::pdf<boost::math::normal_distribution<double>, double>(distribution_boost, 0.0)<<endl;
  ros::Time t2= ros::Time::now();
  cerr<<t2<<endl;
  


  return 0;
}