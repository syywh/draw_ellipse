#include "ros/ros.h"
#include<ros/package.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include <cpu_tsdf/tsdf_volume_octree.h>
#include <cpu_tsdf/marching_cubes_tsdf_octree.h>

#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "pointmatcher_ros/point_cloud.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;

using namespace std;
int main (int argc, char** argv)
{
  	ros::init(argc, argv, "vtk2ply");
	ros::start();
	
	ros::NodeHandle nh;
	
	string input_filter_file = ros::package::getPath("kitti")+"/cfg/voxel.yaml";
	ifstream input_file(input_filter_file);
	PM::DataPointsFilters input_filter;
	if(input_file.is_open()){
		cerr<<"load input_filter from "<<input_filter_file<<endl;
		input_filter = PM::DataPointsFilters(input_file);
		
	}
	
	string name = argv[1];
	DP data_l = DP::load(name);
// 	DP data = data_l.createSimilarEmpty();	int cnt = 0;
	
// 	for(int  i = 0; i<data_l.features.cols(); i++){
// 		if(data_l.features.col(i).head(3).norm() < 35){
// 			data.setColFrom(cnt++, data_l, i);
// 		}
// 	}
// 	data.conservativeResize(cnt);
// 	cerr<<data.features.cols()<<endl;
	input_filter.apply(data_l);
	cerr<<data_l.features.cols()<<endl;
	data_l.save(name+".ply");
}