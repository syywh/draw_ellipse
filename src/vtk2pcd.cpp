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

namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;
int main (int argc, char** argv)
{
  	ros::init(argc, argv, "vtk2pcd");
	ros::start();
	ros::NodeHandle nh;
	ros::Publisher pcPublisher;
	pcPublisher = nh.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
	
	bpo::options_description opts_desc("Allowed options");
	bpo::positional_options_description p;
	
	
	opts_desc.add_options()
	("help,h", "produce help message");
	
// 	string readin = argv[1];

	string vtkfile = ros::package::getPath("mesh")+"/data/"+"DataPoints"+".vtk";
// 	string save = ros::package::getPath("mesh")+"/data/"+readin+".pcd";
	cerr<<vtkfile<<endl;
	DP clouddp  = DP::load(vtkfile);
	sensor_msgs::PointCloud2 sensorPD = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(clouddp, "/map", ros::Time::now());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(sensorPD, *cloud);
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGBA>(cloud->width, cloud->height));
	pcl::copyPointCloud(*cloud, *cloud_rgb);
	
	for (size_t j = 0; j < cloud->size (); j++)
	{
		pcl::PointXYZRGBA &pt = cloud_rgb->at (j);
		pt.r = 1;
		pt.g = 1;
		pt.b = 1;
		pt.a = 1;
	}
	
	// Begin Integration
	float tsdf_size = 12.;
	float cell_size = 0.006;
	int num_random_splits = 1;
	float min_sensor_dist = 0;
	float max_sensor_dist = 30.0;
	float trunc_dist_pos = 0.03;
	float trunc_dist_neg = 0.03;
	
	int tsdf_res;
	
	int desired_res = tsdf_size / cell_size;//格子分辨率
	int n = 1;
	while (desired_res > n)//2的倍数
	{
		n *= 2;
	}
	tsdf_res = n;
	
	cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;
	tsdf.reset (new cpu_tsdf::TSDFVolumeOctree);
	tsdf->setGridSize (tsdf_size, tsdf_size, tsdf_size);
	tsdf->setResolution (tsdf_res, tsdf_res, tsdf_res);
	tsdf->setNumRandomSplts (num_random_splits);
	tsdf->setSensorDistanceBounds (min_sensor_dist, max_sensor_dist);
// 	tsdf->setColorMode("gray");
	tsdf->setIntegrateColor (false);
	
	tsdf->setDepthTruncationLimits (trunc_dist_pos, trunc_dist_neg);
	tsdf->reset ();
	
	cerr<<"reset ok"<<endl;
	cerr<<"cloud size "<<cloud_rgb->size()<<endl;
	pcl::PointCloud<pcl::Normal> cloud_normal;
	tsdf->integrateCloud (*cloud_rgb, cloud_normal);

}