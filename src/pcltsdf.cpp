#include <iostream>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include <pcl/gpu/kinfu/tsdf_volume.h>
#include <pcl/gpu/kinfu_large_scale/device.h>
#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>
#include "/home/dxq/library/pcl-pcl-1.8.0/gpu/kinfu_large_scale/include/pcl/gpu/kinfu_large_scale/impl/standalone_marching_cubes.hpp"
#include <pcl/gpu/kinfu_large_scale/world_model.h>
#include "/home/dxq/library/pcl-pcl-1.8.0/gpu/kinfu_large_scale/include/pcl/gpu/kinfu_large_scale/impl/world_model.hpp"
#include <pcl/gpu/kinfu_large_scale/standalone_marching_cubes.h>



// #include <pcl/gpu/kinfu_large_scale/kinfu.h>
// #include <pcl/gpu/kinfu_large_scale/raycaster.h>
// #include <pcl/gpu/kinfu_large_scale/marching_cubes.h>

#include <ros/ros.h>

#include <cpu_tsdf/marching_cubes_tsdf_octree.h>
#include <cpu_tsdf/tsdf_interface.h>
#include <cpu_tsdf/tsdf_volume_octree.h>

#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

//FIXME using our data
int width_ = 648;
int height_ = 314;
float focal_length_x_ = 289.68255862405465;
float focal_length_y_ = 289.68255862405465;
float principal_point_x_ = 328.56589344569613;
float principal_point_y_ = 162.2624422540056;

using namespace std;
using namespace pcl;
using namespace PointMatcherSupport;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;
typedef typename Nabo::NearestNeighbourSearch<float> NNS;
typedef typename NNS::SearchType NNSearchType;
int main(int argc, char **argv){  
	
	ros::init(argc, argv, "pcltsdf");
	ros::start();
	ros::NodeHandle n;
	
	namespace bpo = boost::program_options;
	namespace bfs = boost::filesystem;
	bpo::options_description opts_desc("Allowed options");
	bpo::positional_options_description p;
	
	opts_desc.add_options()
	("help,h", "produce help message")
	("in", bpo::value<std::string> ()->required (), "Input dir")
	("out", bpo::value<std::string> ()->required (), "Output dir")
	("volume-size", bpo::value<float> (), "Volume size")
	("cell-size", bpo::value<float> (), "Cell size")
	("num-frames", bpo::value<size_t> (), "Partially integrate the sequence: only the first N clouds used")
	("visualize", "Visualize")
	("verbose", "Verbose")
	("color", "Store color in addition to depth in the TSDF")
	("flatten", "Flatten mesh vertices")
	("cleanup", "Clean up mesh")
	("invert", "Transforms are inverted (world -> camera)")
	("world", "Clouds are given in the world frame")
	("organized", "Clouds are already organized")
	("width", bpo::value<int> (), "Image width")
	("height", bpo::value<int> (), "Image height")
	("zero-nans", "Nans are represented as (0,0,0)")
	("num-random-splits", bpo::value<int> (), "Number of random points to sample around each surface reading. Leave empty unless you know what you're doing.")
	("fx", bpo::value<float> (), "Focal length x")
	("fy", bpo::value<float> (), "Focal length y")
	("cx", bpo::value<float> (), "Center pixel x")
	("cy", bpo::value<float> (), "Center pixel y")
	("save-ascii", "Save ply file as ASCII rather than binary")
	("cloud-units", bpo::value<float> (), "Units of the data, in meters")
	("pose-units", bpo::value<float> (), "Units of the poses, in meters")
	("max-sensor-dist", bpo::value<float> (), "Maximum distance data can be from the sensor")
	("min-sensor-dist", bpo::value<float> (), "Minimum distance data can be from the sensor")
	("trunc-dist-pos", bpo::value<float> (), "Positive truncation distance")
	("trunc-dist-neg", bpo::value<float> (), "Negative truncation distance")
	("min-weight", bpo::value<float> (), "Minimum weight to render")
	("cloud-only", "Save aggregate cloud rather than actually running TSDF")
	;
	
	bpo::variables_map opts;
	bpo::store(bpo::parse_command_line(argc, argv, opts_desc, bpo::command_line_style::unix_style ^ bpo::command_line_style::allow_short), opts);
	

	// Initialize
	string tsdfvtkname = argv[1];
	DP tsdfvtk = DP::load(tsdfvtkname);
	float xmin = 100000, xmax = -100000, ymin = 100000, ymax = -100000, zmin = 100000, zmax = -100000;
	for(int i = 0; i < tsdfvtk.features.cols(); i++){
		if(tsdfvtk.features(0,i) < xmin)
			xmin = tsdfvtk.features(0,i);
		if(tsdfvtk.features(0,i) > xmax)
			xmax = tsdfvtk.features(0,i);
		if(tsdfvtk.features(1,i) < ymin)
			ymin = tsdfvtk.features(1,i);
		if(tsdfvtk.features(1,i) > ymax)
			ymax = tsdfvtk.features(1,i);
		if(tsdfvtk.features(2,i) < zmin)
			zmin = tsdfvtk.features(2,i);
		if(tsdfvtk.features(2,i) > zmax)
			zmax = tsdfvtk.features(2,i);
	}
	
	cerr<<"xmin "<<xmin<<" xmax "<<xmax<<" ymin "<<ymin<<" ymax "<<ymax<<" zmin "<<zmin<<" zmax "<<zmax<<endl;
	float tsdf_sizex = xmax - xmin, tsdf_sizey = ymax - ymin, tsdf_sizez = zmax - zmin;
	float tsdf_size = ((xmax - xmin)>(ymax-ymin)) ? (xmax-xmin): (ymax-ymin);
	float cell_size = 0.3;
	int desired_res = tsdf_size / cell_size;
	int tsdf_res;
	// Snap to nearest power of 2;
	int nn = 1;
	while (desired_res > nn)
	{
		nn *= 2;
	}
	tsdf_res = nn;
	cerr<<"tsdf_size "<<tsdf_size<<" tsdf_res "<<tsdf_res<<endl;
	
	int num_random_splits = 1;
	float max_sensor_dist = 60;
	float min_sensor_dist = 0;
	float min_weight = 0;
	float trunc_dist_pos = 0.03;
	float trunc_dist_neg = 0.03;
	bool integrate_color = false;
	
	cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;
	tsdf.reset (new cpu_tsdf::TSDFVolumeOctree);
	tsdf->setGridSize (tsdf_sizex, tsdf_sizey, tsdf_sizez);
	tsdf->setResolution (tsdf_res, tsdf_res, tsdf_res);
	tsdf->setImageSize (width_, height_);
	tsdf->setCameraIntrinsics (focal_length_x_, focal_length_y_, principal_point_x_, principal_point_y_);
	tsdf->setNumRandomSplts (num_random_splits);
	tsdf->setSensorDistanceBounds (min_sensor_dist, max_sensor_dist);
	tsdf->setIntegrateColor (integrate_color);
	tsdf->setDepthTruncationLimits (trunc_dist_pos, trunc_dist_neg);
	tsdf->reset ();
	
	pcl::gpu::kinfuLS::StandaloneMarchingCubes<pcl::PointXYZI> m_cubes (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z, volume_size);

	
}
