#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/surface/mls.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "ros/ros.h"
#include<ros/package.h>

#include "pointmatcher_ros/point_cloud.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;
     
using namespace std;
int main (int argc, char** argv)
{

  	ros::init(argc, argv, "mesh");
	ros::start();
	
	ros::NodeHandle nh;
	ros::Publisher pcPublisher;
	
	pcPublisher = nh.advertise<sensor_msgs::PointCloud2>("point_map", 2, true);
	
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
      //   pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
	string vtkfile = ros::package::getPath("mesh")+"/data/DataPoints.vtk";
	cerr<<"vtkfile "<<vtkfile<<endl;
	DP clouddp  = DP::load(vtkfile);
	
	cerr<<clouddp.features.cols()<<endl;
	sensor_msgs::PointCloud2 sensorPD = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(clouddp, "/map", ros::Time::now());
	pcPublisher.publish(sensorPD);
	// 	cloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(sensorPD);
	pcl_conversions::toPCL(sensorPD,cloud_blob);
// 	cloud = &cloud_blob;
	
// 	pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
	pcl::fromROSMsg(sensorPD, *cloud);
	//* the data should be available in cloud

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1);
	sor.filter(*cloud_filter);
	
	cloud->clear();
	cloud = cloud_filter;
	
	// Normal estimation*
// 	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
// 	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	
// 	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_ptr(new pcl::PointCloud<pcl::PointNormal>);
	

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);	
	//MovingLeastSquares 之后，点的位置被改掉了
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	tree->setInputCloud (cloud);
// 	n.setInputCloud (cloud);
// 	n.setSearchMethod (tree);
// 	n.setKSearch (20);
// 	n.compute (*normals);
	
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setDilationIterations(2);
	mls.setDilationVoxelSize(3);
// 	mls.setSqrGaussParam(0.5);
	mls.setComputeNormals(true);
	mls.setSearchMethod(tree); 
	mls.setSearchRadius(3);
	mls.process(*mls_points_ptr); 
	cerr<<"mls_points_ptr "<<mls_points_ptr->size()<<endl;
	
	
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
// 	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
// 	pcl::concatenateFields (*cloud, *mls_points_ptr, *cloud_with_normals);
	
	pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
	for(int i = 0; i<mls_points_ptr->size(); i++)
	{

		pcl::Normal temp(mls_points_ptr->at(i).normal_x,mls_points_ptr->at(i).normal_y,mls_points_ptr->at(i).normal_z);
// 		cerr<<mls_points_ptr->at(i).x<<" "<<mls_points_ptr->at(i).y<<" "<<mls_points_ptr->at(i).z<<endl;
		normal->push_back(temp);
	}
// 	pcl::PointCloud<pcl::PointNormal> newcl = *mls_points_ptr;
// 	newcl.points;
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
// 	pcl::MarchingCubesHoppe<pcl::PointNormal>::KdTreePtr tree2(pcl::MarchingCubesHoppe<pcl::PointNormal>::KdTree); 
// 	pcl::MarchingCubesHoppe<pcl::PointNormal>::KdTreePtr tree2;	
// 	tree2->setInputCloud (cloud_with_normals);
// 	cerr<<"mls_points "<<mls_points.empty()<<endl;
	tree2->setInputCloud(mls_points_ptr);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::MarchingCubesRBF<pcl::PointNormal> mc;
// 	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	pcl::PolygonMesh triangles;
	pcl::TextureMesh Tex_Mesh;
	

// 	mc.setOffSurfaceDisplacement(0.5);
// 	
// 
// 	mc.setSearchMethod(tree2);
// 	mc.setIsoLevel(0.01);
// 	mc.setGridResolution(50,50,50);
// 	mc.setInputCloud(mls_points_ptr);
// // 	mc.setFieldValue<>();
// // 	mc.setPercentageExtendGrid();
// 	mc.reconstruct(triangles);
// 	// Set the maximum distance between connected points (maximum edge length)

// 
	// Set typical values for the parameters
	gp3.setSearchRadius (5);
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (300);
	gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
	gp3.setMinimumAngle(M_PI/16); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(true);

	// Get result
	gp3.setInputCloud(mls_points_ptr);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);
	
	Tex_Mesh.cloud = triangles.cloud;
	Tex_Mesh.header = triangles.header;
// 	Tex_Mesh.tex_polygons.push_back(triangles.polygons);
	
	for(int i = 0; i<triangles.polygons.size(); i++)
	{
	pcl::TexMaterial tex_material;
	tex_material.tex_Ka.r=255;	tex_material.tex_Ka.g = 0; tex_material.tex_Ka.b = 0;
	tex_material.tex_name = "color";
	Tex_Mesh.tex_materials.push_back(tex_material);
	vector<pcl::Vertices> temp;
	temp.push_back(triangles.polygons[i]);
	Tex_Mesh.tex_polygons.push_back(temp);
	}
		

// 	std::vector< Eigen::Vector2f > texcoord;
// 	texcoord.push_back(Eigen::Vector2f(0.0,0.0));
// 	texcoord.push_back(Eigen::Vector2f(3.0,0.0));
// 	texcoord.push_back(Eigen::Vector2f(1.0,3.0));
// 	texcoord.push_back(Eigen::Vector2f(0.0,1.0));
// 	Tex_Mesh.tex_coordinates.push_back(texcoord);

	


	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();	//-1表示没有连接进去的点
	std::vector<int> states = gp3.getPointStates();	//        NONE = -1（not-defined）  FREE = 0,      FRINGE = 1？边缘,       BOUNDARY = 2 边缘,  COMPLETED = 3内部
	cerr<<"parts "<<parts.size()<<endl<<"states "<<states.size()<<endl;
	string partsfile = ros::package::getPath("checkloop")+"/data/parts.txt";
	string statesfile = ros::package::getPath("checkloop")+"/data/states.txt";
	ofstream  savefile ;
	savefile.open(partsfile);
	for(int i = 0; i<parts.size(); i++)
	{
		savefile<<parts[i]<<endl;
	}
	savefile.close();
	savefile.open(statesfile);
	for(int i = 0; i<states.size(); i++)
	{
		savefile<<states[i]<<endl;
	}
	savefile.close();
	
	cerr<<"triangles "<<triangles.polygons.size()<<endl;
	cerr<<"cloud.width "<<triangles.cloud.width<<endl;
	string meshsave = ros::package::getPath("checkloop")+"/data/mesh.vtk";
	pcl::io::saveVTKFile (meshsave, triangles);
	
	
	
	//visualization
	 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);  //设置背景
	viewer->addPolygonMesh(triangles,"my");  //设置显示的网格
	viewer->addPointCloud(cloud, "cloud");
// 	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 1,1,"point normal");
	viewer->addCoordinateSystem(1);  //设置坐标系
	viewer->initCameraParameters();
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	pcl::visualization::PCLVisualizer* viewer_Tex (new pcl::visualization::PCLVisualizer("Tex_Mesh"));
	viewer_Tex->addTextureMesh(Tex_Mesh,"color");
	viewer_Tex->setBackgroundColor(0,0,0);
	viewer_Tex->addPointCloud(cloud, "cloud");
	viewer_Tex->addCoordinateSystem(1);  //设置坐标系
	viewer_Tex->addPolygonMesh(triangles,"my");  //设置显示的网格
	while(ros::ok())
	{
	    viewer->spinOnce(100);
	    viewer_Tex->spinOnce(100);
	    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
	}
	// Finish
	return (0);
}
