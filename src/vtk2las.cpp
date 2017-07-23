#include <liblas/liblas.hpp>

#include <fstream>  // std::ofstream
#include <iostream> // std::cout

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "ros/ros.h"
#include<ros/package.h>
#include <Eigen/Eigen>

#include <eigen3/Eigen/Geometry>
#include <Eigen/Core>
#include "eigen3/Eigen/src/Geometry/Transform.h"


typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;
     
using namespace std;
int main (int argc, char** argv){
	
  	ros::init(argc, argv, "vtk2las");
	ros::start();
	
	ros::NodeHandle nh;
	ros::Publisher pcPublisher;
	
	string base_dir = ros::package::getPath("offline_optimization")+"/velo/";
	string result_dir = ros::package::getPath("offline_optimization")+"/las/";
	string file_name = "/home/dxq/catkin_ws/src/offline_optimization/name.txt";
	
	Eigen::Quaternion<double> qq;
	qq.x() = 0.0052;
	qq.y() = 0.00368;
	qq.z() = 0.89922;
	qq.w() = 0.43745;
	
	
	
	liblas::SpatialReference srs;
	srs.SetFromUserInput("EPSG:4326");

	Eigen::Isometry3d Iso;
	PM::TransformationParameters pose_matrix = PM::TransformationParameters::Identity(4,4);
	pose_matrix(0,3) = 4;
	Iso = pose_matrix.block<3,3>(0,0).cast<double>();
	Iso.translation() = pose_matrix.block<3,1>(0,3).cast<double>();
	cerr<<Iso.rotation()<<endl;
	cerr<<Iso.translation()<<endl;
	cerr<<Iso.matrix()<<endl;
	Eigen::Matrix4d M = Iso.matrix();
	cerr<<M<<endl;
	cerr<<"____"<<endl;
	cerr<<Iso(0,0)<<" "<<Iso(0,3)<<endl;
	cerr<<Iso.inverse().matrix()<<endl;
// 	
	cerr<<endl;
	Eigen::AngleAxisd poseAngle(qq.toRotationMatrix());
	cerr<<poseAngle.angle()<<endl;
	cerr<<poseAngle.axis()<<endl;
	cerr<<poseAngle.toRotationMatrix()<<endl;
	poseAngle.angle() = 90;
	cerr<<poseAngle.toRotationMatrix()<<endl;
	poseAngle.angle() = 180/180*3.14;
	cerr<<poseAngle.toRotationMatrix()<<endl;
	Iso = poseAngle.toRotationMatrix();
	Eigen::Quaterniond q(Iso.rotation());
	cerr<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
	
// 	
// 	 
	while(ros::ok()){};
// 	
	ifstream readf(file_name);
	while(!readf.eof()){
		string name;
		getline(readf, name);
		cerr<<name<<endl;
		int name_l = name.length();

		unique_ptr<DP> data( new DP(DP::load(base_dir+name)) );
		int intensity_begin = data.get()->getDescriptorStartingRow("intensity");
// 		int ring_begin = data.get()->getDescriptorStartingRow("ring");
// 		cerr<<"intensity "<<intensity_begin<<endl;
// 		cerr<<"ring "<<ring_begin<<endl;
		name[name_l-3] = 'l';
		name[name_l-2] = 'a';
		name[name_l-1] = 's';
		std::ofstream ofs;
		ofs.open(result_dir+name, ios::out | ios::binary);
		
		liblas::Header header;
// 		header.SetDataFormatId(liblas::ePointFormat3); // Time only
		// Set coordinate system using GDAL support


		header.SetSRS(srs);
		header.SetPointRecordsCount(data->features.cols());
		liblas::Schema schema(liblas::ePointFormat1 );
		
	// 	schema.SetDataFormatId(liblas::ePointFormat0);
		header.SetSchema(schema);
		header.SetScale(0.0001,0.0001,0.0001);

		liblas::Writer writer(ofs, header);
		
// 		cerr<<(double)(data.features(0,0))<<endl;

		for(int i = 0; i<data->features.cols(); i++){
			liblas::Point point(&header);
			point.SetX((double)(data->features(0,i)));
			point.SetY((double)(data->features(1,i)));
			point.SetZ((double)(data->features(2,i)));
			
			point.SetIntensity(data->descriptors(0,i));
// 
// // 			cerr<<point.GetRawX()<<" "<<point.GetX()<<endl;
// 	// 		point.GetRawX()
// 	// 		point.SetCoordinates((double)(data.features(0,i)), (double)(data.features(1,i)), (double)(data.features(2,i)));
			writer.WritePoint(point);
// 
		}
// // // 		cerr<<header.GetPointRecordsCount()<<endl;
// // 		
		ofs.close();
	}
// 	std::ifstream ifs;
// // 	ifs.open(base_dir+"file.las", std::ios::in | std::ios::binary);
// 	ifs.open("/home/dxq/catkin_ws/src/offline_optimization/las/1495177083.646931000.las");
// // 	
// 	liblas::ReaderFactory f;
// 	liblas::Reader reader = f.CreateWithStream(ifs);
// // 	
// 	liblas::Header const& headerr = reader.GetHeader();
// // 
// 	std::cout << "Compressed: " << (headerr.Compressed() == true) ? "true":"false";
// 	std::cout << "Signature: " << headerr.GetFileSignature() << '\n';
// 	std::cout << "Points count: " << headerr.GetPointRecordsCount() << '\n';
// 	std::cout << "formid: " << headerr.GetDataFormatId() << '\n';
// 	
// 	int count = 0;
// 	while (reader.ReadNextPoint())
// 	{
// 		liblas::Point const& p = reader.GetPoint();
// 
// 		std::cout << p.GetX() << ", " << p.GetY() << ", " << p.GetZ() <<", "<<p.GetIntensity()<< "\n";
// // 		cerr<<count++<<endl;
// 	}
// 	

	return 0;
}