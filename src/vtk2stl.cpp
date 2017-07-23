#include "vtkPolyDataReader.h"
#include "vtkSTLWriter.h"
#include "vtkTriangleFilter.h"
#include "ros/ros.h"
#include<ros/package.h>

using namespace std;
int main (int argc, char** argv)
{
  	ros::init(argc, argv, "mesh");
	ros::start();
	
	ros::NodeHandle nh;
	
        vtkPolyDataReader *reader=vtkPolyDataReader::New();
	string vtkfile = ros::package::getPath("mesh")+"/data/mesh.vtk";
        reader->SetFileName(vtkfile.c_str());
        vtkSTLWriter *writer=vtkSTLWriter::New();

    vtkTriangleFilter *tri=vtkTriangleFilter::New();
    tri->SetInput(reader->GetOutput());


        writer->SetInputConnection(tri->GetOutputPort());
	string savename = ros::package::getPath("mesh")+"/data/mesh.ply";
        writer->SetFileName(savename.c_str());
	writer->Update();
        writer->Write();
        //writer->Update();

}


