cmake_minimum_required(VERSION 2.8.3)
project(mesh)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
tf tf2 tf_conversions message_filters map_msgs libpointmatcher_ros visualization_msgs
interactive_markers ethzasl_gridmap_2d message_generation libpointmatcher roslib)
find_package(libnabo)

find_package(Eigen3 )

find_package(PCL 1.8  REQUIRED  )

find_package(VTK REQUIRED)

find_package(Boost REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

find_package(liblas REQUIRED)
include_directories(/home/dxq/library/libLAS-1.8.1/include)


# find_package(cpu_tsdf REQUIRED)

catkin_package (
	CATKIN_DEPENDS roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
tf tf2 tf_conversions message_filters map_msgs libpointmatcher_ros ethzasl_gridmap_2d visualization_msgs interactive_markers 
message_runtime libpointmatcher libnabo roslib
)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		add_definitions(-std=gnu++0x)
	else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
	endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
else(CMAKE_COMPILER_IS_GNUCXX)
	message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)
export(PACKAGE CPUTSDF)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# create the CPUTSDF library
add_library (my_cpu_tsdf SHARED
              src/lib/octree.cpp
              src/lib/tsdf_volume_octree.cpp
              src/lib/tsdf_interface.cpp
              src/lib/marching_cubes_tsdf_octree.cpp)

            target_link_libraries (my_cpu_tsdf ${PCL_LIBRARIES})
install(TARGETS my_cpu_tsdf DESTINATION lib${LIB_SUFFIX})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DHAVE_YAML_CPP)

include_directories(
include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${VTK_INCLUDE_DIR}

${CPU_TSDF_INCLUDE_DIR}

)
# message("pcl include dir" ${PCL_INCLUDE_DIRS})


message("PCL---" ${PCL_INCLUDE_DIRS}) 

add_executable(mesh

src/main.cpp)


add_dependencies(mesh ${PROJECT_NAME}_gencpp)
target_link_libraries(mesh
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
libcxsparse.so 
# libg2o_ext_csparse.so
libg2o_core.so 
libg2o_stuff.so 
libg2o_types_slam3d.so 
libg2o_csparse_extension.so 
libg2o_solver_csparse.so  
)

add_executable(vtk2stl
src/vtk2stl.cpp)

add_dependencies(vtk2stl ${PROJECT_NAME}_gencpp)
target_link_libraries(vtk2stl
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}

)

# Install the Header files preserving the directory
set(HEADER_FILES include/cpu_tsdf/impl/tsdf_volume_octree.hpp
		 include/cpu_tsdf/marching_cubes_tsdf_octree.h
		 include/cpu_tsdf/octree.h
		 include/cpu_tsdf/tsdf_interface.h
		 include/cpu_tsdf/tsdf_volume_octree.h
		 include/eigen_extensions/eigen_extensions.h
)
FOREACH(HEADER ${HEADER_FILES})
	STRING(REGEX MATCH "(.*)[/]" DIR ${HEADER})
	INSTALL(FILES ${HEADER} DESTINATION ${DIR})
ENDFOREACH(HEADER)

add_executable(vtk2pcd
src/vtk2pcd.cpp)

add_dependencies(vtk2pcd ${PROJECT_NAME}_gencpp)
target_link_libraries(vtk2pcd
${catkin_LIBRARIES}

${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
my_cpu_tsdf
${Boost_PROGRAM_OPTIONS_LIBRARY}
)

add_executable(pointcloud2image
src/pointcloud2image.cpp)

add_dependencies(pointcloud2image ${PROJECT_NAME}_gencpp)
target_link_libraries(pointcloud2image
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
)

add_executable(vtk2las
src/vtk2las.cpp)

add_dependencies(vtk2las ${PROJECT_NAME}_gencpp)
target_link_libraries(vtk2las
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
/usr/lib/liblas.so

)

add_executable(vtk2ply
src/vtk2ply.cpp)

add_dependencies(vtk2ply ${PROJECT_NAME}_gencpp)
target_link_libraries(vtk2ply
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
/usr/lib/liblas.so

)


# include_directories(
# /home/dxq/library/pcl-pcl-1.8.0/gpu/kinfu_large_scale/include
# /home/dxq/library/pcl-pcl-1.8.0/gpu/containers/include
# /home/dxq/library/pcl-pcl-1.8.0/gpu/kinfu/include
# )
# add_executable(pcltsdf
# src/pcltsdf.cpp)
# 
# add_dependencies(pcltsdf ${PROJECT_NAME}_gencpp)
# target_link_libraries(pcltsdf
# ${catkin_LIBRARIES}
# ${G2O_LIBRARIES}
# ${EIGEN3_LIBS}
# ${PCL_LIBRARIES}
# ${VTK_LIBRARIES}
# /usr/lib/liblas.so
# my_cpu_tsdf
# )

add_executable(gaussian
src/gaussian.cpp)

add_dependencies(gaussian ${PROJECT_NAME}_gencpp)
target_link_libraries(gaussian
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
)

find_package(Pangolin REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
include_directories(
${Pangolin_INCLUDE_DIRS}
)

add_executable(gl_ellipse
src/gl_ellipse.cpp)

add_dependencies(gl_ellipse ${PROJECT_NAME}_gencpp)
target_link_libraries(gl_ellipse
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
${Pangolin_LIBRARIES}
${GLUT_LIBRARIES}
)

add_executable(pangolin_ellipse
src/pangolin_ellipse.cpp)

add_dependencies(pangolin_ellipse ${PROJECT_NAME}_gencpp)
target_link_libraries(pangolin_ellipse
${catkin_LIBRARIES}
${G2O_LIBRARIES}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${VTK_LIBRARIES}
${Pangolin_LIBRARIES}
${GLUT_LIBRARIES}
)

# add_subdirectory(src/cuda_marching_cube)