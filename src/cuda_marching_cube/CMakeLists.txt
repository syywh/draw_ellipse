CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(marching_cube)
set(CMAKE_BUILD_TYPE debug)
FIND_PACKAGE(CUDA REQUIRED )
#FIND_PACKAGE(MPI REQUIRED)

#INCLUDE(FindCUDA)

INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})

FILE(GLOB SOURCES "*.cu" "*.cpp" "*.c" "*.h" "*.cuh")
message("----------SOURCES " ${SOURCES})
CUDA_ADD_EXECUTABLE(marching_cube ${SOURCES})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

LIST(APPEND CMAKE_CXX_FLAGS "-std=c++0x -O3 -ffast-math -Wall")

LIST(APPEND CUDA_NVCC_FLAGS --compiler-options -fno-strict-aliasing -lineinfo -use_fast_math -Xptxas -dlcm=cg)
#LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_20,code=sm_20)
#LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_30,code=sm_30)
LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_45,code=sm_45)

CUDA_ADD_CUBLAS_TO_TARGET(marching_cube)



TARGET_LINK_LIBRARIES(marching_cube ${CUDA_LIBRARIES})
TARGET_LINK_LIBRARIES(marching_cube ${CUDA_cublas_LIBRARY})
TARGET_LINK_LIBRARIES(marching_cube ${CUDA_cusparse_LIBRARY})
message(status  ${CUDA_cusparse_LIBRARY})
#LINK_DIRECTORIES(/usr/local/cuda-7.5/lib64)
#TARGET_LINK_LIBRARIES(marching_cube libcusolver.so)
set(CUDA_cusolver_LIBRARY /usr/local/cuda-8.0/targets/x86_64-linux/lib/libcusolver.so)
set(OpenCV_INCLUDE /home/dxq/library/opencv-2.4.13/release/include/)
set(OpenCV_core_LIB "/home/dxq/library/opencv-2.4.13/release/lib/libopencv_core.so")
set(OpenCV_imgproc_LIB "/home/dxq/library/opencv-2.4.13/release/lib/libopencv_core.so")
set(OpenCV_highgui_LIB "/home/dxq/library/opencv-2.4.13/release/lib/libopencv_core.so")
message(status  ${CUDA_cusolver_LIBRARY})
message(status  ${OpenCV_INCLUDE})
include_directories(${OpenCV_INCLUDE})
TARGET_LINK_LIBRARIES(marching_cube ${CUDA_cusolver_LIBRARY})
target_link_libraries(marching_cube ${OpenCV_core_LIB} )
target_link_libraries(marching_cube ${OpenCV_imgproc_LIB} )
target_link_libraries(marching_cube ${OpenCV_highgui_LIB} )

#TARGET_LINK_LIBRARIES(marching_cube ${CUDA_cusolver_LIBRARY})

