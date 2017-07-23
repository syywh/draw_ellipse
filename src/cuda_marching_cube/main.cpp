/*Build a wall and test it with Marching cube*/
#include "cuda_runtime.h"
#include "cublas_v2.h"
#include <ctime>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "CUDAMarchingCubes.h"
#include "Convenience.h"

using namespace std;
#define NUM_ROW 480
#define NUM_COL 480
#define NUM_HEI 90
#define INDEX_WALL 100
#define MAX_FACE 100000

int main()
{
    float* volume = new float[NUM_ROW * NUM_COL * NUM_HEI];
    float* colors = new float[NUM_ROW * NUM_COL * NUM_HEI];
    for(int i = 0;i < NUM_ROW;i++)
    {
        for(int j = 0;j < NUM_COL;j++)
        {
            for(int k = 0;k < NUM_HEI;k++)
            {
                if(j<INDEX_WALL)
                {
                    *(volume+NUM_COL*NUM_HEI*i+NUM_HEI*j+k) = -1;
                }
                else if(j==INDEX_WALL)
                {
                    *(volume+(NUM_COL*NUM_HEI)*i+NUM_HEI*j+k) = 0;
                }
                else
                {
                    *(volume+(NUM_COL*NUM_HEI)*i+NUM_HEI*j+k) = 1;
                }
                *(colors+(NUM_COL*NUM_HEI)*i+NUM_HEI*j+k) = 0;
            }
        }
    }
    uint3 gridsize      = make_uint3(NUM_ROW,NUM_COL,NUM_HEI);
    float3 gridOrigin   = make_float3(0,0,0);
    float3 boundingBox  = make_float3(NUM_ROW,NUM_COL,NUM_HEI);//actual length of box
    bool cudaArray      = false;
    unsigned int maxverts = MAX_FACE * 3;

    float3* d_vertOut;
    float3* d_normOut;
    float3* d_colOut;
    cudaMalloc((void**)&d_vertOut,3*MAX_FACE*sizeof(float3));
    cudaMalloc((void**)&d_normOut,  MAX_FACE*sizeof(float3));
    cudaMalloc((void**)&d_colOut, 3*MAX_FACE*sizeof(float3));


    CUDAMarchingCubes marchingCube;
    float *d_volume, *d_colors;
    cudaMalloc((void**)&d_volume,NUM_ROW * NUM_COL * NUM_HEI*sizeof(float));
    cudaMalloc((void**)&d_colors,NUM_ROW * NUM_COL * NUM_HEI*sizeof(float));
    cudaMemcpy(d_volume, volume, NUM_ROW * NUM_COL * NUM_HEI*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_colors, colors, NUM_ROW * NUM_COL * NUM_HEI*sizeof(float), cudaMemcpyHostToDevice);
    marchingCube.computeIsosurface(d_volume, d_colors,
                                         CUDAMarchingCubes::RGB4U,
                                         gridsize, gridOrigin,
                                         boundingBox, true,
                                         d_vertOut, d_normOut,
                                         d_colOut, maxverts);
//    marchingCube.computeIsosurface(volume, colors,
//                                         CUDAMarchingCubes::RGB4U,
//                                         gridsize, gridOrigin,
//                                         boundingBox, cudaArray,
//                                         d_vertOut, d_normOut,
//                                         d_colOut, maxverts);


    //  Export stl model
    float3 *vertOut = new float3[3*MAX_FACE];
    float3 *normOut = new float3[  MAX_FACE];
    cudaMemcpy(vertOut, d_vertOut, 3*MAX_FACE*sizeof(float3), cudaMemcpyDeviceToHost);
    cudaMemcpy(normOut, d_normOut,   MAX_FACE*sizeof(float3), cudaMemcpyDeviceToHost);
    int num_face = MAX_FACE;
    char* filename = "model.stl";
    exportSTL(vertOut,normOut,num_face,filename,marchingCube.GetVertexCount());

    return 0;
}
