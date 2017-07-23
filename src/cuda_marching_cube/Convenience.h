#ifndef CONVENIENCE_H
#define CONVENIENCE_H

#endif // CONVENIENCE_H

#include <iostream>
#include <vector>
#include <fstream>

using namespace std;
bool exportSTL(const float3* vertOut, const float3* normOut,
               const int num_face, const char* filename,
               const int num_vert)
{
    ofstream outFile(filename);
    if(!outFile.is_open())
    {
        exit(EXIT_FAILURE);
    }

    //  Begin writing ...
    outFile<<"solid MYSOLID"<<endl;

    for(int i = 0;i < num_vert/3;i++)
    {
        outFile<<"  facet normal "<<normOut[i].x<<" "<<normOut[i].y<<" "<<normOut[i].z<<endl;
        outFile<<"    outer loop"<<endl;
        outFile<<"      vertex "<<vertOut[3*i  ].x<<" "<<vertOut[3*i  ].y<<" "<<vertOut[3*i  ].z<<endl;
        outFile<<"      vertex "<<vertOut[3*i+1].x<<" "<<vertOut[3*i+1].y<<" "<<vertOut[3*i+1].z<<endl;
        outFile<<"      vertex "<<vertOut[3*i+2].x<<" "<<vertOut[3*i+2].y<<" "<<vertOut[3*i+2].z<<endl;
        outFile<<"    endloop"<<endl;
        outFile<<"  endfacet"<<endl;
    }

    //  End writing
    outFile<<"endsolid MYSOLID"<<endl;
}
