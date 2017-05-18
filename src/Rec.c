#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <sstream>
#include<fstream>
#include"../include/functions2.h"

using namespace std;
using namespace FlyCapture2;

int main(int argc, char** argv)
{
    //PrintBuildInfo();
    size_t seq_size=5000;
    BusManager busMgr;
    PGRGuid guid;
    busMgr.GetCameraFromIndex(0, &guid);
	Camera cam;
	cam.Connect(&guid);
    
    struct thread_data2 RSC_input;
    RSC_input.cam = &cam;
    RSC_input.proc_folder=argv[1];
    RSC_input.display="display";
    RSC_input.seq_size=seq_size;

    Rec_SingleCamera((void*)&RSC_input);
    ReadImageSeq("test","display");

    return 0;
}
