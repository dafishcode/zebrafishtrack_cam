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
    size_t seq_size=atoi(argv[4]);
    BusManager busMgr;
    PGRGuid guid;
	F7 f7;
    busMgr.GetCameraFromIndex(0, &guid);

	Mode mode;
	if(atoi(argv[1])==0) mode=MODE_0;
	else mode=MODE_1;

	Camera cam;
	cam.Connect(&guid);
	SetCam(&cam,f7,mode,PIXEL_FORMAT_RAW8);
    
    struct thread_data2 RSC_input;
    RSC_input.cam = &cam;
    RSC_input.proc_folder=argv[2];
	RSC_input.crop=atoi(argv[3]);
    RSC_input.display="display";
    RSC_input.seq_size=seq_size;

    Rec_SingleCamera((void*)&RSC_input);
    ReadImageSeq(RSC_input.proc_folder,"display");

    return 0;
}
