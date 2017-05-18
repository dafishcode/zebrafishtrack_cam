#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <sstream>
#include<fstream>
#include<sys/stat.h>
#include<signal.h>
#include<cstdio>
#include"../include/functions2.h"

using namespace std;
using namespace FlyCapture2;

int main(int argc, char** argv)
{
    if(argc<2){
	    cout<<"Recording software."<<endl;
	    cout<<"-------------------"<<endl;
	    cout<<"Usage:"<<endl;
	    cout<<"./Rec_onDisk <folder>"<<endl;
	    cout<<"(note: folder is automatically generated when absent)"<<endl;
            exit(0);
    }

    //PrintBuildInfo();
    int i;
    BusManager busMgr;
    PGRGuid guid;
    busMgr.GetCameraFromIndex(0, &guid);
    
    struct thread_data RSC_input;
    RSC_input.guid = &guid;
    RSC_input.proc_folder=argv[1];
    RSC_input.display="display";
    RSC_input.crop=atoi(argv[2]);
    
    cv::namedWindow("display",cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("display", 800,800);

    Rec_onDisk_SingleCamera((void*)&RSC_input);

    ReadImageSeq(argv[1],"display");

    return 0;
}
