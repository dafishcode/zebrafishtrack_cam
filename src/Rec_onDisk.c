#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <FlyCapture2Defs.h>
#include <FlyCapture2Platform.h>
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
        cout<<"./Rec_onDisk <MODE=1> <folder> <crop?>"<<endl;
	    cout<<"(note: folder is automatically generated when absent)"<<endl;
            exit(0);
    }

    //PrintBuildInfo();

    //int i = 0;
    Error error;
    BusManager busMgr;
    PGRGuid guid;
    VideoMode cVidMod;
    FrameRate cFps;
    F7 f7;
    Mode k_fmt7Mode = MODE_1; //Default
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
   {
       PrintError(error);
       return -1;
   }
   std::cout << "There N=" << numCameras << " Cameras connected " << std::endl;


    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
   {
       PrintError(error);
       return -1;
   }
    std::cout << "Got camera idx 0 Guid " << guid.value <<  std::endl;

    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;
    if(atoi(argv[1])==0)
    {
        k_fmt7Mode=MODE_0;
    }
    else
    {
        k_fmt7Mode=MODE_1;
    }
	
    //Connect to Cam
    Camera cam;
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
   {
       cam.Disconnect();
       PrintError(error);
       return -1;
   }
    std::cout << "Connected To cam guid:" << guid.value <<  std::endl;

    ///Format 7 Is the Non IEEE compliant formats that the camera supports
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;

    error = cam.GetFormat7Info(&fmt7Info, &supported);
    if (error != PGRERROR_OK)
    {
         PrintError(error);
         return -1;
    }
    PrintFormat7Capabilities(fmt7Info);


   ///Set mode and Print Camera Info
   unsigned int camFps;
   SetCam(&cam,f7,k_fmt7Mode,k_fmt7PixFmt,camFps);


    ///Print Mode Information
    cam.GetVideoModeAndFrameRate(&cVidMod,&cFps);
    std::cout << "///// Current Camera Video Mode ////" << std::endl;
    std::cout << "Vid Mode :" << cVidMod << " Fps Mode Set: " << cFps << std::endl;

   // cam.SetVideoModeandFrameRate( VIDEOMODE_640x480Y8 , FRAMERATE_FORMAT7 );
    if ((k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0)
      {
          // Pixel format not supported!
          cout << "Pixel format is not supported" << endl;
          return -1;
      }
    

    //Setup thread
    struct thread_data2 RSC_input;
    RSC_input.cam = &cam;
    RSC_input.proc_folder=argv[2];
    RSC_input.display="display";
    RSC_input.crop=atoi(argv[3]);
    
    cv::namedWindow("display",cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("display", 640,480);

    unsigned int cMaxFrames = camFps*60*2; //2 Mins at 300fps

    Rec_onDisk_SingleCamera2((void*)&RSC_input,cMaxFrames);

    ReadImageSeq(argv[2],"display");

    cam.StopCapture();
    cam.Disconnect();
    return 0;
}
