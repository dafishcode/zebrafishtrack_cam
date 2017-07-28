#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <FlyCapture2Defs.h>
#include <FlyCapture2Platform.h>
#include <iostream>
#include <sstream>
#include<fstream>
#include<sys/stat.h>
#include<signal.h>
#include <pthread.h>
#include <semaphore.h>
#include<cstdio>

#include "util.h" //Our Custom Functions for Recording and Saving IMages


using namespace std;
using namespace FlyCapture2;

cv::Mat gframeBuffer; //Global Image Buffer For display
cv::Mat gframeMask;
sem_t   semImgCapCount;////Semaphore for image Captured Signal
sem_t   semImgFishDetected;////Semaphore for Fish Detected
pthread_cond_t cond;
pthread_mutex_t lock    = PTHREAD_MUTEX_INITIALIZER;
bool bImgCaptured       = false;/// Global Flag indicating new Image Has been captured by camera


int main(int argc, char** argv)
{

    pthread_t tidDisplay,tidRec; //Recorder And Monitor/Display Thread

    //int i = 0;
    Error error;
    BusManager busMgr;
    Camera cam;
    PGRGuid guid;
    VideoMode cVidMod;
    FrameRate cFps;
    F7 f7;
    /// Handle Command Line Parameters //
    const cv::String keys =
        "{help h usage ? |      | print this help  message   }"
        "{@outputDir     |<none>| Dir where To save sequence of images }"
        "{@crop          | 0    | ROI to crop images before save       }"
        "{cammode m      |1     | Mode 1 is low Res high FPS}"
        "{fps            | 300.0| Camera capture fps and output video fps}"
        "{shutter        | 3.0  | Camera shutter speed - set to 3ms }"
        "{duration t     |120   | Max recording time in seconds     }"
        "{ts timestamp   |false | use time stamp       }"
        "{e event        |false | Record only when fish is visible (Event Capture))  }"
        ;

    cv::CommandLineParser parser(argc, argv, keys);

    stringstream ssMsg;

    ssMsg<<"Zebrafish Experiments Recording for the Cameleon 3 FLIR camera."<<endl;
    ssMsg<<"--------------------------"<<endl;
    ssMsg<<"Author : Kontantinos Lagogiannis 2017"<<endl;
    ssMsg<<"./Rec_onDisk <MODE=1> <outfolder> <crop=0> <fps=300> <duration=120sec> <timestamp=false>"<<endl;
    ssMsg<<"(note: folder is automatically generated when absent)"<<endl;

    parser.about(ssMsg.str() );

    if (parser.has("help"))
    {
        parser.printMessage();

        return 0;
    }


    Mode k_fmt7Mode     = MODE_1; //Default
    k_fmt7Mode          = (Mode)parser.get<int>("cammode");
    float fFrameRate    = parser.get<float>("fps");
    int iCrop           = parser.get<int>("@crop");
    float fshutter      = parser.get<float>("shutter");
    uint uiduration     = parser.get<uint>("duration");
    string soutFolder   = parser.get<string>("@outputDir");
    bool use_time_stamp = parser.has("timestamp");


    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }
    //PrintBuildInfo();

    /// END OF COmmand LIne Parsing
    ///////////////////////////////
    /// Camera Discovery
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
	
    ///Connect to Camera - Set it to derired MODE and FPS and Shutter///
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


   ///Set mode and Print Camera Info / In/Out Camera Fps Setting - Setting And Actual
   SetCam(&cam,f7,k_fmt7Mode,k_fmt7PixFmt,fFrameRate,fshutter);


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
    RSC_input.cam               = &cam;
    RSC_input.proc_folder       = soutFolder;
    RSC_input.display           = string(ZR_WINDOWNAME);
    RSC_input.crop              = iCrop;
    RSC_input.MaxFrameDuration =  fFrameRate*uiduration; //Calc Max Frames given camera FPS
    RSC_input.eventCount        = 0;//Start Zero And IMg Detection will increment this
    //init Semaphore
    sem_init(&semImgCapCount,0,0);
    sem_init(&semImgFishDetected,0,1); //Initially In Run

    //Make Mask
    ///Draw ROI Mask
    gframeMask = cv::Mat::zeros(512,640,CV_8UC1);
    cv::circle(gframeMask,cv::Point(640/2,512/2),512/2+20,CV_RGB(255,255,255),-1,CV_FILLED);


    //Rec_onDisk_SingleCamera2((void*)&RSC_input,cMaxFrames);
    //Start The Recording Thread
    if (pthread_create(&tidRec, NULL, &Rec_onDisk_SingleCamera2, (void *)&RSC_input) != 0) {
        printf("Oh Ohh... Thread for Camera recording Rec_onDisk_SingleCamera2 could not start :( \n");
        cam.StopCapture();
        cam.Disconnect();
        return -1;
    }

    //cv::namedWindow(RSC_input.display,cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    //cv::resizeWindow(RSC_input.display, fmt7Info.maxHeight,fmt7Info.maxWidth);

    struct thread_data ReaderFnArgs;
    ReaderFnArgs.mode           = 0; //How to Save File Mode
    ReaderFnArgs.proc_folder    = soutFolder;
    ReaderFnArgs.prefix0        = fixedLengthString(1,3);
    ReaderFnArgs.windisplay     = RSC_input.display;
    ReaderFnArgs.format         = ZR_OUTPICFORMAT;

    CreateOutputFolder(soutFolder);

    ///\note
    /// cv:imshow functions need to be run from same thread - otherwise opencv hangs
    if (pthread_create(&tidDisplay, NULL, &ReadImageSeq, (void *)&ReaderFnArgs) != 0) {
        printf("Oh Ohh... Thread for Camera Display ReadImageSeq could not start :( \n");
        cam.StopCapture();
        cam.Disconnect();
        return -1;
    }


    //pthread_join(tidRec, NULL); //Wait Until Done / Join Main Thread
    pthread_join(tidDisplay, NULL); //Wait Until Done / Join Main Thread

//    char c = 0;
//    while(c!='q'){
//        //What for Quit
//        c=cv::waitKey(100);
//    }


    sem_destroy(&semImgCapCount);
    sem_destroy(&semImgFishDetected);

    //cam.StopCapture();
    //cam.Disconnect();





    return 0;
}
