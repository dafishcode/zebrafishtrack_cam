/// * \title zebraprey_cam
/// \author Konstantinos Lagogiannis 2017
/// \brief An event triggered recoding software used with pointgrey camera drivers.
/// records a an event for a min of mineventduration, up to max (2min).
///  on execution 1st event is immediatelly recorded, subsequent events trigger when a blob of sufficient size (like zebrafish)
/// is in view.
///
/// \param fps
/// \param timeout
/// \param mineventduration
/// \param mode 0/1 :  0 hres low fps,
/// \example ./zebraprey_cam '/mnt/SSDFastDisk/camera/expDataDylan/tst15/' --fps=30 --timeout=40 --mineventduration=5 --cammode=0
///*


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

#include"../include/circular_buffer_ts.h"
#include<boost/thread.hpp>
#include<boost/chrono.hpp>
#include <signal.h>
#include <functional>

using namespace std;
using namespace FlyCapture2;


cv::Mat gframeBuffer; //Global Image Buffer For display
cv::Mat gframeMask;
sem_t   semImgCapCount;////Semaphore for image Captured Signal
sem_t   semImgFishDetected;////Semaphore for Fish Detected
pthread_cond_t cond;
pthread_mutex_t lock    = PTHREAD_MUTEX_INITIALIZER;
bool bImgCaptured       = false;/// Global Flag indicating new Image Has been captured by camera

// Attempts to connect to Camera And Check If Cam supports A particular MODE
int connectCam(BusManager& busMgr,Camera& cam,int camIDX,Format7Info fmt7Info)
{
    PGRGuid guid;
    Error error;

    error = busMgr.GetCameraFromIndex(camIDX, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
     std::cout << "Got camera idx "<< camIDX << " Guid " << guid.value <<  std::endl;



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

     bool supported;


     error = cam.GetFormat7Info(&fmt7Info, &supported);
     if (error != PGRERROR_OK)
     {
          PrintError(error);
          return -1;
     }
     PrintFormat7Capabilities(fmt7Info);

     return 1;
} //End oF Connnect to Cam

int main(int argc, char** argv)
{

    pthread_t tidDisplay,tidRec; //Recorder And Monitor/Display Thread

    int BUFFER_SIZE=200; //Circular Buffer

    //int i = 0;
    Error error;
    BusManager busMgr;
    Camera camA,camB;
    Format7Info fmt7InfoA,fmt7InfoB;
    VideoMode cVidModA,cVidModB;
    FrameRate cFpsA,cFpsB;



    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;
    const PixelFormat k_fmt7PixFmt_Compressed = PIXEL_FORMAT_422YUV8_JPEG;      //= 0x40000001, /**< JPEG compressed stream. */

    F7 f7;
    /// Handle Command Line Parameters //
    const cv::String keys =
        "{help h usage ? |      | print this help  message   }"
        "{@outputDir     |<none>| Dir where To save sequence of images }"
        "{@crop          | 0    | ROI to crop images before save       }"
        "{camAmode Am      |1     | Mode 1 is low Res high FPS}"
        "{camBmode Bm      |0     | Mode 1 is low Res high FPS}"
        "{camAfps  Af       | 450.0| Camera capture fps and output video fps}"
        "{camBfps  Bf       | 40.0| Camera capture fps and output video fps}"
        "{camAshutter As       | 3.0  | Camera shutter speed - set to 3ms }"
        "{camBshutter Bs       | 3.0  | Camera shutter speed - set to 3ms }"
        "{eventtimeout e |240   | Max event recording duration, new event is created after timeout }"
        "{timeout t      |600   | Max recording time in seconds, stops the recording process (= 10 mins)  }"
        "{mineventduration d |30 | min duration (sec) of event once recording is triggered on CamA (1st event is autotriggered) }"
        "{ts timestamp   |true | use time stamp       }"
        "{e event        |true | Record only when fish is visible (Event Capture))  }"
        ;

    cv::CommandLineParser parser(argc, argv, keys);

    stringstream ssMsg;

    ssMsg<<"Zebrafish Experiments Recording for the Cameleon 3 FLIR camera."<<endl;
    ssMsg<<"Upgraded to Dual camera capture (2021)."<<endl;
    ssMsg<<"--------------------------"<<endl;
    ssMsg<<"Author : Kontantinos Lagogiannis 2017"<<endl;
    ssMsg<<"./Rec_onDisk <MODE=1> <outfolder> <crop=0> <camAfps=450> <timeout=600sec> <timestamp=false>"<<endl;
    ssMsg<<"(note: folder is automatically generated when absent)"<<endl;

    parser.about(ssMsg.str() );

    if (parser.has("help"))
    {
        parser.printMessage();

        return 0;
    }


    Mode k_fmt7ModeA     = MODE_1; //Default
    Mode k_fmt7ModeB     = MODE_0; //Default

    k_fmt7ModeA          = (Mode)parser.get<int>("camAmode");
    k_fmt7ModeB          = (Mode)parser.get<int>("camBmode");
    float fFrameRateA    = parser.get<float>("camAfps");
    float fshutterA      = parser.get<float>("camAshutter");
    float fFrameRateB    = parser.get<float>("camBfps");
    float fshutterB      = parser.get<float>("camBshutter");

    int iCrop           = parser.get<int>("@crop");
    uint uiduration     = parser.get<uint>("eventtimeout");
    string soutFolder   = parser.get<string>("@outputDir");
    bool use_time_stamp = parser.has("timestamp");
    uint uiTimeOutSec   = parser.get<uint>("timeout");
    uint uiEventMinDuration = parser.get<uint>("mineventduration");

    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    fmt7InfoA.mode       = k_fmt7ModeA;
    fmt7InfoB.mode       = k_fmt7ModeB;
    //PrintBuildInfo();

    /// END OF COmmand LIne Parsing //

    ///////////////////////////////

    /// Camera Discovery //
   unsigned int numCameras;
   error = busMgr.GetNumOfCameras(&numCameras);
   if (error != PGRERROR_OK)
   {
       PrintError(error);
       return -1;
   }


   if (numCameras == 0){
        cerr << "Could not find any FLIR cameras connected. Exiting" << endl;
        exit(-1);
   }else
       std::cout << "There N=" << numCameras << " Cameras connected " << std::endl;

    ///Connect to 1st Camera
    if (connectCam(busMgr,camA,0,fmt7InfoA) == 1)
    {
        ///Set mode and Print Camera Info / In/Out Camera Fps Setting - Setting And Actual
        SetCam(&camA,f7,k_fmt7ModeA,k_fmt7PixFmt,fFrameRateA,fshutterA);
        ///Print CamA Mode Information
        camA.GetVideoModeAndFrameRate(&cVidModA,&cFpsA);
        std::cout << "IDX:0 Current Camera Video Mode ////" << std::endl;
        std::cout << "Vid Mode :" << cVidModA << " Fps Mode Set: " << cFpsA << std::endl;
        // cam.SetVideoModeandFrameRate( VIDEOMODE_640x480Y8 , FRAMERATE_FORMAT7 );
         if ((k_fmt7PixFmt & fmt7InfoA.pixelFormatBitField) == 0)
           {
               cout << "Pixel format is not supported" << endl;
               return -1;
           }
    } //If COnnect To CAmA

     //Connect to 2nd Cam If Exists
     if (numCameras > 1)
     {
       cout << "Found 2nd Camera. Attempting to connect..." << std::endl;
       if (connectCam(busMgr,camB,0,fmt7InfoB) == 1)
       {
           ///Set mode and Print Camera Info / In/Out Camera Fps Setting - Setting And Actual
           SetCam(&camB,f7,k_fmt7ModeB,k_fmt7PixFmt,fFrameRateB,fshutterB);
           ///Print CamA Mode Information
           camA.GetVideoModeAndFrameRate(&cVidModB,&cFpsB);
           std::cout << "IDX:1 Current Camera Video Mode ////" << std::endl;
           std::cout << "Vid Mode :" << cVidModB << " Fps Mode Set: " << cFpsB << std::endl;

       } // If COnnected to Cam B

     }

    std::cout << "Will Record for a total of " << uiTimeOutSec << "sec" << std::endl;
    std::cout << "Min event duration " << uiEventMinDuration << " sec" << std::endl;



 //Got CAM1
    //Setup thread
    struct thread_data2 RSC_input;
    RSC_input.cam               = &camA;
    RSC_input.proc_folder       = soutFolder;
    RSC_input.display           = string(ZR_WINDOWNAME);
    RSC_input.crop              = iCrop;
    RSC_input.MaxFrameDuration =  fFrameRateA*uiduration; //Calc Max Frames given camera FPS
    RSC_input.eventtimeout     =  (uint)(uiEventMinDuration*fFrameRateA); //min duration of an event in frames
    RSC_input.eventCount        = 0;//Start Zero And IMg Detection will increment this

    //init Semaphore
    sem_init(&semImgCapCount,0,0);
    sem_init(&semImgFishDetected,0,1); //Initially In Run

    //Make Mask
    ///Draw ROI Mask
    gframeMask = cv::Mat::zeros(fmt7InfoA.maxHeight,fmt7InfoA.maxWidth,CV_8UC1);
    cv::circle(gframeMask,cv::Point(gframeMask.cols/2,gframeMask.rows/2),gframeMask.cols/2-50,CV_RGB(255,255,255),-1,CV_FILLED);


    //Rec_onDisk_SingleCamera2((void*)&RSC_input,cMaxFrames);
    //Start The Recording Thread
    if (pthread_create(&tidRec, NULL, &Rec_onDisk_SingleCamera2, (void *)&RSC_input) != 0) {
        printf("Oh Ohh... Thread for Camera recording Rec_onDisk_SingleCamera2 could not start :( \n");
        camA.StopCapture();
        camA.Disconnect();
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
    ReaderFnArgs.timeout        = uiTimeOutSec;


    CreateOutputFolder(soutFolder);

    // Frames recorded when writing the buffer are specified in bufferfile
    stringstream bufferfilename;
    bufferfilename << ReaderFnArgs.proc_folder << "/buffer.log";
    ofstream bufferfile(bufferfilename.str().c_str());

    // Creation of the thread-safe curcular buffer
    circular_buffer_ts circ_buffer(BUFFER_SIZE,ReaderFnArgs.proc_folder,&bufferfile);


    // Start rec and proc threads /////////////////////////////////////////////////////////////////////
    boost::thread T_REC(Rec_onDisk_TopCamera, std::ref(circ_buffer), std::ref(RSC_input) ) ;

    if(T_REC.joinable()){
        T_REC.join();
    }
    //boost::thread T_PROC(blob_detector_thread,std::ref(circ_buffer),std::ref(center_head));
//    // ################################################################################################

//    if(VisualStimulation_ON && run){
//        if(!Barrage->Background_ON) Barrage->VisualStimulation(RSC_input->proc_folder,run);
//        else Barrage->VisualStimulation_BG(RSC_input->proc_folder,run);
//    } else display_blobs(circ_buffer);

//    if(T_REC.joinable()){
//        T_REC.join();
//    }
//    if(T_PROC.joinable()){
//        T_PROC.join();
//    }


    ///\note
    /// cv:imshow functions need to be run from same thread - otherwise opencv hangs
    if (pthread_create(&tidDisplay, NULL, &ReadImageSeq, (void *)&ReaderFnArgs) != 0) {
        printf("Oh Ohh... Thread for Camera Display ReadImageSeq could not start :( \n");
        camA.StopCapture();
        camA.Disconnect();
        return -1;
    }


    //pthread_join(tidRec, NULL); //Wait Until Done / Join Main Thread
    pthread_join(tidDisplay, NULL); //Wait Until Done / Join Main Thread
    pthread_join(tidRec, NULL); //Wait Until Done / Let it Join Main Thread
    //pthread_kill(tidRec,SIGTERM); //Stop The recording Thread
//    char c = 0;
    //Set TimeOut
//    double tstart = (double)cv::getTickCount();
//    double t=0;
//    char c;
//    //Wait Until Time Or q pressed
//    while(c!='q' && t < uiTimeOutSec){
//        t = ((double)cv::getTickCount() - tstart)/cv::getTickFrequency();
//        //What for Quit

//        c=cv::waitKey(1000);
//    }


    sem_destroy(&semImgCapCount);
    sem_destroy(&semImgFishDetected);

    std::cout <<  std::endl << "~~ Done ~~" << std::endl;

    std::exit(EXIT_SUCCESS);
    //cam.StopCapture();
   // cam.Disconnect();





    return 0;
}
