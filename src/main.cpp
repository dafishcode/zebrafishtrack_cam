/// * \title zebraprey_cam
/// \author Konstantinos Lagogiannis 2017
/// \brief A recording controller for a FLIR Chameleon3 camera used for recording larval zebrafish behaviour at fast fps.
/// Allows for event triggered recording as optional in which it records a an event for a min of mineventduration, up to max (2min).
/// When started in this mode, an initial event is immediatelly recorded with subsequent events triggered when a blob of sufficient size (like larval zebrafish)
/// is in view.
//////
/// Uses a Circular Buffer in order to have accesss to frames preceding an event trigger.
/// Extended to be able to record from 2 cameras (camA , camB) simultaneously.
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

int main(int argc, char** argv)
{

    pthread_t tidDisplay,tidRec; //Recorder And Monitor/Display Thread

    int BUFFER_SIZE=100; //Circular Buffer

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
        "{help h usage ?    |      | print this help  message   }"
        "{outputDir         |<none>| Dir where To save output video/images and logs}"
        "{outputType        | 0    | Image Sequence 0 /Video file 1}"
        "{@crop             | 0    | ROI to crop images before save       }"
        "{camAmode Am       |1     | Mode 1 is low Res high FPS}"
        "{camBmode Bm       |0     | Mode 1 is low Res high FPS}"
        "{camAfps  Af       | 40.0| Camera capture fps and output video fps}"
        "{camBfps  Bf       | 40.0 | Camera capture fps and output video fps}"
        "{camAshutter As    | 3.0  | Camera shutter speed - set to 3ms }"
        "{camBshutter Bs    | 3.0   | Camera shutter speed - set to 3ms }"
        "{camAIdx Cid       |0     | Flycap Idx identifying camera that will be the main event triggered camera (choose Bottom camera)}"
        "{camBIdx Cid       |1     | Flycap Idx to identify the camera hardware (choose idx of top camera)}"
        "{eventtimeout e    |240   | Max event recording duration (sec), new event is created after timeout }"
        "{timeout t         |600   | Max recording time (sec), stops the recording process (= 10 mins)  }"
        "{mineventduration d |30 | min duration (sec) of event once recording is triggered on CamA (1st event is autotriggered) }"
        "{timestamp ts      |true | use time stamp       }"
        "{motiontriggered e  |false| Event Capture: Trigger recording  only when something large is moving in the scene (a fish) / Non-Conitnuous recording  }"
        ;

    cv::CommandLineParser parser(argc, argv, keys);

    stringstream ssMsg;

    ssMsg<<"Zebrafish Experiments Recording videp from the Cameleon3 FLIR camera."<<endl;
    ssMsg<<"Upgraded to Dual camera capture (2021)."<<endl;
    ssMsg<<"--------------------------"<<endl;
    ssMsg<<"Author : Kontantinos Lagogiannis 2017"<<endl;
    ssMsg<<"./Rec_onDisk <MODE=1> <outfolder> <outputFormat='avi'> <crop=0> <camAfps=450> <timeout=600sec> <timestamp=false>"<<endl;
    ssMsg<<"(note: folder is automatically generated when absent)"<<endl;

    parser.about(ssMsg.str() );

    if (parser.has("help"))
    {
        parser.printMessage();

        return 0;
    }


    Mode k_fmt7ModeA     = MODE_1; //Default
    Mode k_fmt7ModeB     = MODE_0; //Default

    int camAIdx          = (int)parser.get<int>("camAIdx");
    int camBIdx          = (int)parser.get<int>("camBIdx");
    k_fmt7ModeA          = (Mode)parser.get<int>("camAmode");
    k_fmt7ModeB          = (Mode)parser.get<int>("camBmode");
    float fFrameRateA    = parser.get<float>("camAfps");
    float fshutterA      = parser.get<float>("camAshutter");
    float fFrameRateB    = parser.get<float>("camBfps");
    float fshutterB      = parser.get<float>("camBshutter");

    int iCrop                   = parser.get<int>("@crop");
    string soutFolder           = parser.get<string>("outputDir");
    int ioutputType             = parser.get<int>("outputType");
    bool use_time_stamp         = parser.has("timestamp");
    // Read User set Recording durations for each event and the total recording
    uint uieventminduration         = parser.get<uint>("mineventduration");
    uint uimaxeventduration_sec     = parser.get<uint>("eventtimeout");
    uint uimaxsessionduration_sec   = parser.get<uint>("timeout");

    bool eventtriggered         = parser.get<bool>("motiontriggered");

    // When No Event Triggering - Make Whole Recording Session a Single Event
    if (!eventtriggered)
        uieventminduration = uimaxeventduration_sec = uimaxsessionduration_sec;

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
       std::cout << "There N=" << numCameras << " cameras connected " << std::endl;

    ///Connect to 1st Camera
    if (connectCam(busMgr,camA,camAIdx,fmt7InfoA) == 1)
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
        //If we can Connect To CAmA
      }  else {
            cerr << " Failed to connect to camera 1" << std::endl;
             exit(-1);
      }


     //Connect to 2nd Cam If Exists
     if (numCameras > 1)
     {
       cout << "Found 2nd Camera. Attempting to connect..." << std::endl;
       if (connectCam(busMgr,camB,camBIdx,fmt7InfoB) == 1)
       {
           ///Set mode and Print Camera Info / In/Out Camera Fps Setting - Setting And Actual
           SetCam(&camB,f7,k_fmt7ModeB,k_fmt7PixFmt,fFrameRateB,fshutterB);
           ///Print CamA Mode Information
           camA.GetVideoModeAndFrameRate(&cVidModB,&cFpsB);
           std::cout << "IDX:1 Current Camera Video Mode ////" << std::endl;
           std::cout << "Vid Mode :" << cVidModB << " Fps Mode Set: " << cFpsB << std::endl;
        // If COnnected to Cam B
       } else {
          cerr << " Failed to connect to camera" << std::endl;
           exit(-1);
       }

       // Make OutputFolder camB
       CreateOutputFolder(soutFolder + "/camB/");
     }

    std::cout << "Will Record for a total of " << uimaxsessionduration_sec << "sec" << std::endl;
    std::cout << "Min event duration " << uieventminduration << " sec" << std::endl;

    // Make OutputFolder and camA subfolder
    CreateOutputFolder(soutFolder);
    CreateOutputFolder(soutFolder + "/camA/");

    // Define the codec and create VideoWriter object
    //To save image sequence use a proper filename (eg. img_%02d.jpg) and fourcc=0 OR fps=0. Use uncompressed image format (eg. img_%02d.BMP) to save raw frames.
    //('M','J','P','G')
    cv::VideoWriter* pVideoWriter;
    string stroutputfile = soutFolder;
    if (ioutputType == 1){
         stroutputfile = stroutputfile.append("/camA/exp_video.avi");
         pVideoWriter = new cv::VideoWriter(stroutputfile, CV_FOURCC('Y','8','0','0'), fFrameRateA, cv::Size(1024,1024), false); //initialize the VideoWriter object
    }else{
        stroutputfile = stroutputfile.append("/camA/img_%07d.bmp");
        pVideoWriter = new cv::VideoWriter(stroutputfile, 0, 0, cv::Size(1024,1024), false); //initialize the VideoWriter object
      }


    //Got CAM1
    //Setup thread Event Triggered Cam A
    struct camera_thread_data RSC_input_camA;
    RSC_input_camA.cam               = &camA;
    RSC_input_camA.proc_folder       = soutFolder + "/camA/";
    RSC_input_camA.display           = string("Bottom display (camA)");
    RSC_input_camA.crop              = iCrop;
    RSC_input_camA.MaxEventFrames =  fFrameRateA*uimaxeventduration_sec; //Calc Max Frames given camera FPS
    RSC_input_camA.eventtimeout     =  (uint)(uieventminduration*fFrameRateA); //min duration of an event in frames
    RSC_input_camA.eventCount        = 0;//Start Zero And IMg Detection will increment this

    // Frames recorded when writing the buffer are specified in bufferfile
    stringstream bufferfilename;
    bufferfilename << RSC_input_camA.proc_folder << "/../circ_buffer_camA.log";
    ofstream bufferfile(bufferfilename.str().c_str());

    // thread-safe circular buffer CAM A allows saving a Number of images prior to an Event being Triggered
    circular_buffer_ts circ_buffer_camA(BUFFER_SIZE,RSC_input_camA.proc_folder,&bufferfile);
    RSC_input_camA.pcircbuffer = &circ_buffer_camA;

    //init Semaphore
    sem_init(&semImgCapCount,0,0);
    sem_init(&semImgFishDetected,0,1); //Initially In Run

    //Make Mask
    ///Draw ROI Mask
    gframeMask = cv::Mat::zeros(fmt7InfoA.maxHeight,fmt7InfoA.maxWidth,CV_8UC1);
    cv::circle(gframeMask,cv::Point(gframeMask.cols/2,gframeMask.rows/2),gframeMask.cols/2-50,CV_RGB(255,255,255),-1,CV_FILLED);
    //Rec_onDisk_SingleCamera2((void*)&RSC_input,cMaxFrames);


//    //Start The Recording Thread - Continuous
   if (pthread_create(&tidRec, NULL, &rec_onDisk_camA, (void *)&RSC_input_camA) != 0) {
        printf("Oh Ohh... Thread for Camera recording Rec_onDisk_SingleCamera2 could not start :( \n");
        camA.StopCapture();
        camA.Disconnect();
        return -1;
    }
    //cv::namedWindow(RSC_input.display,cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    //cv::resizeWindow(RSC_input.display, fmt7Info.maxHeight,fmt7Info.maxWidth);



    //Setup thread Constantly On Cam B
    struct camera_thread_data RSC_input_camB;
    RSC_input_camB.cam               = &camB;
    RSC_input_camB.proc_folder       = soutFolder + "/camB/";
    RSC_input_camB.display           = string("Top display (camB)");
    RSC_input_camB.crop              = iCrop;
    RSC_input_camB.MaxEventFrames =  fFrameRateB*uimaxsessionduration_sec; //Calc Max Frames given camera FPS
    RSC_input_camB.eventtimeout     =  (uint)(uimaxsessionduration_sec*fFrameRateB); //min duration of an event in frames
    RSC_input_camB.eventCount       = 0;//CAm B creates 1 event

    circular_buffer_ts circ_buffer_camB(5,RSC_input_camB.proc_folder,&bufferfile);
    RSC_input_camB.pcircbuffer = &circ_buffer_camB;

    /// Start Top Camera Recording Thread / BOOST Thread Version
    //using boost thread here as join is blocking
    boost::thread T_REC_B(rec_onDisk_camB, boost::ref(RSC_input_camB) ) ;


    struct observer_thread_data ReaderFnArgs;
    ReaderFnArgs.mode           = 0; //How to Save File Mode
    ReaderFnArgs.proc_folder    = soutFolder;
    ReaderFnArgs.prefix0        = fixedLengthString(1,3);
    ReaderFnArgs.windisplay     = RSC_input_camA.display;
    ReaderFnArgs.format         = ZR_OUTPICFORMAT;
    ReaderFnArgs.timeout        = uimaxsessionduration_sec;
    ReaderFnArgs.pcircbufferA = &circ_buffer_camA; //So to Trigger Dumping of event Antecedent frames
    ReaderFnArgs.pcircbufferB = &circ_buffer_camB; //So to Trigger Dumping of event Antecedent frames

    ///\note cv:imshow functions need to be run from same thread - otherwise opencv hangs
    if (pthread_create(&tidDisplay, NULL, &camViewEventTrigger, (void *)&ReaderFnArgs) != 0) {
        printf("Oh Ohh... Thread for Camera Display ReadImageSeq could not start :( \n");
        camA.StopCapture();
        camA.Disconnect();
        return -1;
    }

    //pthread_join(tidRec, NULL); //Wait Until Done / Join Main Thread

    if(T_REC_B.joinable()) //Cam B
        T_REC_B.join();


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
    delete(pVideoWriter);
    std::cout <<  std::endl << "~~ Done ~~" << std::endl;

    std::exit(EXIT_SUCCESS);
    //cam.StopCapture();
   // cam.Disconnect();




    return 0;
}

