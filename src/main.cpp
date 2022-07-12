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
///
/// \note Press r to manually trigger the recording of a new event
/// \note Press s to start the timer and overall recording of the experiment
///
/// \attention When streaming with multiple devices on Linux, when the program calls start_cameras the program will fail with errors such as an error for libusb_submit_transfer and similar-looking errors.
///  Diving deeper, it becomes clear that a function within libusb is failing, returning ENOMEM indicating that the kernel is out of memory.
/// The problem is that there's a setting (readable at /sys/module/usbcore/parameters/usbfs_memory_mb) that limits the amount of memory available for USB IO.
///  However, some applications that require more intensive usage of that memory often use more than that. As documented at OpenKinect/libfreenect2#97, there is a fix for this: allocate more memory. I tried it with 32mb instead of 16 and it seems to be working (I ran into issues the first time I tested it but they seemed unrelated; documenting in case someone else runs into this).
///  To fix on AMD64, using 32 mb instead of 16,
/// Edit /etc/default/grub, replacing the line that says GRUB_CMDLINE_LINUX_DEFAULT="quiet splash" with GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=32"
/// Run sudo update-grub
/// Restart the computer
/// \bug Starts new events even in Continuous recording mode
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

#include"../include/circular_video_buffer_ts.h"
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
//pthread_cond_t cond;
//pthread_mutex_t gmutex_lock    = PTHREAD_MUTEX_INITIALIZER;
bool bImgCaptured       = false;/// Global Flag indicating new Image Has been captured by camera
bool gbeventtriggered = true; ///Global Flag whether recorder is operating in the event/motion trigger or continuous recording mode

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

    F7 f7_camA;
    F7 f7_camB;
    /// Handle Command Line Parameters //
    const cv::String keys =
        "{help h usage ?    |      | print this help  message   }"
        "{outputDir         |<none>| Dir where To save output video/images and logs}"
        "{outputType o      | 2    | 0->Image Sequence , 1-> Uncompressed Video file, 2-> Vid MPEG Compression, 3-> XVID Compression}"
        "{@crop             | 0    | ROI to crop images before save       }"
        "{camAmode Am       | 0     | Mode 1 is low Res high FPS}"
        "{camBmode Bm       | 0     | Mode 1 is low Res high FPS}"
        "{camAfps  Af       | 60.0| Camera capture fps and output video fps}"
        "{camBfps  Bf       | 60.0 | Camera capture fps and output video fps}"
        "{camAshutter As    | 6.0  | Camera shutter speed - set to 3ms }"
        "{camBshutter Bs    | 6.0   | Camera shutter speed - set to 3ms }"
        "{camAIdx Cid       |0     | Flycap Idx identifying camera that will be the main event triggered camera }"
        "{camBIdx Cid       |1     | Flycap Idx to identify the camera hardware (choose idx of top camera)}"
        "{eventtimeout e    |240   | Max event recording duration (sec), new event is created after timeout }"
        "{timeout t         |600   | Max recording time (sec), stops the recording process (= 10 mins)  }"
        "{mineventduration d |30   | min duration (sec) of event once recording is triggered on CamA (1st event is autotriggered) }"
        "{timestamp ts       |true | use time stamp       }"
        "{motiontriggered e  |false| Event Capture: Trigger recording  only when something large is moving in the scene (a fish) / Non-Conitnuous recording  }"
        "{dualCam            |false| Record from 2 cameras simulteneously (Dual-View Experiment Mode }"
        ;

    cv::CommandLineParser parser(argc, argv, keys);

    stringstream ssMsg;

    ssMsg<<"Camera controller for Zebrafish Experiments using the Cameleon3 FLIR camera."<<endl;
    ssMsg<<"V1.1 Upgraded to Dual-camera capture (2021)."<<endl;
    ssMsg<<"V1.2 Upgraded to direct video export (2022)."<<endl;
    ssMsg<<"--------------------------" << endl;
    ssMsg<<"Author : Kontantinos Lagogiannis 2017"<<endl;
    ssMsg<<"./Rec_onDisk <MODE=1> <outfolder> <outputFormat='avi'> <crop=0> <camAfps=60> <timeout=600sec> <timestamp=false>"<<endl;
    ssMsg<<"(note: folder is automatically generated when absent)"<<endl;
    ssMsg<<" * Press s to start recording until timeout exceeded "<< endl;
    ssMsg<<" * In event trigger mode, press r to manually trigger an event "<< endl;

    parser.about(ssMsg.str() );

    if (parser.has("help"))
    {
        parser.printMessage();
        exit(0);
        return 0;

    }


    Mode k_fmt7ModeA     = MODE_0; //Default
    Mode k_fmt7ModeB     = MODE_0; //Default

    int camAIdx          = (int)parser.get<int>("camAIdx");
    int camBIdx          = (int)parser.get<int>("camBIdx");
    k_fmt7ModeA          = (Mode)parser.get<int>("camAmode");
    k_fmt7ModeB          = (Mode)parser.get<int>("camBmode");
    float fFrameRateA    = parser.get<float>("camAfps");
    float fshutterA      = parser.get<float>("camAshutter");
    float fFrameRateB    = parser.get<float>("camBfps");
    float fshutterB      = parser.get<float>("camBshutter");
    bool bdualCam        = parser.get<bool>("dualCam");
    int iCrop                   = parser.get<int>("@crop");
    string soutFolder           = parser.get<string>("outputDir");
    outputType ioutputType      = (outputType)parser.get<int>("outputType");
    bool use_time_stamp         = parser.has("timestamp");
    // Read User set Recording durations for each event and the total recording
    uint uieventminduration         = parser.get<uint>("mineventduration");
    uint uimaxeventduration_sec     = parser.get<uint>("eventtimeout");
    uint uimaxsessionduration_sec   = parser.get<uint>("timeout");

    gbeventtriggered            = parser.get<bool>("motiontriggered");

    // When No Event Triggering - Make Whole Recording Session a Single Event
    if (!gbeventtriggered)
    {
        uieventminduration = uimaxeventduration_sec = uimaxsessionduration_sec;
        cout << "CONTINUOUS Recording mode" << std::endl;
    }else
        cout << "~ MOTION-EVENT triggered ~~ min Event duration " << uieventminduration << "sec, max duration: " <<  uimaxeventduration_sec << std::endl;

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
   unsigned int numCameras,iCamSN;
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

   busMgr.GetCameraSerialNumberFromIndex(camAIdx,&iCamSN);
   std::cout << "-->> User selected camIdx:" << camAIdx << " with S/N:" << iCamSN << "  <<--" << std::endl;

    ///Connect to 1st Camera
    if (connectCam(busMgr,camA,camAIdx,fmt7InfoA) == 1)
    {
        ///Set mode and Print Camera Info / In/Out Camera Fps Setting - Setting And Actual
        SetCam(&camA,f7_camA,k_fmt7ModeA,k_fmt7PixFmt,fFrameRateA,fshutterA);
        ///Print CamA Mode Information
        camA.GetVideoModeAndFrameRate(&cVidModA,&cFpsA);
        std::cout << "IDX:" << camAIdx << "  Current Camera Video Mode ////" << std::endl;
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


    std::cout << "Will Record for a total of " << uimaxsessionduration_sec << "sec" << std::endl;
    std::cout << "Min event duration " << uieventminduration << " sec" << std::endl;

    // Make OutputFolder and camA subfolder
    CreateOutputFolder(soutFolder);
    CreateOutputFolder(soutFolder + "/cam_" + std::to_string(camAIdx) + "/" );

    // Define the codec and create VideoWriter object
    //To save image sequence use a proper filename (eg. img_%02d.jpg) and fourcc=0 OR fps=0. Use uncompressed image format (eg. img_%02d.BMP) to save raw frames.
    //('M','J','P','G')
    //cv::VideoWriter* pVideoWriterA;
    string stroutputfile = soutFolder;
//    if (ioutputType == zCam_SEQIMAGES){
//       stroutputfile = stroutputfile.append("/camA/img_%09d.bmp");
//       pVideoWriterA = new cv::VideoWriter(stroutputfile, 0, 0, cv::Size(640,512), false); //initialize the VideoWriter object
//     }
//    if (ioutputType == 1){
//         stroutputfile = stroutputfile.append("/camA/exp_video_y800.avi");
//         pVideoWriterA = new cv::VideoWriter(stroutputfile, cv::VideoWriter::fourcc('Y','8','0','0') , fFrameRateA, cv::Size(640,512), false); //initialize the VideoWriter object //('Y','8','0','0') cv::VideoWriter::fourcc('M','J','P','G') cv::VideoWriter::fourcc('X','V','I','D')
//    }
//     if (ioutputType == 2){
//          stroutputfile = stroutputfile.append("/camA/exp_video_mpeg.mp4");
//          pVideoWriterA = new cv::VideoWriter(stroutputfile, cv::VideoWriter::fourcc('M','J','P','G') , fFrameRateA, cv::Size(640,512), false); //initialize the VideoWriter object cv::VideoWriter::fourcc('Y','8','0','0')
//     }
//     if (ioutputType == 3){
//          stroutputfile = stroutputfile.append("/camA/exp_video_xvid.mp4");
//          pVideoWriterA = new cv::VideoWriter(stroutputfile, cv::VideoWriter::fourcc('X','V','I','D') , fFrameRateA, cv::Size(640,512), false); //initialize the VideoWriter object cv::VideoWriter::fourcc('Y','8','0','0')
//     }



    //Got CAM1
    //Setup thread Event Triggered Cam A
    struct camera_thread_data RSC_input_camA;
    RSC_input_camA.cam               = &camA;
    RSC_input_camA.proc_folder       = soutFolder + "/cam_" + std::to_string(camAIdx) + "/";
    RSC_input_camA.display           = string("Bottom display (camA)");
    RSC_input_camA.crop              = iCrop;
    RSC_input_camA.MaxEventFrames =  fFrameRateA*uimaxeventduration_sec; //Calc Max Frames given camera FPS
    RSC_input_camA.MinEventframes     =  (uint)(uieventminduration*fFrameRateA); //min duration of an event in frames
    RSC_input_camA.eventCount        = 0;//Start Zero And IMg Detection will increment this

    // Frames recorded when writing the buffer are specified in bufferfile
    stringstream bufferfilename;
    bufferfilename << RSC_input_camA.proc_folder << "/../circ_buffer_camA.log";
    ofstream bufferfile(bufferfilename.str().c_str());

    // thread-safe circular buffer CAM A allows saving a Number of images prior to an Event being Triggered
    ///\todo Initialize With VideoWriter poijnter - circBuff Needs to be the only video Output to disk.
    circular_video_buffer_ts circ_buffer_camA(BUFFER_SIZE,RSC_input_camA.proc_folder,&bufferfile,ioutputType,fFrameRateA);
    RSC_input_camA.pcircbuffer = &circ_buffer_camA;

    //init Semaphore
    sem_init(&semImgCapCount,0,0);
    sem_init(&semImgFishDetected,0,0); //Initially In Wait - Not Run

    //Make Mask
    ///Draw ROI Mask
    gframeMask = cv::Mat::zeros(fmt7InfoA.maxHeight,fmt7InfoA.maxWidth,CV_8UC1);
    cv::circle(gframeMask,cv::Point(gframeMask.cols/2,gframeMask.rows/2),gframeMask.cols/2-50,CV_RGB(255,255,255),-1,cv::FILLED);
    //Rec_onDisk_SingleCamera2((void*)&RSC_input,cMaxFrames);


    //cv::namedWindow(RSC_input.display,cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    //cv::resizeWindow(RSC_input.display, fmt7Info.maxHeight,fmt7Info.maxWidth);


    //    //Start The Recording Thread - Continuous
    //camA.StartCapture();
   if (pthread_create(&tidRec, NULL, &rec_onDisk_camA, (void *)&RSC_input_camA) != 0) {
        printf("Oh Ohh... Thread for Camera recording Rec_onDisk_SingleCamera2 could not start :( \n");
        camA.StopCapture();
        camA.Disconnect();
        return -1;
    }
   /// END OF CAM A //


   /// CAM B Repeat as Above//
   //cv::VideoWriter* pVideoWriterB = 0;

   //Connect to 2nd Cam If Exists and If User wants DUAL CAMERA Mode
   if (numCameras > 1 && bdualCam)
   {
     cout << "Found 2nd Camera. Attempting to connect..." << std::endl;
     if (connectCam(busMgr,camB,camBIdx,fmt7InfoB) == 1)
     {
         ///Set mode and Print Camera Info / In/Out Camera Fps Setting - Setting And Actual
         SetCam(&camB,f7_camB,k_fmt7ModeB,k_fmt7PixFmt,fFrameRateB,fshutterB);
         ///Print CamA Mode Information
         camB.GetVideoModeAndFrameRate(&cVidModB,&cFpsB);
         std::cout << "IDX:"<< camBIdx << " Current Camera Video Mode ////" << std::endl;
         std::cout << "Vid Mode :" << cVidModB << " Fps Mode Set: " << cFpsB << std::endl;
      // If COnnected to Cam B
     } else {
        cerr << " Failed to connect to camera B" << std::endl;
         exit(-1);
     }

     // Make OutputFolder camB
         CreateOutputFolder(soutFolder + "/cam_" + std::to_string(camBIdx) + "/" );

//     string stroutputfile = soutFolder;
//     if (ioutputType == 1){
//          stroutputfile = stroutputfile.append("/camB/exp_video.avi");
//          pVideoWriterB = new cv::VideoWriter(stroutputfile, cv::VideoWriter::fourcc('Y','8','0','0') , fFrameRateA, cv::Size(640,512), true); //initialize the VideoWriter object cv::VideoWriter::fourcc('Y','8','0','0')
//     }
//     if (ioutputType == 2){
//          stroutputfile = stroutputfile.append("/camB/exp_video_mpeg.avi");
//          pVideoWriterB = new cv::VideoWriter(stroutputfile, cv::VideoWriter::fourcc('X','V','I','D') , fFrameRateA, cv::Size(640,512), true); //initialize the VideoWriter object cv::VideoWriter::fourcc('Y','8','0','0')
//     }

//     if (ioutputType == 0){
//         stroutputfile = stroutputfile.append("/camB/img_%07d.bmp");
//         pVideoWriterB = new cv::VideoWriter(stroutputfile, 0, 0, cv::Size(1024, 1024), false); //initialize the VideoWriter object
//       }
   }


    //Setup thread Constantly On Cam B
    struct camera_thread_data RSC_input_camB;
    RSC_input_camB.cam               = &camB;
    RSC_input_camB.proc_folder       = soutFolder + "/cam_" + std::to_string(camBIdx) + "/" ;
    RSC_input_camB.display           = string("Top display (camB)");
    RSC_input_camB.crop              = iCrop;
    RSC_input_camB.MaxEventFrames     =  fFrameRateB*uimaxsessionduration_sec; //Calc Max Frames given camera FPS
    RSC_input_camB.MinEventframes     =  (uint)(uimaxsessionduration_sec*fFrameRateB); //min duration of an event in frames
    RSC_input_camB.eventCount         = 0;//IMg Detection will increment this

    circular_video_buffer_ts circ_buffer_camB(5,RSC_input_camB.proc_folder,&bufferfile,ioutputType,fFrameRateB);
    RSC_input_camB.pcircbuffer = &circ_buffer_camB;

    /// Start Top Camera Recording Thread / BOOST Thread Version
    //using boost thread here as join is blocking

    struct observer_thread_data ReaderFnArgs;
    ReaderFnArgs.mode           = 0; //How to Save File Mode
    ReaderFnArgs.proc_folder    = soutFolder;
    ReaderFnArgs.prefix0        = fixedLengthString(1,3);
    ReaderFnArgs.windisplay     = RSC_input_camA.display + RSC_input_camA.proc_folder;
    ReaderFnArgs.format         = ZR_OUTPICFORMAT;
    ReaderFnArgs.timeout        = uimaxsessionduration_sec;
    ReaderFnArgs.pcircbufferA = &circ_buffer_camA; //So to Trigger Dumping of event Antecedent frames
    ReaderFnArgs.pcircbufferB = &circ_buffer_camB; //So to Trigger Dumping of event Antecedent frames




    if (pthread_create(&tidDisplay, NULL, &camViewEventTrigger, (void *)&ReaderFnArgs) != 0) {
        printf("Oh Ohh... Thread for Camera Display ReadImageSeq could not start :( \n");
        camA.StopCapture();
        camA.Disconnect();
        return -1;
    }


    //pthread_join(tidRec, NULL); //Wait Until Done / Join Main Thread
    boost::thread *T_REC_B = 0;
    if (bdualCam){
        if (camB.IsConnected())
        {
            T_REC_B = new boost::thread(rec_onDisk_camB, boost::ref(RSC_input_camB) ) ;
            camB.StartCapture();
        }
     }


    //if(T_REC_B.joinable()) //Cam B
    //    T_REC_B.join();
    //need the monitor both threads to join/ exit for the programme to stop Normally - Otherwise Mutex may be locked
    pthread_join(tidDisplay, NULL); //Wait Until Done / Join Main Thread
    pthread_join(tidRec, NULL); //Wait Until Done / Let it Join Main Thread


    if (T_REC_B)
        T_REC_B->detach();

    pthread_detach(tidRec);
    pthread_detach(tidDisplay);
    //usleep(5000); //Pause For all activity to finish


    sem_destroy(&semImgCapCount);
    sem_destroy(&semImgFishDetected);
    //delete(pVideoWriterA);
    std::cout <<  std::endl << "~~ Done ~~" << std::endl;

    /// Disconnect cameras //
    if (camA.IsConnected()){
        camA.StopCapture();
        camA.Disconnect();
    }
    if (numCameras > 1){
        if (camB.IsConnected()){
             camB.StopCapture();
             camB.Disconnect();
        }
    }

    std::exit(EXIT_SUCCESS);
    //cam.StopCapture();
   // cam.Disconnect();




    return 0;
}


