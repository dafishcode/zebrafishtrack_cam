#include <string.h>
#include <iostream>
#include <iomanip> //For setprecision
#include <sstream>
#include<fstream>
#include<signal.h>
#include <sys/stat.h> //for Mkdir
#include <sys/types.h> //Not Necessary for Mkdir

#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <limits.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <FlyCapture2.h>
#include <Utilities.h>
#include <CameraBase.h>


#include"../include/util.h"
#include"../include/circular_video_buffer_ts.h"

using namespace std;
using namespace FlyCapture2;


bool run    =true;
boost::mutex mtx;

bool gbtimeoutreached =  false; //Global Flag used from readImSeq to stop RecOnDisc from Starting a new recording
bool gbRecStarted     = false; //Global Flag - User has started the Experiment Recording And Overall Timer has initiated
bool gbEventRecording = false; //Global Variable Indicating Files are saved to disk
bool gbrun            = true; //Global Flag CAn Be Altered by Signal Handler
unsigned int gFrameRate; //Global Var Holding FrameRate Read from Camera

extern uint uiEventMinDuration; //min duration for an event (if nothing shows up in view)
extern bool gbeventtriggered; //Continuous Recording - or motion triggered/event

void my_handler(int sig){
       cout<<endl<<"Recording stopped."<<endl;
       gbrun=false;
}

// Attempts to connect to Camera And Check If Cam supports A particular MODE
int connectCam(BusManager& busMgr,Camera& cam,int camIDX,Format7Info& fmt7Info)
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


void on_mouse(int event, int x, int y, int flags, void* p){
    int ws=50;
    ioparam* param=(ioparam*)p;
    param->center.x=x;
    param->center.y=y;
    param->pt1.x=x-ws; param->pt1.y=y-ws;
    param->pt2.x=x+ws; param->pt2.y=y+ws;
    if  ( event == cv::EVENT_LBUTTONDOWN ){
	    param->status=true;
    } 
    
    if (event==cv::EVENT_LBUTTONUP ){
            param->status=false;
    }
	   
}

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PrintFormat7Capabilities(Format7Info fmt7Info)
{
    std::cout << "----------------" << std::endl;
    std::cout << "Max image pixels: (" << fmt7Info.maxWidth << ", "
         << fmt7Info.maxHeight << ")" << endl;
    std::cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", "
         << fmt7Info.imageVStepSize << ")" << endl;
    std::cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", "
         << fmt7Info.offsetVStepSize << ")" << endl;
    std::cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
    std::cout << "MODE:" << fmt7Info.mode << endl;
    std::cout << "Packet Size:" << fmt7Info.packetSize << endl;

    float targetFrameRate = (float)(fmt7Info.packetSize)/(fmt7Info.maxWidth*fmt7Info.maxHeight*8.0);
    std::cout << "Calculated Frame Rate is:" << fixed << targetFrameRate << std::endl;


    std::cout << "----------------" << std::endl;
}

void PrintCameraInfo(CameraInfo *pCamInfo)
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Color - "<< pCamInfo->isColorCamera <<endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl;

}

/// Remove Auto from Shutter and FrameRate
/// Set to Fixed/Desired Framerate (On / No Auto) and shutter speed (No Auto).
void SetCam(Camera *cam, F7 &f7, const Mode k_fmt7Mode, const PixelFormat k_fmt7PixFmt, float& pfFrameRate,float& fshutter){

    //Calculation from KB articleshttp://digital.ni.com/public.nsf/allkb/ED092614FCCC900D86256D8D004A3B0C
    //TransferredFramesPerSecond = (BytesPerPacket * 8000) / (ImageWidth * ImageHeight * BytesPerPixel).
    assert(k_fmt7PixFmt == PIXEL_FORMAT_RAW8); //Assume 8 bit raw data output of sensor
   //float targetFrameRate = (float)(f7.fmt7Info.packetSize)/(f7.fmt7Info.maxWidth*f7.fmt7Info.maxHeight*8.0);
    //For 300 Fps then PacketSize Should be 92160 bytes
    Error error;


    CameraInfo cInfo;
    cam->GetCameraInfo(&cInfo);
    PrintCameraInfo(&cInfo);
    std::cout << "Get Camera Config:" << std::endl;
    error = cam->GetConfiguration(&(f7.config));
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return ;
    }
    // Set the number of driver buffers used to 10.
    f7.config.numBuffers = 10;

    // Set the camera configuration
    cam->SetConfiguration(&(f7.config));

    f7.fmt7Info.mode=k_fmt7Mode;
    error = cam->GetFormat7Info(&(f7.fmt7Info), &(f7.supported));
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return ;
    }
	PrintFormat7Capabilities(f7.fmt7Info);

    // Check If Cropping to smaller Square Frame //

    f7.fmt7ImageSettings.mode = k_fmt7Mode;
    f7.fmt7ImageSettings.offsetX = 0;
    f7.fmt7ImageSettings.offsetY = 0;
    f7.fmt7ImageSettings.width = f7.fmt7Info.maxWidth;
    f7.fmt7ImageSettings.height = f7.fmt7Info.maxHeight;
    f7.fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

    std::cout << "Validating Format7 Settings..." << std::endl;
    error = cam->ValidateFormat7Settings(&(f7.fmt7ImageSettings), &(f7.valid), &(f7.fmt7PacketInfo));
    if (error != PGRERROR_OK)
    {
           PrintError(error);
           return ;
    }
    if (!f7.valid)
     {
         // Settings are not valid
         std::cout << "Format7 settings are not valid" << std::endl;
         return ;
     }

    error = cam->SetFormat7Configuration(&(f7.fmt7ImageSettings), f7.fmt7PacketInfo.recommendedBytesPerPacket);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return ;
    }

    ///SHUTTER - FRAME RATE CONTROL
    // Check if the camera supports the FRAME_RATE property
    PropertyInfo propInfo;
    propInfo.type = FRAME_RATE;
    error = cam->GetPropertyInfo(&propInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }

    /// Require Timestamp from camera
    EmbeddedImageInfo imageInfo;
    imageInfo.timestamp.onOff = true;
    error = cam->SetEmbeddedImageInfo(&imageInfo);
    if (error != PGRERROR_OK)
    {
        delete cam;
        PrintError(error);
        return;
    }


    if (propInfo.present == true)
     {
         // Turn off frame rate
         Property prop;
         prop.type = FRAME_RATE;
         error = cam->GetProperty(&prop);
         if (error != PGRERROR_OK)
         {
             PrintError(error);
             return ;
         }

         prop.autoManualMode = false;
         prop.onOff = true;
         prop.absControl = true;
         prop.absValue = pfFrameRate;
         error = cam->SetProperty(&prop);
         if (error != PGRERROR_OK)
         {
             PrintError(error);
             return ;
         }

         std::cout << "FrameRate set to to " << fixed << prop.absValue << " fps" << std::endl;
    } //If Frame Rate Property Exists

     // Set the shutter property of the camera
    Property prop;
    prop.type = SHUTTER;
    error = cam->GetProperty(&prop);
    if (error != PGRERROR_OK)
    {
         PrintError(error);
         return ;
    }

     prop.autoManualMode = false;
     prop.absControl = true;

     float k_shutterVal = fshutter;//3ms Shutter speed

     prop.absValue = k_shutterVal;

     error = cam->SetProperty(&prop);
     if (error != PGRERROR_OK)
     {
         PrintError(error);
         return;
     }

     cout << "Shutter time set to " << fixed << k_shutterVal << "ms" << endl;

     /// Start capturing images
     cout << "-- STARTING CAPTURE --" << endl;
     error = cam->StartCapture();
     if (error != PGRERROR_OK)
     {
          PrintError(error);
          return ;
     }

    ///// VERIFY SETTINGS ////
     /// Retrieve frame rate property
   Property frmRate;
   frmRate.type = FRAME_RATE;
   error = cam->GetProperty(&frmRate);
   if (error != PGRERROR_OK)
   {
       std::cerr << " Camera Frame rate   " <<  std::endl;
       PrintError(error);
       return ;
   }

   std::cout << " Camera Frame rate is " << fixed  << frmRate.absValue  << " fps" << std::endl;
   pfFrameRate =  frmRate.absValue;


   /// Retrieve Shutter property
     Property propShutter;
     propShutter.type = SHUTTER;
     error = cam->GetProperty(&propShutter);
     if (error != PGRERROR_OK)
     {
         std::cerr << " Camera Shutter  " <<  std::endl;
         PrintError(error);
         return ;
     }

    std::cout << " Camera Shutter set " << fixed  << propShutter.absValue  << " ms" << std::endl;
}

// Make Directory
void CreateOutputFolder(string folder){
    struct stat sb;

    if (stat(folder.c_str(), &sb) != 0){
        const int dir_err = mkdir(folder.c_str(),0777);
        if ( dir_err == -1){

            std::cerr <<  dir_err << " Error creating directory: " << folder << " \t" <<  std::endl;
            //perror(argv[0]);
            //exit(1);
            return;
	    }
    }
}

//void Select_ROI(Camera *cam, ioparam &center, bool &recording){

//    Image rawImage;
//    cv::Mat tmp_image;
//    cv::namedWindow(ZR_WINDOWNAME,cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
//    cv::resizeWindow(ZR_WINDOWNAME, 800,800);

//    // Retrieve a single image
//    cam->RetrieveBuffer(&rawImage);

//    unsigned char* data = rawImage.GetData();
//    cv::Mat cvm(rawImage.GetRows(),rawImage.GetCols(),CV_8U,(void*)data);
    
//    char key='a';
//    ioparam tmp_center;
//    tmp_center.status=0;
//    cv::setMouseCallback(ZR_WINDOWNAME,on_mouse,&tmp_center);
//    string yn;
//    cv::Mat drawing;
//    cvm.copyTo(drawing);
    
//    while(key!='q'){
//		if(tmp_center.status){
//			cvm.copyTo(drawing);
//			cv::rectangle(drawing,tmp_center.pt1,tmp_center.pt2,0,4,8,0);
//			ostringstream info;
//            info<<"center = (" << tmp_center.center.x <<',' <<tmp_center.center.y << ")";
//            cv::imshow(ZR_WINDOWNAME,drawing);
//            cv::displayStatusBar(ZR_WINDOWNAME,info.str(),0);
//			center=tmp_center;
//        } else cv::imshow(ZR_WINDOWNAME,drawing);

//	    key=cv::waitKey(10);
//		if(key=='r'){
//            recording=true;
//		    break;
//	    }
//    }

//    cv::destroyWindow(ZR_WINDOWNAME);
//}

/// Extension to Dual Camera - That uses Circular Bufffer
void* rec_onDisk_camB(camera_thread_data &RSC_input)
{
    char tbuff [30];
    struct tm *sTm;

    bool bVerbose = false;
    mtx.lock();
    cout<<"Created main on thread "<<boost::this_thread::get_id()<<endl;
    mtx.unlock();
    int64 initial_time=cv::getTickCount();

    FlyCapture2::Error error;

    unsigned int nfrmCamB           = 0;
    unsigned char* data;

    stringstream logfilename;
    logfilename	<< RSC_input.proc_folder<<"/../time_camB.log";
    ofstream logfile(logfilename.str().c_str());
    logfile << "Frame" << '\t' << "clock_time" << '\t' << "CPU_ticks" <<'\t'<< "camts_microsec" << endl;
    Image rawImage;


    while(gbrun){
        //RSC_input->cam->FireSoftwareTrigger(false);
        RSC_input.cam->RetrieveBuffer(&rawImage);
        TimeStamp TimeStamp_fromCamera = rawImage.GetTimeStamp(); // use time stamp
        int64 TimeStamp_microseconds = TimeStamp_fromCamera.seconds*1e6+TimeStamp_fromCamera.microSeconds;
        int64 ms1 = cv::getTickCount();  // use clock ticks (less good)

        time_t now = time (0);
        sTm = gmtime (&now);
        strftime (tbuff, sizeof(tbuff), "%H:%M:%S", sTm);

        data = rawImage.GetData();
        if (rawImage.GetRows() == 0){
            //cerr << "empty image retrieved from camera" << std::endl;
            continue;
        }
        cv::Mat cvm (rawImage.GetRows(),rawImage.GetCols(),CV_8U,(void*)data);
        //  Alternativelly can copy data into existing cv img instance:
        // memcpy( img->imageData, rawImage.GetData(), data_size );

        //Crop Sides
        cv::Mat image = cvm(cv::Range(0,cvm.rows),cv::Range(65 ,cvm.cols-65));

        stringstream filename;
        filename<<RSC_input.proc_folder<<"/"<<fixedLengthString(nfrmCamB)<<".pgm";
        cv::imwrite((cv::String)filename.str().c_str(),image);

        //Add To CircBuffer
        //RSC_input.pcircbuffer->update_buffer(image,nfrmCamB,TimeStamp_fromCamera.microSeconds);

        if(bVerbose){
            mtx.lock();
            cout<<"writing: "<<nfrmCamB<<' '<<ms1<<endl;
            mtx.unlock();
        }

        logfile << nfrmCamB << '\t'<< tbuff << "\t" << ms1 <<'\t'<< TimeStamp_microseconds<< endl;

        //mtx.lock(); //If global
            nfrmCamB++;
            //cv::imshow("Fish Top camera View", image );
        //mtx.unlock();

        // Normally gbtimeoutreached will signal end - Here is an additional internal Stop Condition Based on estimated total Frame count
        if((cv::getTickCount()-initial_time)/1e9 > RSC_input.MinEventframes){
            mtx.lock();
            gbrun=false;
            mtx.unlock();
        }


    }

    // Stop capturing images
    error = RSC_input.cam->StopCapture();

    // Disconnect the camera
    error = RSC_input.cam->Disconnect();

    mtx.lock();
    cout<<"Camera B disconnected."<<endl;
    mtx.unlock();

    pthread_exit(EXIT_SUCCESS);
}



/// \brief Rec_onDisk_SingleCamera2 Captures images from Camera And Saves them to disk
/// Signals Display Function Using Semaphor
///  Called by Recording Thread To begin Camera Capture
void* rec_onDisk_camA(void *tdata)
{
    char buff[32]; //For Time Stamp
    string outfolder;
    struct tm *sTm;
    int64 nfrmCamA           = 0; //Frame Counter of Whole Recording Episode
    int64 nEventfrmCamA      = 0; //Frame Counter For current Event being Recorded
    int64 ms0                      = cv::getTickCount();
    int64 ms1                      = 0;
    double dmFps                    = 0.0;

    signal(SIGINT,my_handler);
    struct camera_thread_data * RSC_input; //Get Thread Parameters
    RSC_input =  (struct camera_thread_data *)tdata; //Cast To CXorrect pointer Type

    unsigned int cMaxEventFrames = RSC_input->MaxEventFrames;
    unsigned int fishTimeout      = RSC_input->MinEventframes;

    Error error;  

    Image rawImage;
    cv::Mat tmp_image;

    unsigned char* data;
    

    ioparam center;
    ioparam tmp_center;
    
    stringstream logfilename;
    logfilename	<< RSC_input->proc_folder <<"/../logfile_camA.log";
    ofstream logfile(logfilename.str().c_str());
    logfile << "eventNumber" << '\t' << "FrameN" << '\t'<< "clock_time" << '\t' <<"cpu_Dt_ms" << '\t' << "CPU_ticks"  << "\t" << "camts_microsec"  << std:: endl;
//	if(RSC_input->crop){
//		Select_ROI(RSC_input->cam, center , brecording);
//        if(!brecording){
//            cout<<" Quit. "<<endl;
//		}
//	} else {
//		recording=1;
//	}
		

    //string outfolder = RSC_input->proc_folder + "/" + fixedLengthString(RSC_input->eventCount,3) ;
    //RSC_input->pcircbuffer->set_outputfolder(outfolder);
    outfolder = RSC_input->proc_folder;
    cout<<"Output set to " << RSC_input->proc_folder << endl;

    ms0            = cv::getTickCount();

    int64 TimeStamp_microseconds_start = 0;
    int64 TimeStamp_microseconds = 0;
    while(gbrun){

        if (!RSC_input->cam->IsConnected())
           break;
        // Get img data from FLIR Camera
		RSC_input->cam->RetrieveBuffer(&rawImage);
        data = rawImage.GetData();

        //Convert to openCV Matrix - No Need
        cv::Mat cvm(rawImage.GetRows(),rawImage.GetCols(),CV_8U,(void*)data);

        if (!gbRecStarted){ //Not started yet - Freeze Time
             TimeStamp_microseconds_start = 0;
        }else{
            /// TimeStamps -
            // Add milliseconds timestamp to image frame
            TimeStamp tsmp_cam = rawImage.GetTimeStamp(); //count restart for microseconds
            TimeStamp_microseconds = tsmp_cam.seconds*1e6+tsmp_cam.microSeconds-TimeStamp_microseconds_start;
            if (TimeStamp_microseconds_start == 0){
                TimeStamp_microseconds_start = TimeStamp_microseconds;
                TimeStamp_microseconds = 0;
            }
            sprintf(buff,"%06.2f",((double)TimeStamp_microseconds/1000.0) );
            cv::putText(cvm,buff,cv::Point(cvm.cols-135,cvm.rows-10),cv::FONT_HERSHEY_COMPLEX,0.5,CV_RGB(50,200,50));
        }

            time_t now = time (0);
            sTm = gmtime (&now);
            strftime (buff, sizeof(buff), "%H:%M:%S", sTm);

            ///LOG: Append Frame Timing To Event Logfile
            // CPU based Frame Interval in seconds
            ms1 =  cv::getTickCount();
            double delta = (double)(ms1-ms0)/cv::getTickFrequency();
            ms0 = ms1;
            //Get Timestamp
        /// Time Stamps

        stringstream logss;
        logss << RSC_input->eventCount <<'\t'<< nfrmCamA <<'\t'<< buff <<'\t' << delta  << "\t" << ms1 << "\t" << TimeStamp_microseconds << std:: endl;

        // Pass all Frames to circ buffer
        RSC_input->pcircbuffer->update_buffer(cvm,nfrmCamA,ms1,logss.str());
        nfrmCamA++;

        //If Consumer Thread Has Consumed this Image on Display
        int value;
        sem_getvalue(&semImgCapCount, &value);
        if (value == 0) //If Last Image Has been displayed
        {
            //Clone To Global Variable
            cvm.copyTo(gframeBuffer); //gframeMask
            sem_post(&semImgCapCount); //Notify / Increment Image Count
            // Could Use Mem COpy of Data And Pass Pointer to Cv::Mat
            //gframeBuffer.data;
        }

        //Crop Image to REgion Of Interest - If Crop flag is set
        ///\todo Place This in the Camera Settings
//	    if(RSC_input->crop){
            //tmp_image=cvm(cv::Range(center.pt1.y,center.pt2.y),cv::Range(center.pt1.x,center.pt2.x));
//	    } else {
//		    tmp_image=cvm;
//	    }

        //Save to Disk If Recording is triggered
        if (gbEventRecording && gbRecStarted)
        {
            nEventfrmCamA++;

            // FlyCapture2: reference the seconds and microseconds attributes of the TimeStamp structure
            // *, where seconds is UNIX time in seconds*
            //OUtput TSstmp seconds and millisec of each frame
            logfile << RSC_input->eventCount <<'\t'<< nEventfrmCamA <<'\t'<< buff <<'\t' << delta  << "\t" << ms1 << "\t" << TimeStamp_microseconds << std:: endl;

            //stringstream filename;
            //filename << outfolder << "/"  << fixedLengthString(nEventfrmCamA) <<".pgm";
            //if(tmp_image.empty()) cout<<center.center.x<<' '<<center.center.y<<endl;
            //rawImage.Save(filename.str().c_str()); //This is SLOW!!
            //cv::imwrite(filename.str().c_str(),cvm); //THis Is fast

            RSC_input->pcircbuffer->set_recorder_state(gbEventRecording);
            RSC_input->pcircbuffer->writeNewFramesToVideostream();
            //RSC_input->pcircbuffer->writeNewFramesToImageSequence();


            dmFps += delta;
        }


        // Check Limits (Integer limits and maximum event duration limits)
        if (ULONG_MAX == nEventfrmCamA || (nEventfrmCamA == cMaxEventFrames && gbEventRecording ) )
        {   std::cerr << "Limit Of Event Period Reached / End Recording of this event.";
            mtx.lock();
             gbEventRecording = false;
            mtx.unlock();
            RSC_input->pcircbuffer->set_recorder_state(false);
            std::cout << "Event Mean Rec fps " << fixed << 1.0/(dmFps / (nEventfrmCamA+1)) << std::endl;
        }

        //FISH in ROI Event - Read in If Recording Needs to End
        int fishFlag;
        sem_getvalue(&semImgFishDetected, &fishFlag);
        //sem_wait(semImgFishDetected); //Wait Until Fish Is detected
        if (fishFlag == 0 && gbEventRecording) //there are no fish currently stop Recording
        { //In MotionEvent Triggered Enforce A wait Before Stopping Event Recording
           if (fishTimeout < 1 && gbeventtriggered)
           {
                mtx.lock();
                 gbEventRecording = false; //sTOP rECORDING aFTER tIMEOUT pERDIOD
                mtx.unlock();
                RSC_input->pcircbuffer->set_recorder_state(gbEventRecording);
                std::cout << "Event "<< RSC_input->eventCount << " Duration:" << dmFps << " sec, Mean Rec fps " << fixed << 1.0/(dmFps / (nfrmCamA+1))<< std::endl;
           }
           else
               fishTimeout--;
       }

        /// Fish Event Triggered mode - Count Event And Start Recording
        /// Check Recording Period Has not timedout
        if ((fishFlag > 0 && !gbEventRecording && !gbtimeoutreached ||
                (!gbeventtriggered && RSC_input->eventCount == 0)) &&  //If nOt In Event Trigger Then allow REC to trigger Only Once
                gbRecStarted) // User Needs to have Hit start Rec
        {

            fishTimeout         =  RSC_input->MinEventframes; //rESET tIMER
            //Make New    Sub Directory Of Next Recording
            RSC_input->eventCount++;

            //Update File Name to set to new SubDir
            outfolder = RSC_input->proc_folder + "/" + fixedLengthString(RSC_input->eventCount,3);
            RSC_input->pcircbuffer->set_outputfolder(outfolder);
            CreateOutputFolder(outfolder);
            //Set VideoWriter to new Location


            nEventfrmCamA = 0;//Restart Image Frame Count
            dmFps       = 0.0;
            ms0         = cv::getTickCount(); //Reset Timer 0 Start tm
            gbEventRecording = true; //Timeout of Event - Start A new One
            RSC_input->pcircbuffer->set_recorder_state(gbEventRecording);
        }

    } //Main Loop

    //Report Mean FPS
    std::cout << "Mean Rec fps " << fixed << 1.0/(dmFps / (nfrmCamA+1));


    // Stop capturing images
    //error = RSC_input->cam->StopCapture();
//    if (error != PGRERROR_OK)
//    {
//         PrintError(error);
//         pthread_exit(0);
//         return NULL;
//    }
	// Disconnect the camera
    error = RSC_input->cam->Disconnect();

    if (error != PGRERROR_OK)
    {
         PrintError(error);
         pthread_exit(0);
         return NULL;
    }

    ///Leave App
    pthread_exit(EXIT_SUCCESS);

    //exit(0);

    return 0;
}

/// \brief Displays Captured Image / Detects Fish Automatically and signals Recording events that timeout after a period of inactivity
/// Pressing r triggers an event trigger manually
/// Requires User to press s to initiate Recording - Otherwise Stuck in LIve view and timer not initiated
///
void *camViewEventTrigger(void* tdata){
    const double dBGLearningRate = 0.001;
    int ind=0;
    int nImgDisplayed = 0;

    cv::Mat cvimage;
    cv::Mat imS;//Scaled Down Display Image
    cv::Mat im_with_keypoints;

    cv::Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
    cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2; //MOG2 Background subtractor

    int fishFlag = 0;

    pMOG2 = cv::createBackgroundSubtractorMOG2(); //MOG2 approach


    // Detect blobs.

    std::vector<cv::KeyPoint> fishkeypoints,preykeypoints;
    std::vector<cv::KeyPoint> keypoints_in_mask;

    cv::SimpleBlobDetector::Params params;

    params.filterByCircularity  = false;
    params.filterByColor        = false;
    params.filterByConvexity    = false;

    //params.maxThreshold = 31;
    //params.minThreshold = 21;
    //params.thresholdStep = 2;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 200;
    params.maxArea = 1200;

    /////An inertia ratio of 0 will yield elongated blobs (closer to lines)
    ///  and an inertia ratio of 1 will yield blobs where the area is more concentrated toward the center (closer to circles).
    params.filterByInertia      = false;
    params.minInertiaRatio      = 0.2;
    params.maxInertiaRatio      = 0.8;

    //params.filterByInertia = true;

    // Set up the detector with default parameters.
    cv::Ptr<cv::SimpleBlobDetector> fishdetector = cv::SimpleBlobDetector::create(params);
    // Filter Prey Blobs- Count Prey//
    params.minArea = 8;
    params.maxArea = 100;
    params.filterByInertia      = true;
    params.minInertiaRatio      = 0.3;
    params.maxInertiaRatio      = 0.9;
    cv::Ptr<cv::SimpleBlobDetector> preydetector = cv::SimpleBlobDetector::create(params);

    struct observer_thread_data * Reader_input; //Get Thread Parameters

    Reader_input =  (struct observer_thread_data *)tdata; //Cast To CXorrect pointer Type

    //if(!gframeMask.empty())
    //      cv::imshow("mask", gframeMask );

    gframeMask.convertTo(gframeMask,CV_8UC1);


    char c          = '1';
    double tstart   = (double)cv::getTickCount();
    double t        = 0;
    double lastRept = 0;

    std::cout << "Will record for " << std::setprecision(4) << (Reader_input->timeout-t)/60.0 << " minutes" << std::endl;

    cout << "Reading image sequence into buffer. " << endl <<
            "* Press s to start recording and q to exit *" << endl;

    while(c!='q' && (t < (Reader_input->timeout+1) || gbEventRecording)){
        c=cv::waitKey(20);


        //if (!gbRecStarted)
        //    tstart   = (double)cv::getTickCount(); //Recording Not Started Freeze time To Now -

        t = ((double)cv::getTickCount() - tstart)/cv::getTickFrequency();

        if (t > 1 && (t-lastRept)>60 )
        {
            lastRept = t;
            if (gbRecStarted)
                std::cout << std::setprecision(4) << round((Reader_input->timeout-t)/60.0) << " minutes left" << std::endl;
            else
                std::cout << " Waiting for user to press s to start recording..." << std::endl;
        }
        sem_wait(&semImgCapCount); //Wait For Post From Camera Capture
        int value;
        sem_getvalue(&semImgCapCount, &value); //Read Current Frame
        //printf("The value of the semaphors is %d\n", value);

        ind = value+nImgDisplayed; //Semaphore Value Should be number of images N3ot Display Yet (ie Sem Incrmenets)


        /// Use Circular Buffer And Retrieve last image (replaced the global frame variable),
        cv::Mat image_from_bufferA,image_from_bufferB;
        long int camA_frame_counter, camB_frame_counter;
        Reader_input->pcircbufferA->retrieve_last(image_from_bufferA, camA_frame_counter);
        if (Reader_input->pcircbufferB)
            Reader_input->pcircbufferB->retrieve_last(image_from_bufferB, camB_frame_counter);
        //gframeBuffer.copyTo(image_from_buffer); //Old and direct  Way of obtaining last image

        if(!image_from_bufferA.empty())
        {
            nImgDisplayed++;
            //cv::destroyWindow("display");
            //cv::imshow(Reader_input->windisplay,gframeBuffer);
            //Apply MOG BG Substraction - To Remove Any artificial Triggering
            // (For example When doing the barrie experiments )
            pMOG2->apply(image_from_bufferA, fgMaskMOG2,dBGLearningRate);
            fishdetector->detect( fgMaskMOG2, fishkeypoints,gframeMask); //frameMask
            preydetector->detect( fgMaskMOG2, preykeypoints,gframeMask); //frameMask

            /// \note : Mask Is Ignored so Custom Solution Required
            //for (cv::KeyPoint &kp : keypoints)
            keypoints_in_mask.clear();
            for(int i=0;i<fishkeypoints.size();i++)
            {
                cv::KeyPoint kp = fishkeypoints[i];
                int maskVal=(int)gframeMask.at<uchar>(kp.pt);
                if (maskVal > 0)
                     keypoints_in_mask.push_back(kp);
            }


            // Draw detected blobs as red circles.
            // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
            image_from_bufferA.copyTo(image_from_bufferA,gframeMask); //mask Source Image
            // Show blobs
            cv::drawKeypoints( image_from_bufferA, keypoints_in_mask, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
            cv::drawKeypoints( image_from_bufferA, preykeypoints, im_with_keypoints, cv::Scalar(20,180,20), cv::DrawMatchesFlags::DEFAULT );
            /// SHOW Prey Count //
            cv::putText(im_with_keypoints,std::to_string(preykeypoints.size()),cv::Point(40,20),cv::FONT_HERSHEY_COMPLEX,0.5,CV_RGB(255,0,0));
            // Show Recording is ACTIVE
            if (gbEventRecording)
            {
                cv::circle(im_with_keypoints,cv::Point(20,20),5,CV_RGB(255,0,0),-1,cv::FILLED);
            }
            if (t >= Reader_input->timeout)
            {
                cv::putText(im_with_keypoints,"Time-Out reached",cv::Point(25,20),cv::FONT_HERSHEY_COMPLEX,0.8,CV_RGB(255,0,0));
                gbtimeoutreached = true; //Setting to true stops new events starting once this ones times out

            }
            //Show Masked Version
            cv::circle(im_with_keypoints,cv::Point(gframeMask.cols/2,gframeMask.rows/2),gframeMask.cols/2-50,CV_RGB(0,205,15),1,cv::LINE_8);

            cv::imshow(Reader_input->windisplay, im_with_keypoints );
            if (!image_from_bufferB.empty())
            {
                cv::resize(image_from_bufferB, imS, cv::Size((1280-130)/2, (1024)/2));
                cv::imshow("Top camera View (1/2 scale )", imS );
            }


        }
		else
        { // No Frame Retrieved
          cout << "f:" << camA_frame_counter << endl;
          continue; //Wait Until Q Is pressed
        }

        ///Process Image - Check If Fish Is in there

        ///Tell Recorder that Fish Is Here - if Blob has been Detected
        /// Press r or s  To Force Recording
        if (c=='r' && !gbEventRecording) //User Initiated Event Trigger Rec
        {
            sem_post(&semImgFishDetected); //Trigger an Initial Event By Default
            cout << "Event Triggered by user." << endl;
        }else{
            if (c=='s' && !gbRecStarted) //User Initiated Recording
            {
                mtx.lock();
                gbRecStarted = true;
                mtx.unlock();
                tstart   = (double)cv::getTickCount();
                //gbEventRecording = true;
                sem_post(&semImgFishDetected); //Trigger an Initial Event By Default
                cout << "REC started by user." << endl;
            }else{
                sem_getvalue(&semImgFishDetected, &fishFlag); //Read Current Frame
                //Post That Fish Have been Found if we obtain Filtered Masks or if recorder is in Continuous Recording mode
                if ((fishFlag == 0 && keypoints_in_mask.size() > 0) && gbRecStarted )
                {
                    //Signal that to Initiate Recording on rec thread
                    sem_post(&semImgFishDetected);
                    //Moved to rec_cam thread Reader_input->pcircbufferA->writeNewFramesToImageSequence(); //Save recent Frames preceding Trigger
                }else{
                  //The semaphore will be decremented if its value is greater than zero. If the value of the semaphore is zero, then sem_trywait() will return -1 and set errno to EAGAIN
                  if (keypoints_in_mask.size() == 0  && gbRecStarted) //There are no fish / Decremend Semaphore - But dont Lock
                          sem_trywait(&semImgFishDetected);
                    }
            }
       }


    } //Main Loop Wait for control input

    timespec tout;
    tout.tv_sec = 1;
    tout.tv_nsec = 0;
    sem_timedwait(&semImgFishDetected,&tout); //Try Decrement Value Of Fish Exists - And So Stop Recording
    mtx.lock();
        gbrun = false; //Flag that stops the inf loop of recorder.
    mtx.unlock();
    fishdetector->clear();
    Reader_input->pcircbufferA->set_recorder_state(false);
    if (Reader_input->pcircbufferB)
        Reader_input->pcircbufferB->set_recorder_state(false);

    if (t >= Reader_input->timeout)
    {
        std::cout << "Maximum Recording Duration reached :"<< Reader_input->timeout << " sec" << std::endl;
        cv::imshow(Reader_input->windisplay, im_with_keypoints );
        //std::flush;
  //      gbrun = false;
        cv::waitKeyEx(200);
        usleep(5000);
   }



    pthread_exit(EXIT_SUCCESS);
//
    //std::exit(EXIT_SUCCESS);
    return 0;

}


