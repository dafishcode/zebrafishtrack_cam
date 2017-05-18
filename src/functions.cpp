#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <sstream>
#include<fstream>
#include<signal.h>
#include<sys/stat.h>
#include"../include/functions.h"

using namespace std;
using namespace FlyCapture2;

bool run=true;

void my_handler(int sig){
           cout<<endl<<"Recording stopped."<<endl;
	   run=false;
}

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
    cout << "Max image pixels: (" << fmt7Info.maxWidth << ", "
         << fmt7Info.maxHeight << ")" << endl;
    cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", "
         << fmt7Info.imageVStepSize << ")" << endl;
    cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", "
         << fmt7Info.offsetVStepSize << ")" << endl;
    cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
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
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl
         << endl;
}

int Rec_SingleCamera(void* tdata)
{
    Error error;  
 
    struct thread_data * RSC_input;
    RSC_input = (struct thread_data*) tdata;
    size_t seq_size = RSC_input->seq_size;

    struct stat sb;
    if (stat(RSC_input->proc_folder, &sb) != 0){
	    const int dir_err = mkdir(RSC_input->proc_folder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	    if (-1 == dir_err){
		    printf("Error creating directory!");
		    exit(1);
	    }
    }

    // Connect to a camera
    Camera cam;
    cam.Connect(RSC_input->guid);

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration(&config);

    // Set the number of driver buffers used to 10.
    config.numBuffers = 10;

    // Set the camera configuration
    cam.SetConfiguration(&config);

    // Set the custom image format7 
    const Mode k_fmt7Mode = MODE_1;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;

    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode=k_fmt7Mode;
    cam.GetFormat7Info(&fmt7Info, &supported);
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;
    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);   

    CameraInfo cInfo;
    cam.GetCameraInfo(&cInfo);
    PrintCameraInfo(&cInfo);

    // Start capturing images
    cam.StartCapture();

    Image rawImage;
    cv::Mat tmp_image;
    vector<cv::Mat> imvec(seq_size);
    cv::namedWindow("display",cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("display", 800,800);
    

    // Retrieve a single image
    cam.RetrieveBuffer(&rawImage);

    unsigned char* data = rawImage.GetData();
    cv::Mat cvm(rawImage.GetRows(),rawImage.GetCols(),CV_8U,(void*)data);
    
    char key='a';
    bool recording=false;
    ioparam center;
    ioparam tmp_center;
    tmp_center.status=0;
    cv::setMouseCallback("display",on_mouse,&tmp_center);
    string yn;
    cv::Mat drawing;
    cvm.copyTo(drawing);
    ofstream logfile("log.txt");
    
    while(key!='q'){
            if(tmp_center.status){
                    cvm.copyTo(drawing);
		    cv::rectangle(drawing,tmp_center.pt1,tmp_center.pt2,0,4,8,0);
                    ostringstream info; 
		    
                    info<<"center = ("<<tmp_center.center.x<<','<<tmp_center.center.y<<")";
	            cv::imshow("display",drawing);
	            cv::displayStatusBar("display",info.str(),0);
				center=tmp_center;
            } else cv::imshow("display",drawing);

	    key=cv::waitKey(10); 
                    
            if(key=='r'){
		    cout<<"RECORDING..."<<endl;
		    recording=true;
		    break;
	    }
    }

    cv::destroyWindow("display");

    if(recording){
	    long int ms0 = cv::getTickCount();
	    for(unsigned int i=0;i<imvec.size();i++){
		    cam.RetrieveBuffer(&rawImage);
		    long int ms1 = cv::getTickCount(); 
                    double delta = (ms1-ms0)/cv::getTickFrequency();
                    logfile<<i<<' '<<delta<<' '<<endl;
		    data = rawImage.GetData();
		    cv::Mat cvm(rawImage.GetRows(),rawImage.GetCols(),CV_8U,(void*)data);
		    tmp_image=cvm(cv::Range(center.pt1.y,center.pt2.y),cv::Range(center.pt1.x,center.pt2.x));
		    tmp_image.copyTo(imvec[i]);
	    }                    
    }

    // Stop capturing images
    error = cam.StopCapture();

    // Disconnect the camera
    error = cam.Disconnect();
    
    if(recording){
	    for(unsigned int i=0;i<imvec.size();i++){
		    stringstream filename;
		    filename<<"test/test-"<<i<<".tiff";
		    
		    imwrite(filename.str().c_str(),imvec[i]);
	    }
    }

    return 0;
}

void SetCam(PGRGuid *guid, Camera *cam){

    Error error;  

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration(&config);

    // Set the number of driver buffers used to 10.
    config.numBuffers = 10;

    // Set the camera configuration
    cam.SetConfiguration(&config);

    // Set the custom image format7 
    const Mode k_fmt7Mode = MODE_1;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;

    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode=k_fmt7Mode;
    cam.GetFormat7Info(&fmt7Info, &supported);
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;
    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);   

    CameraInfo cInfo;
    cam.GetCameraInfo(&cInfo);
    PrintCameraInfo(&cInfo);


void Select_ROI(void *tdata, ioparam center){


void *Rec_onDisk_SingleCamera(void *tdata)
{
    signal(SIGINT,my_handler);
    struct thread_data * RSC_input;
    RSC_input = (struct thread_data*) tdata;

    struct stat sb;
    if (stat(RSC_input->proc_folder, &sb) != 0){
	    const int dir_err = mkdir(RSC_input->proc_folder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	    if (-1 == dir_err){
		    printf("Error creating directory!");
		    exit(1);
	    }
    }

    Error error;   

    // Connect to a camera
    Camera cam;
    cam.Connect(RSC_input->guid);

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration(&config);

    // Set the number of driver buffers used to 10.
    config.numBuffers = 10;

    // Set the camera configuration
    cam.SetConfiguration(&config);

    // Set the custom image format7 
    const Mode k_fmt7Mode = MODE_1;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;

    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode=k_fmt7Mode;
    cam.GetFormat7Info(&fmt7Info, &supported);
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;
    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);   

    CameraInfo cInfo;
    cam.GetCameraInfo(&cInfo);
    PrintCameraInfo(&cInfo);

    // Start capturing images
    cam.StartCapture();

    Image rawImage;
    cv::Mat tmp_image;
    

    // Retrieve a single image
    cam.RetrieveBuffer(&rawImage);

    unsigned char* data = rawImage.GetData();
    cv::Mat cvm(rawImage.GetRows(),rawImage.GetCols(),CV_8U,(void*)data);
    
    char key='a';
    bool recording=false;
    ioparam center;
    ioparam tmp_center;
    tmp_center.status=0;
    cv::setMouseCallback(RSC_input->display,on_mouse,&tmp_center);
    string yn;
    cv::Mat drawing;
    cvm.copyTo(drawing);
    ofstream logfile("log.txt");
    char input;
    
    while(key!='q'){
		if(tmp_center.status && RSC_input->crop){
			cvm.copyTo(drawing);
			cv::rectangle(drawing,tmp_center.pt1,tmp_center.pt2,0,4,8,0);
			ostringstream info; 
			
			info<<"center = ("<<tmp_center.center.x<<','<<tmp_center.center.y<<")";
			cv::imshow(RSC_input->display,drawing);
			
			//center.status=false;
			cv::displayStatusBar(RSC_input->display,info.str(),0);
			center=tmp_center;
		} else cv::imshow(RSC_input->display,drawing);
		
		key=cv::waitKey(10); 
		
		if(key=='r'){
			cout<<"RECORDING..."<<endl;
			recording=true;
			break;
		}
	}
	
	cv::destroyWindow(RSC_input->display);
    
    unsigned int i=0;
    long int ms0 = cv::getTickCount();

    while(recording && run){
		cam.RetrieveBuffer(&rawImage);
	    long int ms1 = cv::getTickCount(); 
	    double delta = (ms1-ms0)/cv::getTickFrequency();
	    logfile<<i<<' '<<delta<<' '<<endl;
	    data = rawImage.GetData();
	    cv::Mat cvm(rawImage.GetRows(),rawImage.GetCols(),CV_8U,(void*)data);
	    if(RSC_input->crop){
		    tmp_image=cvm(cv::Range(center.pt1.y,center.pt2.y),cv::Range(center.pt1.x,center.pt2.x));
	    } else {
		    tmp_image=cvm;
	    }
	    stringstream filename;
	    filename<<RSC_input->proc_folder<<"/"<<i<<".tiff";
	    imwrite(filename.str().c_str(),tmp_image);
	    i++; 
    }                    


    // Disconnect the camera
    error = cam.Disconnect();

    return 0;
}

void ReadImageSeq(string prefix,char* display){
	int ind=0;
	cv::Mat image;
	cv::namedWindow(display,cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO );
        cv::resizeWindow(display, 800,800);

        char c='1';
	while(c!='q'){
		if(c=='f') ind++;
		if(c=='b') ind=max(0,ind-1);
		stringstream filename;
		filename<<prefix<<'/'<<ind<<".tiff";
		image=imread(filename.str().c_str(),cv::IMREAD_UNCHANGED);
		if(!image.empty())
			imshow(display,image);
                c=cv::waitKey(10);
	}
}

int Run_SingleCamera(PGRGuid guid)
{

    Error error;    

    // Connect to a camera
    Camera cam;
    cam.Connect(&guid);

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration(&config);

    // Set the number of driver buffers used to 20.
    config.numBuffers = 10;

    // Set the camera configuration
    cam.SetConfiguration(&config);

    CameraInfo cInfo;
    cam.GetCameraInfo(&cInfo);
    PrintCameraInfo(&cInfo);

    // Set format7 custom mode
    const Mode k_fmt7Mode = MODE_0;
    const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_RAW8;

    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode=k_fmt7Mode;
    cam.GetFormat7Info(&fmt7Info, &supported);
    PrintFormat7Capabilities(fmt7Info);

    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;
    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    cam.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
    cam.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);   

    // Start capturing images
    cam.StartCapture();

    Image rawImage;
    namedWindow("display",cv::WINDOW_NORMAL);
    while (cv::waitKey(30)!='q')
    {
        // Retrieve an image
        cam.RetrieveBuffer(&rawImage);

        // Create a converted image
        Image convertedImage=rawImage;

        // Convert the raw image
        //rawImage.Convert(PIXEL_FORMAT_MONO8, &convertedImage);
        
        unsigned char* data = convertedImage.GetData();
        cv::Mat cvm(convertedImage.GetRows(),convertedImage.GetCols(),CV_8U,(void*)data);
        cv::transpose(cvm,cvm);
        cv::imshow("display",cvm);

    }

    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        cout<<"ERROR!";
        return -1;
    }

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        cout<<"ERROR!";
        return -1;
    }

    return 0;
}

