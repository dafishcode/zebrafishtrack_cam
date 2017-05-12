#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <sstream>
#include<fstream>

using namespace std;
using namespace FlyCapture2;

class ioparam {
	public:
	cv::Point2i center;
	cv::Point2i pt1, pt2;
	bool status;
};

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

int RunSingleCamera(PGRGuid guid)
{

    Error error;   
    int seq_size=5000; 

    // Connect to a camera
    Camera cam;
    cam.Connect(&guid);

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
    center.status=0;
    cv::setMouseCallback("display",on_mouse,&center);
    string yn;
    cv::Mat drawing;
    cvm.copyTo(drawing);
    ofstream logfile("log.txt");
    
    while(key!='q'){
            if(center.status){
                    cvm.copyTo(drawing);
		    cv::rectangle(drawing,center.pt1,center.pt2,0,4,8,0);
                    ostringstream info; 
		    
                    info<<"center = ("<<center.center.x<<','<<center.center.y<<")";
	            cv::imshow("display",drawing);
    
                    //center.status=false;
	            cv::displayStatusBar("display",info.str(),0);
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

void ReadImageSeq(string prefix,int n_images){
	int ind=0;
	vector<cv::Mat> image_seq(n_images);
	for(unsigned int i=0;i<n_images;++i){
		stringstream filename;
		filename<<prefix<<'/'<<prefix<<"-"<<i<<".tiff";
		image_seq[i]=imread(filename.str().c_str(),cv::IMREAD_UNCHANGED);
	}
	cv::namedWindow("display",cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO );
        cv::resizeWindow("display", 800,800);
	cv::createTrackbar( "index", "display", &ind, n_images-1,  NULL);

        char c='1';
	while(c!='q'){
		imshow("display",image_seq[ind]);
                c=cv::waitKey(10);
	}
}

int main(int argc, char** argv)
{
    //PrintBuildInfo();
    int seq_size=5000;
    BusManager busMgr;
    PGRGuid guid;
    busMgr.GetCameraFromIndex(0, &guid);
    RunSingleCamera(guid);
    ReadImageSeq("test",seq_size);

    return 0;
}
