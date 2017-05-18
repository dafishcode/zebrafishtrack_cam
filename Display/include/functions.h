#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>

using namespace std;
using namespace FlyCapture2;

class ioparam {
	public:
	cv::Point2i center;
	cv::Point2i pt1, pt2;
	bool status;
};

struct thread_data{
	PGRGuid *guid;
	char* proc_folder;
    char* display;
};

void my_handler(int);

void PrintBuildInfo();
void PrintFormat7Capabilities(Format7Info fmt7Info);
void PrintCameraInfo(CameraInfo *pCamInfo);
int Rec_SingleCamera(PGRGuid guid,int seq_size);
void *Rec_onDisk_SingleCamera(void *tdata);
void ReadImageSeq(string prefix,char* display);
int Run_SingleCamera(PGRGuid guid);
