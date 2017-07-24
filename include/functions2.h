#pragma once

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

class F7 {
	public:
	FC2Config config;
	Format7Info fmt7Info;
	Format7ImageSettings fmt7ImageSettings;
	Format7PacketInfo fmt7PacketInfo;
	bool valid;
	bool supported;
};

struct thread_data{
	PGRGuid *guid;
	char* proc_folder;
	char* display;
	size_t seq_size;
	bool crop;
};

struct thread_data2{
	Camera *cam;
	char* proc_folder;
    char* display;
    size_t seq_size;
    bool crop;
};


void my_handler(int);

void SetCam(Camera *cam, F7 &f7, const Mode k_fmt7Mode, const PixelFormat k_fmt7PixFmt, unsigned int& FrameRate);

inline void PrintError(Error error) { error.PrintErrorTrace(); }
void PrintBuildInfo();
void PrintFormat7Capabilities(Format7Info fmt7Info);
void PrintCameraInfo(CameraInfo *pCamInfo);
int Rec_SingleCamera(void*);
void *Rec_onDisk_SingleCamera2(void *tdata,unsigned int cMaxFrames);
void ReadImageSeq(string prefix,char* display,int mode=0,char* format=".pgm",char* prefix0="");
int Run_SingleCamera(PGRGuid);
