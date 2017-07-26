#pragma once

#ifndef _UTIL_H
#define _UTIL_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <string>
#include <pthread.h>
#include <semaphore.h>


#define ZR_OUTPICFORMAT  ".pgm"
#define ZR_WINDOWNAME     "display"

using namespace std;
using namespace FlyCapture2;


extern sem_t   semImgCap;////Semaphore for image Captured Signal
extern pthread_cond_t cond;
extern pthread_mutex_t lock;
extern bool bImgCaptured;/// Global Flag indicating new Image Has been captured by camera

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
    string proc_folder;
    string display;
	size_t seq_size;
	bool crop;
};

struct thread_data2{
	Camera *cam;
    string proc_folder;
    string display;
    size_t seq_size;
    uint MaxFrameDuration;
    bool crop;
};


void my_handler(int);

void SetCam(Camera *cam, F7 &f7, const Mode k_fmt7Mode, const PixelFormat k_fmt7PixFmt, float& pfFrameRate,float& pfShutter);

inline void PrintError(Error error) { error.PrintErrorTrace(); }
void PrintBuildInfo();
void PrintFormat7Capabilities(Format7Info fmt7Info);
void PrintCameraInfo(CameraInfo *pCamInfo);
int Rec_SingleCamera(void*);
void *Rec_onDisk_SingleCamera2(void *tdata);
void ReadImageSeq(string prefix,string display,int mode=0,char* format=ZR_OUTPICFORMAT,char* prefix0="");
int Run_SingleCamera(PGRGuid);

#endif
