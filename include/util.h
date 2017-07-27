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

extern cv::Mat gframeBuffer; //Global Image Buffer holding Next Image for Display
extern sem_t   semImgCapCount;////Semaphore for image Captured Signal
extern sem_t semImgFishDetected; //There is a fish In the scene lock
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

///Used for Image Display And Processing function
struct thread_data{
    string prefix;
    string proc_folder;
    string windisplay;
    int mode;
    char* format;
    string prefix0;
};

///Used for Image Capture
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
void *ReadImageSeq(void *tdata);
int Run_SingleCamera(PGRGuid);

#endif
