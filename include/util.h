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

#include "../include/aux.h"
#include "../include/circular_buffer_ts.h"


#define ZR_OUTPICFORMAT  ".pgm"
#define ZR_WINDOWNAME     "display"
#define ZR_FISHTIMEOUT    60 //sec of min time recording
using namespace std;
using namespace FlyCapture2;

extern cv::Mat gframeBuffer; //Global Image Buffer holding Next Image for Display
extern cv::Mat gframeMask;
extern sem_t   semImgCapCount;////Semaphore for image Captured Signal
extern sem_t semImgFishDetected; //There is a fish In the scene lock
extern pthread_cond_t cond;
extern pthread_mutex_t lock;
extern bool bImgCaptured;/// Global Flag indicating new Image Has been captured by camera
extern bool gbEventRecording; //Glag That Images Are being saved
extern bool gbtimeoutreached; //Glag That Images Are being saved

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
struct observer_thread_data{
    string prefix;
    string proc_folder;
    string windisplay;
    circular_buffer_ts* pcircbufferA; // pointer to Circular Buffer Used in Recording Thread camA
    circular_buffer_ts* pcircbufferB; // pointer to Circular Buffer Used in Recording Thread camB
    int mode;
    char* format;
    string prefix0;
    uint timeout;

};

///Used for Image Capture
struct camera_thread_data{
	Camera *cam;
    string proc_folder;
    circular_buffer_ts* pcircbuffer;  // pointer to Circular Buffer where to store all camera frames retrieved
    int eventCount; ///Number of Times Fish Has been cited/ Prey cApture events
    uint eventtimeout; //Min n of frames an event recording should last
    string display;
    size_t seq_size;
    uint MaxFrameDuration;
    bool crop;
};


void my_handler(int);

void CreateOutputFolder(string folder);
void SetCam(Camera *cam, F7 &f7, const Mode k_fmt7Mode, const PixelFormat k_fmt7PixFmt, float& pfFrameRate,float& pfShutter);

inline void PrintError(Error error) { error.PrintErrorTrace(); }
void PrintBuildInfo();
void PrintFormat7Capabilities(Format7Info fmt7Info);
void PrintCameraInfo(CameraInfo *pCamInfo);

void *rec_onDisk_BottomCamera(void *tdata); //High FPS Event |Triggered
void* rec_onDisk_TopCamera(camera_thread_data &RSC_input); //Low FPS Top Camera Thread

void *camViewEventTrigger(void *tdata);
//int Run_SingleCamera(PGRGuid);
//int Rec_SingleCamera(void*);

#endif
