#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <sstream>
#include<fstream>
#include<sys/stat.h>
#include<signal.h>
#include<cstdio>
#include"../include/functions2.h"

using namespace std;
using namespace FlyCapture2;

int main(int argc, char** argv)
{
    if(argc<2){
	    cout<<"Recording Reader."<<endl;
	    cout<<"-------------------"<<endl;
	    cout<<"Usage:"<<endl;
	    cout<<"./ShowRec <folder>"<<endl;
            exit(0);
    }

    int i;
   
    cv::namedWindow("display",cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow("display", 800,800);

    ReadImageSeq(argv[1],"display",1,".tiff");

    return 0;
}
