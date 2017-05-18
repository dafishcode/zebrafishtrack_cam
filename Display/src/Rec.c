#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <sstream>
#include<fstream>
#include"../include/functions.h"

using namespace std;
using namespace FlyCapture2;

int main(int argc, char** argv)
{
    //PrintBuildInfo();
    int seq_size=5000;
    BusManager busMgr;
    PGRGuid guid;
    busMgr.GetCameraFromIndex(0, &guid);
    Rec_SingleCamera(guid,seq_size);
    ReadImageSeq("test","display");

    return 0;
}
