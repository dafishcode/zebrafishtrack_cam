#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <FlyCapture2.h>
#include <CameraBase.h>
#include <iostream>
#include <sstream>
#include "../include/functions.h"

using namespace FlyCapture2;
using namespace std;

int main(int argc, char** argv)
{
    PrintBuildInfo();

    BusManager busMgr;
    
    PGRGuid guid;
    busMgr.GetCameraFromIndex(0, &guid);
    Run_SingleCamera(guid);

    return 0;
}
