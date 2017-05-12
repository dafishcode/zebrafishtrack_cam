
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <sstream>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
	Mat frame;
	namedWindow("frame");
	//double rate = capture.get(CV_CAP_PROP_FPS);
	//int delay = 1000/rate;
        
	while(true){
		if(!frame=imread(argv[1])){
			break;
		}
		imshow("frame",frame);
		
		if(waitKey(30)>=0)
			break;
	}

	capture.release();
	return 0;
}
