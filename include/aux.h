#ifndef AUX_H
#define AUX_H

#include <iostream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <string>


#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

std::string fixedLengthString(int value, int digits = 10);

cv::Mat gLastfishimg_template;
cv::Mat gFishTemplateCache; //A mosaic image contaning copies of template across different angles

int idxLastTemplateRow = 1;
int idxLastTemplateCol = 1;
int gFishBoundBoxSize               = 24; /// pixel width/radius of bounding Box When Isolating the fish's head From the image
int gFishTemplateAngleSteps     = 1;

const int giHeadIsolationMaskVOffset      = 24; //Vertical Distance to draw  Mask and Threshold Sampling Arc in Fish Head Mask
bool bTemplateSearchThroughRows           = false; /// Stops TemplateFind to Scan Through All Rows (diff temaplte images)- speeding up search + fail - Rows still Randomly Switch between attempts
int gnumberOfTemplatesInCache       = 0; //INcreases As new Are Added

//list of template images
std::vector<cv::Mat> vTemplImg;

/// Eye Tracking Params
int gi_maxEllipseMajor      = 21; /// thres  for Eye Ellipse Detection methods
int gi_minEllipseMajor      = 10; ///thres for Eye Ellipse Detection methods (These Values Tested Worked Best)
int gi_minEllipseMinor      = 0; /// ellipse detection width - When 0 it allows for detecting straight line
int gi_MaxEllipseSamples    = 10; //The number of fitted ellipsoids draw from the ranked queue to calculate mean fitted eye Ellipse
int gthresEyeSeg                    = -10; //Additional Adjustment for Adaptive Threshold  For Eye Segmentation In Isolated Head IMage
int gthresEyeSegL                   = 2;


///Fish Features Detection Params
 double gTemplateMatchThreshold  = 0.80; //If not higher than 0.9 The fish body can be matched at extremeties
 int iTemplateMatchFailCounter   = 0; //Counts the number of consecutive times template failed to match
#endif // AUX_H
