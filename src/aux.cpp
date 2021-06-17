
#include "../include/aux.h"

std::string fixedLengthString(int value, int digits) {
    unsigned int uvalue = value;
    if (value < 0) {
        uvalue = -uvalue;
    }
    std::string result;
    while (digits-- > 0) {
        result += ('0' + uvalue % 10);
        uvalue /= 10;
    }
    if (value < 0) {
        result += '-';
    }

    std::reverse(result.begin(), result.end());
    return result;
}



///// \brief Load internal and external template images memory cache //
//int initDetectionTemplates()
//{
//    cv::Mat lastfish_template_img;// OUr Fish Image Template
//    ///////////////////////////////////////
//    /// Setup Fish Body Template Cache //
//    int idxTempl;

//    for (idxTempl=0; idxTempl<gTrackerState.nTemplatesToLoad;idxTempl++)
//    {
//        trackerState::loadFromQrc(QString::fromStdString(gTrackerState.strTemplateImg + to_string(idxTempl+1) + std::string(".pgm")),lastfish_template_img,IMREAD_GRAYSCALE); //  loadImage(strTemplateImg);
//        if (lastfish_template_img.empty())
//        {
//            std::cerr << "Could not load template" << std::endl;
//            exit(-1);
//        }
//        //Add to Global List Of Template Images
//        vTemplImg.push_back(lastfish_template_img);
//        //Add to Cache and generate all All Angle Varations
//        addTemplateToCache(lastfish_template_img,gFishTemplateCache,idxTempl); //Increments Index
//    }

//    // Set Template Size
//    gszTemplateImg.width = lastfish_template_img.size().width; //Save TO Global Size Variable
//    gszTemplateImg.height = lastfish_template_img.size().height; //Save TO Global Size Variable

//    // Set Paster Region for Inset Image
//    gTrackerState.rect_pasteregion.x = (640-gszTemplateImg.width*2);
//    gTrackerState.rect_pasteregion.y = 0;
//    gTrackerState.rect_pasteregion.width = gszTemplateImg.width*2; //For the upsampled image
//    gTrackerState.rect_pasteregion.height = gszTemplateImg.height*2;

//    gTrackerState.gstroutDirTemplates = gTrackerState.gstroutDirCSV + ("/templates/");
//    int ifileCount = loadTemplatesFromDirectory(QString::fromStdString(  gTrackerState.gstroutDirTemplates) );

//    //Make Mean Fish And Add to Cache
//     cv::Mat templFrame = makeMeanTemplateImage(gTrackerState.vTemplImg);
//     addTemplateToCache(templFrame,gFishTemplateCache,gTrackerState.gnumberOfTemplatesInCache);
//     gTrackerState.gLastfishimg_template = templFrame; //Set To Global
// #if defined(_DEBUG)
//     cv::imshow("Template Cache",gFishTemplateCache);
//#endif

//    return (gTrackerState.gnumberOfTemplatesInCache);
//    /// END OF FISH TEMPLATES ///
//}
