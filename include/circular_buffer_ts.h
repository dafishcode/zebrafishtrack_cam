#ifndef CIRCULAR_BUFFER_TS_H
#define CIRCULAR_BUFFER_TS_H

#include <iostream>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/noncopyable.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <signal.h>

#include "aux.h"
#include "util.h"
// Thread safe circular buffer

using namespace std;
/// \brief Handles writing of video frames to disk along with logging time of each frame. Uses a rolling Buffer of images so as to allow to antecedently save images following an event trigger.
///
//To prevent copying a class, you can very easily declare a private copy constructor / assignment operators. But you can also inherit boost::noncopyable.
//: private boost::noncopyable
class circular_buffer_ts: private boost::noncopyable
{
public:

    typedef boost::mutex::scoped_lock slock;

    circular_buffer_ts() {}

    circular_buffer_ts(int n, string pf,ofstream* lf, cv::VideoWriter* pVideoStream) {
        circ_buff_img.set_capacity(n);
        frame_index.set_capacity(n);
        time_index.set_capacity(n);
        logstring.set_capacity(n);

        idx_last_recorded=0;
        proc_folder=pf;
        logfile=lf;
        recording_state=false;
        writing_buffer=false;
    }

    // Add new Image to Buffer - Cannot not be done while Buffer is being dumped to disk
    void update_buffer(const cv::Mat &imdata, uint f,uint t, string logentrystring) {
        slock lk(monitor);
        if(!writing_buffer){
            //cv::Mat im;
            //imdata.copyTo(im);
            if(verbose) std::cout<<"update buffer: "<< f <<' '<< t <<endl;
            circ_buff_img.push_back(imdata.clone());
            frame_index.push_back(f);
            time_index.push_back(t);
            logstring.push_back(logentrystring);

            buffer_not_empty.notify_one();
        }
    }

    void retrieve_last(cv::Mat &image, long int &lastframeIdx) {
        slock lk(monitor);

        if (frame_index.size() == 0)
            return; //No Images Exist

        if(verbose)
            cout<<"retrieve frame: "<<frame_index.back()<<' '<<time_index.back()<<endl;
        
        circ_buff_img.back().copyTo(image);
        lastframeIdx=frame_index.back();
    }

    void clear() {
        slock lk(monitor);
        circ_buff_img.clear();
    }

    int size() {
        slock lk(monitor);
        return circ_buff_img.size();
    }

    void set_capacity(int capacity) {
        slock lk(monitor);
        circ_buff_img.set_capacity(capacity);
    }

    void set_recorder_state(bool rs){
        slock lk(monitor);
        recording_state=rs;
    }

    void set_last_recorded_index(long int f){
        slock lk(monitor);
        idx_last_recorded=f;
    }

    void set_writing_buffer(bool br){
        slock lk(monitor);
        writing_buffer = br;
    }

    void set_outputfolder(string sdir)
    {   slock lk(monitor);
        proc_folder = sdir;
    }

    bool get_recorder_state(){
        slock lk(monitor);
        return recording_state;
    }

    bool get_writing_buffer(){
        slock lk(monitor);
        return writing_buffer;
    }

    // Dumps unexported part of Buffer To Disk - while it lock adding any new contents to it
    // This action can only be done by the processor thread so it does not need to be thread safe.
    void writeNewFramesToImageSequence(){


        unsigned int i=0;
        unsigned int lri;

        // set writing_buffer, get last recorded index
        {
            slock lk(monitor);
            writing_buffer=true;
            lri=idx_last_recorded;
            //cout<<"writing whole buffer"<<endl;
        }

        for(i=0;i<circ_buff_img.size();i++){
            if(frame_index[i]>lri){
                // Writing to file
                stringstream filename;

                filename << proc_folder<<"/" << fixedLengthString((int)frame_index[i],10) << ".pgm";
                cv::imwrite(filename.str().c_str(),circ_buff_img[i]);
                *logfile <<  logstring[i];// frame_index[i] << '\t' <<time_index[i]<<'\t'<<cv::getTickFrequency() << endl;
                if(verbose) cout<<"write buffer: "<<'\t'<<frame_index[i]<<'\t'<<time_index[i]<<endl;
            }
        }

        // update last recorded index and turn off writing_buffer
        {
            slock lk(monitor);
            idx_last_recorded=frame_index[i-1];
            writing_buffer=false;
        }
    }

    // Dumps unexported part of Buffer Contents To Disk - while it lock adding any new contents to it
    // This action can only be done by the processor thread so it does not need to be thread safe.
    // Only exports frames that have not been saved yet
    void writeNewFramesToVideostream(){

        if ( !pVideowriter->isOpened() ) //if not initialize the VideoWriter successfully, exit
        {
              cerr << "ERROR: Failed to write circular buffer to video, videoWriter not opened" << endl;
              return;
        }

        unsigned int i=0;
        unsigned int lri;

        // set writing_buffer, get last recorded index
        {
            slock lk(monitor);
            writing_buffer=true;
            lri=idx_last_recorded; //save idx of most recently exported/saved img to file
            cout<<"writing buffer to vid"<<endl;
        }

        for(i=0;i < circ_buff_img.size();i++){
            if(frame_index[i] > lri){
                // Writing to file
                //cv::imwrite(filename.str().c_str(),);
                pVideowriter->write(circ_buff_img[i]);
                *logfile << logstring[i];//frame_index[i] << '\t' <<time_index[i]<<'\t'<<cv::getTickFrequency() << endl;

                if(verbose)
                    cout<<"write buffer: "<<'\t'<<frame_index[i]<<'\t'<<time_index[i]<<endl;
            }
        }

        // update last recorded index and turn off writing_buffer
        {
            slock lk(monitor);
            idx_last_recorded=frame_index[i-1];
            writing_buffer=false;
        }
    }


private:

    boost::condition buffer_not_empty;
    boost::condition buffer_ready;
    boost::mutex monitor;
    boost::circular_buffer<cv::Mat> circ_buff_img;
    boost::circular_buffer<uint> frame_index;
    boost::circular_buffer<int64> time_index; //Camera Microseconds
    boost::circular_buffer<string> logstring; //Camera Microseconds

    bool recording_state;
    bool writing_buffer;
    long int idx_last_recorded;

    string proc_folder;
    ofstream* logfile;
    cv::VideoWriter* pVideowriter = 0;
    bool verbose=false;
};

#endif // CIRCULAR_BUFFER_TS_H
