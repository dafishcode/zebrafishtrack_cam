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
//To prevent copying a class, you can very easily declare a private copy constructor / assignment operators. But you can also inherit boost::noncopyable.
//: private boost::noncopyable
class circular_buffer_ts: private boost::noncopyable
{
public:

    typedef boost::mutex::scoped_lock slock;

    circular_buffer_ts() {}

    circular_buffer_ts(int n, string pf,ofstream* lf) {
        cb.set_capacity(n);
        frame_index.set_capacity(n);
        time_index.set_capacity(n);
        last_recorded_index=0;
        proc_folder=pf;
        logfile=lf;
        recording_state=false;
        writing_buffer=false;
    }


    void update_buffer(const cv::Mat &imdata, int f, int64 t) {
        slock lk(monitor);
        if(!writing_buffer){
            //cv::Mat im;
            //imdata.copyTo(im);
            if(verbose) std::cout<<"update buffer: "<<f<<' '<<t<<endl;
            cb.push_back(imdata.clone());
            frame_index.push_back(f);
            time_index.push_back(t);
            buffer_not_empty.notify_one();
        }
    }

    void retrieve_last(cv::Mat &image, long int &cf) {
        slock lk(monitor);

        if(verbose)
            cout<<"retrieve frame: "<<frame_index.back()<<' '<<time_index.back()<<endl;
        
        cb.back().copyTo(image);
        cf=frame_index.back();
    }

    void clear() {
        slock lk(monitor);
        cb.clear();
    }

    int size() {
        slock lk(monitor);
        return cb.size();
    }

    void set_capacity(int capacity) {
        slock lk(monitor);
        cb.set_capacity(capacity);
    }

    void set_recorder_state(bool rs){
        slock lk(monitor);
        recording_state=rs;
    }

    void set_last_recorded_index(long int f){
        slock lk(monitor);
        last_recorded_index=f;
    }

    void set_writing_buffer(bool br){
        slock lk(monitor);
        writing_buffer = br;
    }

    bool get_recorder_state(){
        slock lk(monitor);
        return recording_state;
    }

    bool get_writing_buffer(){
        slock lk(monitor);
        return writing_buffer;
    }

    // This action can only be done by the processor thread so it does not need to be thread safe.
    void write_buffer(){


        unsigned int i=0;
        unsigned int lri;

        // set writing_buffer, get last recorded index
        {
            slock lk(monitor);
            writing_buffer=true;
            lri=last_recorded_index;
            cout<<"writing whole buffer"<<endl;
        }

        for(i=0;i<cb.size();i++){
            if(frame_index[i]>lri){
                stringstream filename;
                filename << proc_folder<<"/" << fixedLengthString((int)frame_index[i],10) << ".pgm";
                cv::imwrite(filename.str().c_str(),cb[i]);
                *logfile<<time_index[i]<<' '<<cv::getTickFrequency()<<' '<<1<<endl;
                if(verbose) cout<<"write buffer: "<<' '<<frame_index[i]<<' '<<time_index[i]<<endl;
            }
        }

        // update last recorded index and turn off writing_buffer
        {
            slock lk(monitor);
            last_recorded_index=frame_index[i-1];
            writing_buffer=false;
        }
    }


private:

    boost::condition buffer_not_empty;
    boost::condition buffer_ready;
    boost::mutex monitor;
    boost::circular_buffer<cv::Mat> cb;
    boost::circular_buffer<int> frame_index;
    boost::circular_buffer<int64> time_index;
    bool recording_state;
    bool writing_buffer;
    long int last_recorded_index;

    string proc_folder;
    ofstream* logfile;
    bool verbose=false;
};

#endif // CIRCULAR_BUFFER_TS_H
