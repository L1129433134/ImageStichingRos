#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <boost/asio.hpp>
#include "gtest/gtest_prod.h"

/**
The class ImageInvert is used for accepting 2 video streams and stiching together there inverted frames.
The constructor accepts the number of strings used, an array of strings having the location of the video files and
the number of threads needed to be used for performing frame invertion using multi threads.
**/

class ImageInvert{
    public:
        /** Smart pointer holding the Videolocation **/
        boost::scoped_array<std::string> m_pVideoLocation;
        /** Variable for holding the number of strings **/
        int m_iNumberStrings;
        /** Variable for holding the number of threads **/
        int num_threads_invert;
        /** Smart pointer holding the Video writer object **/
        boost::scoped_ptr<cv::VideoWriter>m_pVideoOutputptr;

        ImageInvert(int num_video_streams, std::string input_string[], int threads_num = 100)
        {
            m_iNumberStrings = num_video_streams;
            /** Assigning value to the pointer  **/
            m_pVideoLocation.reset(new std::string[num_video_streams]);
            for(int i = 0; i< num_video_streams;i++)
                m_pVideoLocation[i] = input_string[i];
            /** Checking if the number of threads to be used is within a reasonable limit**/
            if(threads_num<100)
                num_threads_invert = 100;
            else if (threads_num > 900)
                num_threads_invert= 900;
            else
                num_threads_invert=threads_num ;
        }
        ~ImageInvert()
        {}
        /** Function for performing the stitching **/
        void StartStitchVideos();
        /** Function for Resizing the Images into the size of the smallest one **/
        void ResizeImages(cv::Mat &Image,bool status, int FrameSizes[4] ,int& image_number) ;
        /** Function to identify the smallest Frame **/
        bool SmallerImage(int FrameSizes[4]);
        /** Function to Invert the Frames using Parallelization**/
        void InvertImages(cv::Mat &Image);
        /** Function to Invert the Frames  **/
        void DemoFunc(int i,int j,cv::Mat &Image);
};
