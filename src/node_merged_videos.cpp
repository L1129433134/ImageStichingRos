#include "node_merged_videos.h"

/**
Function to check which frame is smaller. The input arguement is array having the width and height of the frames.
Width of each frames are compared first and then the height if both the widths are equal.
**/

bool ImageInvert::SmallerImage(int FrameSizes[4])
{
    if(FrameSizes[0] < FrameSizes[2])
        return true;
    else if(FrameSizes[0] > FrameSizes[2])
        return false;
    else if ( (FrameSizes[0] == FrameSizes[2]) && (FrameSizes[1] < FrameSizes[3]) )
        return true;
    else if   ( (FrameSizes[0] == FrameSizes[2]) && (FrameSizes[1] > FrameSizes[3]) )
        return false;
}

/*************************************/

/**
Helper function for finding the inverse of a pixel
**/
void ImageInvert::DemoFunc(int i,int j,cv::Mat &Image)
{   /** Type castng into an int **/
    int val = (int) Image.at<uchar>(i, j);
    if(val >= 0 && val<=255)
        Image.at<uchar>(i,j) = 255 - val;
    else
        ROS_ERROR("The Mat is not of GrayScale type");
}

/*************************************/

/** Function for inverting an image **/
void ImageInvert::InvertImages(cv::Mat &Image)
{   /** Calculates the size of the frame**/
    cv::Size Image_dim(Image.size());
    /** Creating an instance  of asynchronous input/output **/
    boost::asio::io_service ioService;

    for (int i =0;i<Image_dim.height;i++)
    {
        for(int j=0; j<Image_dim.width;j++)
        {   /** Queueing up task to be executed **/
            ioService.post(boost::bind(&ImageInvert::DemoFunc,this,i,j,boost::ref(Image)));
        }
    }
    boost::thread_group threadpool;
    for (unsigned t = 0; t <num_threads_invert ; t++)
    {   /** Creating threads upon user requirement and providing them with a task to complete**/
        threadpool.create_thread(boost::bind(&boost::asio::io_service::run, &ioService));
    }
    /** Waiting for all tasks to be completed**/
    threadpool.join_all();
}

/*************************************/

/**Function for Stitching 2 streams into a single source **/
void ImageInvert::StartStitchVideos()
{
    /**Create VideoCapture object to access the video streams**/
    cv::VideoCapture CapObjectHolder1(m_pVideoLocation[0]);
    cv::VideoCapture CapObjectHolder2(m_pVideoLocation[1]);

    if(CapObjectHolder1.get(CV_CAP_PROP_FRAME_COUNT) ==0 || CapObjectHolder2.get(CV_CAP_PROP_FRAME_COUNT) == 0)
        ROS_ERROR("An input file is empty.PleaseCheck");
    /** Create  Frames to hold the frame from video sources**/
    cv::Mat Frame1(CapObjectHolder1.get(CV_CAP_PROP_FRAME_HEIGHT),CapObjectHolder1.get(CV_CAP_PROP_FRAME_WIDTH),0);
    cv::Mat Frame2(CapObjectHolder2.get(CV_CAP_PROP_FRAME_HEIGHT),CapObjectHolder2.get(CV_CAP_PROP_FRAME_WIDTH),0);

    /** Vector for holding boost threads created using shared pointers **/
    std::vector<boost::shared_ptr<boost::thread> >threadImageHolder;
    /** Variable to limit the frames in the output video**/
    int SmallestSize;

    if(CapObjectHolder1.get(CV_CAP_PROP_FRAME_COUNT) < CapObjectHolder2.get(CV_CAP_PROP_FRAME_COUNT))
        SmallestSize= CapObjectHolder1.get(CV_CAP_PROP_FRAME_COUNT);
    else
        SmallestSize= CapObjectHolder2.get(CV_CAP_PROP_FRAME_COUNT);

    /**Array having the width and height of the 2 video streams **/
    int FrameSizes[4] = {CapObjectHolder1.get(CV_CAP_PROP_FRAME_WIDTH), CapObjectHolder1.get(CV_CAP_PROP_FRAME_HEIGHT), CapObjectHolder2.get(CV_CAP_PROP_FRAME_WIDTH) ,CapObjectHolder2.get(CV_CAP_PROP_FRAME_HEIGHT)};

    bool SmallImage = SmallerImage(FrameSizes);

    cv::Size * FrameSize;

    if(SmallImage)
         FrameSize = new cv::Size(CapObjectHolder1.get(CV_CAP_PROP_FRAME_WIDTH),CapObjectHolder1.get(CV_CAP_PROP_FRAME_HEIGHT));
    else
         FrameSize = new cv::Size(CapObjectHolder2.get(CV_CAP_PROP_FRAME_WIDTH),CapObjectHolder2.get(CV_CAP_PROP_FRAME_HEIGHT));
    /** Assigning the m_pVideoOutputptr to an instance of VideoWriter object**/
    m_pVideoOutputptr.reset( new cv::VideoWriter("/home/vishnu/Desktop/RosTutorials/catkin_ws/src/iris_automation_test/videos/output.avi",CV_FOURCC('X','V','I','D'), CapObjectHolder2.get(CV_CAP_PROP_FPS), *FrameSize, false));

    delete FrameSize;

    ros::NodeHandle nh;

    while(ros::ok() && SmallestSize >0 )
    {   /** Getting the latest frame**/
        CapObjectHolder1 >> Frame1;
        CapObjectHolder2 >> Frame2;
        /** Converting to GrayScale**/
        cv::cvtColor(Frame1,Frame1,CV_BGR2GRAY);
        cv::cvtColor(Frame2,Frame2,CV_BGR2GRAY);

        threadImageHolder.push_back(  boost::shared_ptr<boost::thread> (new boost::thread(boost::bind(&ImageInvert::ResizeImages,this,boost::ref(Frame1),SmallImage,FrameSizes,0 ))));
        threadImageHolder.push_back(  boost::shared_ptr<boost::thread> (new boost::thread(boost::bind(&ImageInvert::ResizeImages,this,boost::ref(Frame2),SmallImage,FrameSizes,1 ))));
        /**Waiting for threads to finish task **/
        for(auto it = threadImageHolder.begin();it!=threadImageHolder.end();it++)
            (*it)->join();
        threadImageHolder.clear();
        /** Writing to output video**/
        m_pVideoOutputptr->write(Frame1);
        m_pVideoOutputptr->write(Frame2);
        SmallestSize -=2;
    }
    ROS_INFO("Image Stiching Completed");
}

/*************************************/
/** Function to Resize a frame based on the smallest of the video frames**/
void ImageInvert::ResizeImages(cv::Mat &Image,bool status, int FrameSizes[4], int &image_number)
{
    int length;
    int width;
    /** If condition for resizes only the bigger frame based on the dimensions of the smaller frame**/
    if (status)
    {
        width = FrameSizes[0];
        length = FrameSizes[1];
        if(image_number == 1)
        {
            if(!Image.empty())
            {
                cv::resize(Image,Image,cv::Size(width,length),0, 0);
            }
            else
                ROS_ERROR("The Matrix is Empty.Cant Be Resized");
        }
    }
    else
    {
        length = FrameSizes[3];
        width = FrameSizes[2];
        if(image_number == 0)
        {
            if(!Image.empty())
                cv::resize(Image,Image,cv::Size(width,length),0, 0);
            else
                ROS_ERROR("The Matrix is Empty.Cant Be Resized");
        }
    }
 /** Calling the Invert Function for Inverting the pixel values**/
    InvertImages(Image);
}

/*************************************/


