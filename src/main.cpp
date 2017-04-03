
#include "node_merged_videos.h"



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "video_publisher");

    std::string InputString[2] = {argv[1],argv[2]};
    ImageInvert Imgin(2,InputString,  strtol(argv[3], NULL, 10));
    Imgin.StartStitchVideos();
    ros::spin();
}
