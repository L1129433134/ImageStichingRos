#include "node_merged_videos.h"
#include <memory>
#include <gtest/gtest.h>



struct StitchingTest:public ::testing::Test{
    std::string input_string[2] = {"/home/vishnu/Desktop/RosTutorials/catkin_ws/src/iris_automation_test/videos/dynamic_test.mp4" ,"/home/vishnu/Desktop/RosTutorials/catkin_ws/src/iris_automation_test/videos/field_trees.avi"};
    std::unique_ptr< ImageInvert > m_pobj;
    StitchingTest(){
         m_pobj.reset(new ImageInvert(2,input_string,500));
    }
    virtual ~StitchingTest()
    {}

};

  TEST_F(StitchingTest, checkForThreadNum)
    {
        EXPECT_EQ(500,m_pobj->num_threads_invert);
    }

  TEST_F(StitchingTest, checkForStringsNum)
    {
        EXPECT_EQ(2,m_pobj->m_iNumberStrings);
    }

  TEST_F(StitchingTest, checkFunctionDemoFunc)
    {   int data[4] = {1,2,3,4};
        cv::Mat Mat_test(2,2,0,&data);
        m_pobj->DemoFunc(0,0,Mat_test);
        EXPECT_EQ(254,Mat_test.at<int>(0,0));
    }

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
