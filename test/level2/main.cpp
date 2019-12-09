#include <gtest/gtest.h>
#include <ros/ros.h>

/*
 * @brief main function which runs all test results.
 *
 * @param This function takes the commandline arguments as input.
 *
 * @return This function returns a 0 just to avoid error.
 */
int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "level2Tests");

    // Initialize Google Test
    testing::InitGoogleTest(&argc, argv);

    // Run all tests
    return RUN_ALL_TESTS();
}
