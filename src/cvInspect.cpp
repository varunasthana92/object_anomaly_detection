/**
 * @file compare.hpp
 * @brief compare measured and expected radius of circle
 * @author Varun Asthana
 * @author Saumil Shah
 *
 * Copyright [2019] Group 20
 * All rights reserved.
 *
 * BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "imageConvert.hpp"
#include "circle.hpp"
#include "dataReader.hpp"
#include "dataFetch.hpp"
#include "moveRobot.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cvInspect");  // Ros initialization
  MoveRobot robo;  // Object of MoveRobot class
  std::string filePath;  // Path of file which has information about measurements
  const std::string paramName = "~filePath";
  bool check = ros::param::get(paramName, filePath);
  if (!check) {
    while (ros::ok()) {
      ROS_FATAL_STREAM("Could not get parameter " << paramName);
    }
    return 0;
  }
  ImageConvert ic;
  Circle circ;
  cv::Mat image;
  DataReader dr;
  static const std::string OPENCV_WINDOW = "Image window";
  cv::namedWindow(OPENCV_WINDOW);

  if (dr.readAllData(filePath)) {
    ros::Rate loop = 1;
    int jobNum;
    // Get the user data which about the job user want to measure
    ROS_INFO_STREAM(
        "No. of jobs available on table to inspect: " << dr.getJobs());
    do {
      do {
        ROS_INFO_STREAM(
            "Enter the job number to be checked (<=No. of Jobs on the table)\n");
        ROS_INFO_STREAM("Or else enter 0 to terminate the ternimal\n");
        ROS_INFO_STREAM(
            "To exit gazebo, use Ctrl + C on the terminal used to launch the package.\n\nUser Input: ");
        std::cin >> jobNum;
        if (jobNum < 0 || jobNum > dr.getJobs()) {
          ROS_INFO_STREAM("Wrong input!!!!\nTry again...");
        }
      } while (jobNum < 0 || jobNum > dr.getJobs());  // That job should be present in the file
      if (jobNum == 0) {
        break;
      }
      int jobIndex = jobNum - 1;
      int numHoles = dr.getNumberOfMeasurements(jobIndex);
      ROS_INFO_STREAM("Need to take " << numHoles << " measurements\n");
      int i = 0;
      int jobOk = 1;
      while (ros::ok() && i < numHoles) {
        // Get the pose where you want to mwasure
        float pose = dr.getPose(jobIndex, i);
        ROS_INFO_STREAM("Rotating robot to position " << pose);
        // Pass the angle to rotate and wait for the robot to move
        robo.sendToPosition(pose);
        ros::Duration(4).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("Getting the image");
        ic.getImage(image);
        circ.readImage(image);
        circ.c2g();
        circ.detCircle();
        circ.getImg(image);
        std::vector<float> rad = circ.getData();
        if (!rad.size()) {
          ROS_INFO_STREAM("No hole(s) detected\nRejecting job\n");
          jobOk = 0;
          break;
        } else {
          ROS_INFO_STREAM("Detected radius: " << rad[0]);
          bool check = dr.checkDimentions(jobIndex, i, rad[0]);
          if (check) {
            ROS_INFO_STREAM("Hole " << i + 1 << " dimension OK");
          } else {
            ROS_INFO_STREAM("Hole " << i + 1 << " dimension NOT OK\n\n");
            jobOk = 0;
          }
        }
        cv::imshow(OPENCV_WINDOW, image);  // Shows the imgage on which measurement has been taken
        cv::waitKey(3);
        loop.sleep();
        ++i;
      }
    } while (1);
    return 1;
  } else {
    ROS_INFO_STREAM("Unable to read data file\n");
    return 0;
  }
}

