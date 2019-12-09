/**
 * @file imageConvert.hpp
 * @brief convert ros images into openCV image format
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
#ifndef _INCLUDE_IMAGECONVERT_HPP_
#define _INCLUDE_IMAGECONVERT_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

class ImageConvert {
 private:
  ros::NodeHandle nh_; /**< node handle variable for ros node functionality */
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  cv_bridge::CvImagePtr img_ptr;

 public:
  /**
   * @brief constructor for ImageCnvert
   * @param none
   * @return none
   */
  ImageConvert();

  /**
   * @brief destructor for ImageCnvert
   * @param none
   * @return none
   */
  ~ImageConvert();

  /**
   * @brief callback function to read image from rostopic
   * @param img sensor_Msgs::ImageConstPtr&
   * @return void
   */
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  /**
   * @brief function to return image in cv::Mat format by reference
   * @param img cv::Mat&
   * @return int
   */
  int getImage(cv::Mat &img);

  /**
   * @brief function to publish modified image on ros topic
   * @param img cv::Mat&
   * @return void
   */
};
#endif  /* _INCLUDE_IMAGECONVERT_HPP_ */
