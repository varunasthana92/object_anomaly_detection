/**
 * @file circle.hpp
 * @brief detect and highlight circles in an image
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
#ifndef _INCLUDE_CIRCLE_HPP_
#define _INCLUDE_CIRCLE_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Circle {
 private:
  cv::Mat image; /**< variable to hold image input */
  cv::Mat gray; /**< variable to hold converted grayscale image */
  std::vector<cv::Vec3f> circles; /**< variable to data for all detected circles in an image */
  std::vector<float> rad;

 public:
  /**
   * @brief function to get input image
   * @param img cv::Mat&
   * @return int
   */
  int readImage(cv::Mat &img);

  /**
   * @brief function to convert RBG image to grayscale
   * @param none
   * @return int
   */
  int c2g();

  /**
   * @brief function to identfy and draw circles in an image
   * @param none
   * @return int
   */
  int detCircle();

  /**
   * @brief function to get reference to the modified image with highlighted circles in an image
   * @param img cv::Mat&
   * @return int
   */
  int getImg(cv::Mat &img);

  /**
   * @brief function to return radius of the circle
   * @param img cv::Mat&
   * @return float
   */
  std::vector<float> getData();

  /**
   * @brief destructor for ImageCnvert
   * @param none
   * @return none
   */
  ~Circle();
};
#endif  /* _INCLUDE_CIRCLE_HPP_ */
