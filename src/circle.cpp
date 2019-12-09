/**
 * @file circle.cpp
 * @brief defination of functions of class Circle
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

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "circle.hpp"

int Circle::readImage(cv::Mat &img) {
  image = img;  // Set the values to class variable
  return 1;
}

int Circle::c2g() {
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);  // Convert to gray scale image
  return 1;
}

int Circle::detCircle() {
  // Add gaussian filter to smooth the image. Necessary to tackle noise
  GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
  // Hough classifier to detect circles
  HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 2, 200, 35, 0, 0);
  cv::Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
  int radius = cvRound(circles[0][2]);
  rad.push_back(circles[0][2]);
  // circle center
  cv::circle(image, center, 3, cv::Scalar(10, 255, 128), -1, 8, 0);
  // circle outline
  cv::circle(image, center, radius, cv::Scalar(255, 0, 126), 2, 8, 0);
  return 1;
}

int Circle::getImg(cv::Mat &img) {
  img = image;
  return 1;
}

std::vector<float> Circle::getData() {
  return rad;
}

Circle::~Circle() {
}

