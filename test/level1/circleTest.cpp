/**
 * @file circleTest.cpp
 * @brief unit testing of Circle class
 * @author Varun Asthana
 * @author Saumil Shah

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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gtest/gtest.h>
#include <string>
#include "circle.hpp"

TEST(CircleTest, testreadImage) {
  Circle a;
  cv::Mat refImg;
  std::string demoPath = "level1/demoImg.png";
  refImg= cv::imread(demoPath, 1);
  EXPECT_EQ(1,a.readImage(refImg));
}

TEST(CircleTest, testc2g) {
  Circle a;
  cv::Mat refImg;
  std::string demoPath = "level1/demoImg.png";
  refImg= cv::imread(demoPath, 1);
  ASSERT_TRUE(a.readImage(refImg));
  EXPECT_EQ(1,a.c2g());
}

TEST(CircleTest, testdetCircle) {
  Circle a;
  cv::Mat refImg;
  std::string demoPath = "level1/demoImg.png";
  refImg= cv::imread(demoPath, 1);
  ASSERT_TRUE(a.readImage(refImg));
  EXPECT_EQ(1,a.c2g());
  EXPECT_EQ(1, a.detCircle());
}

TEST(CircleTest, testgetImg) {
  Circle a;
  cv::Mat refImg;
  cv::Mat refImg2;
  std::string demoPath = "level1/demoImg.png";
  refImg= cv::imread(demoPath, 1);
  ASSERT_TRUE(a.readImage(refImg));
  EXPECT_EQ(1, a.getImg(refImg2));
}

TEST(CircleTest, testdgetData) {
  Circle a;
  cv::Mat refImg;
  std::string demoPath = "level1/demoImg.png";
  refImg= cv::imread(demoPath, 1);
  a.readImage(refImg);
  a.c2g();
  a.detCircle();
  std::vector<float> rad =a.getData();
  EXPECT_GT(rad.size(),0);
}
