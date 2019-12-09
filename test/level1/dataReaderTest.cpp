/**
 * @file dataReaderTest.cpp
 * @brief unit testing of DataReader class
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

#include <gtest/gtest.h>
#include <string>
#include "dataReader.hpp"

TEST(FileReaderTest, DataReadTest){
        std::string filename ="level1/test.txt";
	DataReader testReader;
	EXPECT_EQ(1,testReader.readAllData(filename));
}

TEST(FileReaderTest, PositionTest){
        std::string filename ="level1/test.txt";
	DataReader testReader;
	ASSERT_EQ(1,testReader.readAllData(filename));
	ASSERT_EQ(testReader.getPose(0,0),30);
	ASSERT_EQ(testReader.getPose(1,1),135);
}


TEST(FileReaderTest, DimentionaCheckTest){
        std::string filename ="level1/test.txt";
	DataReader testReader;
	ASSERT_EQ(testReader.readAllData(filename),1);
	ASSERT_EQ(testReader.checkDimentions(0,0,45),true);
	ASSERT_EQ(testReader.checkDimentions(2,0,52),false);
}

TEST(FileReaderTest, CheckNumberOfMeasurements){
        std::string filename ="level1/test.txt";
	DataReader testReader;
	ASSERT_EQ(testReader.readAllData(filename),1);
	ASSERT_EQ(testReader.getNumberOfMeasurements(1),2);
	ASSERT_EQ(testReader.getNumberOfMeasurements(0),4);
	ASSERT_EQ(testReader.getNumberOfMeasurements(2),3);
}

TEST(FileReaderTest, CheckNumberOfJobs){
        std::string filename ="level1/test.txt";
	DataReader testReader;
	ASSERT_EQ(testReader.readAllData(filename),1);
	ASSERT_EQ(testReader.getJobs(),3);
}
