/**
 * @file dataReader.hpp
 * @brief Class declaration of DataReader class. This class reads the data from text file
 *        and saves into local containers.
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
#ifndef _INCLUDE_DATAREADER_HPP_
#define _INCLUDE_DATAREADER_HPP_

#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<dataFetch.hpp>

class DataReader : public DataFetch {
 public:
  /**
   * @brief This method reads all the data from text file and saves it into vectors
   * @param filePath of type string
   * @return 1 if read successfully else 0
   */
  int readAllData(std::string filePath);

  /**
   * @brief This method gives the number of measurements for a given job number
   * @param jobNumber of type int
   * @return 1 if read successfully else 0
   */
  int getNumberOfMeasurements(int jobNumber);

  /**
   * @brief This method gives the position where measurement need to be taken
   * @param jobNumber of type int
   * @param measurementNumber of type int
   * @return Angle in degrees read from file of type float
   */
  float getPose(int jobNumber, int measurementNumber);

  /**
   * @brief Method gives the number of jobs available
   * @param none
   * @return Number of jobs of type int
   */
  int getJobs();

  /**
   * @brief Method checks if the measured dimention is in tolerance range or not
   * @param jobNumber of type int
   * @param measurementNumber of type int
   * @param measuredDimention of type float
   * @return true if dimention is in tolerance zone and false otherwise
   */
  bool checkDimentions(int jobNumber, int measurementNumber,
                       float measuredDimention);
};
#endif  /* _INCLUDE_DATAREADER_HPP_ */
