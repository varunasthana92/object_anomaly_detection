/**
 * @file dataReader.cpp
 * @brief Read the file and save the measurement inputs to containers
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
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<dataReader.hpp>

int DataReader::getNumberOfMeasurements(int jobNumber){
	// returns the size of vector which is number of jobs
	return limitations[jobNumber].size();
}

float DataReader::getPose(int jobNumber, int measurementNumber){
	// returns the size of 2nd level of vector which is number of measument of a particular job
	return angles[jobNumber][measurementNumber];
}

bool DataReader::checkDimentions(int jobNumber, int measurementNumber, float measuredDimention){
	std::cout<<"\nExpected measurement between: "<<limitations[jobNumber][measurementNumber][0];
	std::cout<<" and "<<limitations[jobNumber][measurementNumber][1]<<"\n";
	// Checks if measured data is in tolerance range or not
	if (measuredDimention <= limitations[jobNumber][measurementNumber][0] && measuredDimention >= limitations[jobNumber][measurementNumber][1])
		return true;
	else
		return false;
}
	
int DataReader::readAllData(std::string filePath){
	myReadFile.open(filePath);  // Opens the file
	std::string line;
	if (myReadFile.is_open()) {
		while (!myReadFile.eof()) {
		 	getline (myReadFile,line);	// Get the next line from file
			if (line[0]==35){  // ASCII value of # --- Find #
				std::vector<std::vector<float>> limitationsForJob;  /// Maximum and minimum for one job
				std::vector<float> anglesForJob; //angles for one job
				numOfJobs++;
				// Job name is before " : " --- read that
		    		jobNames.push_back(line.substr(1,line.find(":")-1));
		    		// Number of measurements is defined after " = " sign
		 		auto found = line.find("=");
		 		// Read string after " = " and convert into int
		 		noOfMeasurements = stoi(line.substr(found+1));
				int i=0;
				while(i<noOfMeasurements){
   				 	getline (myReadFile,line);  // Ignore the next line
	 			 	getline (myReadFile,line);
	 			 	if (line.substr(0,5)=="angle"){  
	 			 		// Read the string data of angle and convert into float
	 			 		anglesForJob.push_back(stof(line.substr(7)));
	 			 	}
	 			 	getline(myReadFile,line);
	 			 	if (line.substr(0,11) == "(Dmax Dmin)"){
	 			 		line.erase(0,11);
						std::vector<float> tempLimitations;
						// Read the min and max value of dimentions from file and add to vector
	 			 		tempLimitations.push_back(stof(line.substr(line.find("(")+1 , line.find(",")- line.find("(")-1 )));
						tempLimitations.push_back(stof(line.substr(line.find(",")+1 , line.find(")")- line.find(",")-1 )));		
						limitationsForJob.push_back(tempLimitations);
	 			 	}
					++i;
	 			}
				limitations.push_back(limitationsForJob);  // Add tolerances into container			
				angles.push_back(anglesForJob);  // Add angles into containers			    	
	    		}
		}
		return 1;
	} else {
		return 0;
	}
}

int DataReader::getJobs(){
	// Returns the number of getJobs
	return numOfJobs;
}
