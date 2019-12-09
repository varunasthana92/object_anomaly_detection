/**
 * @file moveRobot.hpp
 * @brief Moves UR5 robot to get to desired location
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
#ifndef _INCLUDE_MOVEROBOT_HPP_
#define _INCLUDE_MOVEROBOT_HPP_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"

class MoveRobot {
 private:
  /**
   * @brief Node handle for ROS
   */
  ros::NodeHandle n;

  /**
   * @brief Publisher which publishes joint values in order to rotate robot
   */
  ros::Publisher arm_pub;

  /**
   * @brief Object for trajectory messages
   */
  trajectory_msgs::JointTrajectory traj;

  /**
   * @brief Object which contains joint angles
   */
  trajectory_msgs::JointTrajectoryPoint points_n;

 public:
  /**
   * @brief Constructor of the moveRobotclass
   * @param none
   * @return none
   */
  MoveRobot();

  /**
   * @brief Destructor of the moveRobotclass
   * @param none
   * @return none
   */
  ~MoveRobot();

  /**
   * @brief This method sets new position in message
   * @param trajectoire pointer and val of type float
   * @return 1 if successful and 0 if not
   */
  int setValeurPoint(trajectory_msgs::JointTrajectory *trajectoire, float val);

  /**
   * @brief This method sets default zero position to message
   * @param trajectoire pointer
   * @return 1 if successful and 0 if not
   */
  int setDefaultPoint(trajectory_msgs::JointTrajectory *trajectoire);

  /**
   * @brief This method sets new position and publishes that to move the robot
   * @param angle of type float
   * @return 1 if successful and 0 if not
   */
  int sendToPosition(float angle);
};
#endif  /* _INCLUDE_MOVEROBOT_HPP_ */
