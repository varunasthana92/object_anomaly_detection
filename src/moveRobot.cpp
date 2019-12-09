/**
 * @file moveRobot.cpp
 * @brief Method implementation of class MoveRobot
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
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"
#include "moveRobot.hpp"

// Destructor of class
MoveRobot::~MoveRobot() {
}

int MoveRobot::sendToPosition(float angle) {
  traj.header.stamp = ros::Time::now();
  setValeurPoint(&traj, angle);  // Sets the value of angle in message
  traj.points[0].time_from_start = ros::Duration(2);
  ROS_INFO_STREAM("Move");
  arm_pub.publish(traj);  // Publishes the message
  return 1;
}

int MoveRobot::setDefaultPoint(trajectory_msgs::JointTrajectory *trajectoire) {
  // Sets the default joint angles to message
  trajectoire->points[0].positions[1] = -0.918;
  trajectoire->points[0].positions[2] = 1.529;
  trajectoire->points[0].positions[3] = -0.6778;
  trajectoire->points[0].positions[4] = -0.02287;
  trajectoire->points[0].positions[5] = 0.0581798;
  return 0;
}

int MoveRobot::setValeurPoint(trajectory_msgs::JointTrajectory *trajectoire,
                              float val) {
  // Sets the value of angle to first joint
  trajectoire->points[0].positions[0] = val * 3.1457 / 180 - 0.02
      - 15 * 3.14 / 180;
  return 0;
}

MoveRobot::MoveRobot() {
  // Create the publisher
  arm_pub = n.advertise < trajectory_msgs::JointTrajectory
      > ("/arm_controller/command", 1);
  traj.header.frame_id = "base_link";  // Base frame for trajectory
  traj.joint_names.resize(6);  // 6 available joints
  traj.points.resize(1);
  traj.points[0].positions.resize(6);
  // Name of all the 6 joints
  traj.joint_names[0] = "shoulder_pan_joint";
  traj.joint_names[1] = "shoulder_lift_joint";
  traj.joint_names[2] = "elbow_joint";
  traj.joint_names[3] = "wrist_1_joint";
  traj.joint_names[4] = "wrist_2_joint";
  traj.joint_names[5] = "wrist_3_joint";
  setDefaultPoint (&traj);  // Set default joint values
}
