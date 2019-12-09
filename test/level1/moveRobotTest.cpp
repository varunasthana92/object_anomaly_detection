#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gtest/gtest.h>
#include <iostream>
#include "ros/time.h"
#include "moveRobot.hpp"

class TestSubscriber
{
  public:
  	float position=0.0;
    TestSubscriber()
    {
      ros::NodeHandle n_;
      //Topic you want to subscribe
      ros::Subscriber sub_;
      sub_ = n_.subscribe("/arm_controller/command", 1, &TestSubscriber::callback, this);
      ros::Duration(0.5).sleep();
      std::cout << "\n\nCreated\n\n";
      // last_point_time_ = ros::Duration(1);  //Initial value
      spin();
    }
    void spin(){
    	int i=0;
		while(i<=10){ros::spinOnce();i++;ros::Duration(0.2).sleep();}
    }
    void callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
    {
    	position = msg->points[0].positions[0];
    	std::cout << "Set to "<< position;
    }
};

TEST(Check, c1){	
	// trajectory_msgs::JointTrajectory test_traj;
	// trajectory_msgs::JointTrajectoryPoint test_points_n;
	TestSubscriber t;
	MoveRobot mr;
	ros::Duration(2).sleep();
	
	// ros::Duration(2).sleep();
	mr.sendToPosition(1.0);
	t.spin();

	// std::cout << "Sen..Now wait..";
	// ros::Duration(20).sleep();

	// test_sub = test_node.subscribe("/trajectory_controller/command", 1, &callback);
	// mr.setDefaultPoint();
	ASSERT_EQ(t.position, 1);
}