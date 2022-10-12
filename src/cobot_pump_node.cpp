#pragma once

#include <cmath>
#include <string>
#include <iostream>
#include <thread>

// #include <franka/robot.h>
// #include <franka/vacuum_gripper.h>
// #include <franka/exception.h>

#include "ros/ros.h"
#include "cobot_pump_ros/vacuumService.h"

using namespace std;

bool add(cobot_pump_ros::vacuumService::Request &req, cobot_pump_ros::vacuumService::Response &res){
  res.sum = req.a + req.b;
  return true;
}

int main(int argc, char **argv){

  cout << "Hello, world!, this is the node with services" << endl;
  ros::init(argc, argv, "cobot_pump");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);

  ros::spin();
}
