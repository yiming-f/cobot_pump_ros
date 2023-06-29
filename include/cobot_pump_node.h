#pragma once

// General Includes
#include <cmath>
#include <string>
#include <iostream>
#include <thread>

// libFranka includes
#include <franka/robot.h>
#include <franka/vacuum_gripper.h>
#include <franka/vacuum_gripper_state.h>
#include <franka/exception.h>

// ROS + services includes
#include "ros/ros.h"
#include "cobot_pump_ros/startPump.h"
#include "cobot_pump_ros/stopPump.h"
#include "cobot_pump_ros/dropItem.h"
#include "cobot_pump_ros/readState.h"
#include "cobot_pump_ros/checkItemAttached.h"

#define VACUUM_THRESHOLD_ITEM_ATTACHED 80

class frankaPump{
    public:
        frankaPump(std::string _frankaIP, ros::NodeHandle *n);

        // Service calls
        bool checkItemAttached(cobot_pump_ros::checkItemAttached::Request &req, cobot_pump_ros::checkItemAttached::Response &res);
        bool readState(cobot_pump_ros::readState::Request &req, cobot_pump_ros::readState::Response &res);
        bool dropItem(cobot_pump_ros::dropItem::Request &req, cobot_pump_ros::dropItem::Response &res);
        bool stopPump(cobot_pump_ros::stopPump::Request &req, cobot_pump_ros::stopPump::Response &res);
        bool startPump(cobot_pump_ros::startPump::Request &req, cobot_pump_ros::startPump::Response &res);

    private:

        // Ip address of franka robot
        std::string frankaIP;
        
        // Defining ROS services for vacuum pump
        ros::ServiceServer startPump_Service;
        ros::ServiceServer stopPump_Service;
        ros::ServiceServer dropItem_Service;
        ros::ServiceServer readState_Service;
        ros::ServiceServer checkItemAttached_Service;

        // Defining vacuumPump object to be instantiated in constructor
        //franka::VacuumGripper vacuumPump = franka::VacuumGripper("panda-control");
        
};
