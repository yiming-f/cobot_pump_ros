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

// Instantiate vacuum gripper with our ip address
std::string frankaIP;
// our franka ip is 192.168.130.1
//franka::VacuumGripper vacuum_gripper("192.168.130.1");

//------------------------------------------------------------
//
//          Turns pump on at set pressure
//
//------------------------------------------------------------
bool startPump(cobot_pump_ros::startPump::Request &req, cobot_pump_ros::startPump::Response &res){
    franka::VacuumGripper vacuum_gripper(frankaIP);
    
    try{
        res.vacuumSuccess = vacuum_gripper.vacuum(req.pressure, std::chrono::milliseconds(req.timeout_ms));

    }
    catch(franka::CommandException){
        std::cout << "START PUMP - Command exception" << std::endl;
    }
    catch(franka::NetworkException){
        std::cout << "START PUMP - Network exception" << std::endl;
    }

    // else{
    //     // automatically stop the pump if vacuum not established
    //     vacuum_gripper.stop();
    //     res.vacuumSuccess = false;

    // }

    if(res.vacuumSuccess == false){
        vacuum_gripper.stop();
    }

    

    // Sleep the thread for a small time to allow update to vacuum state
    //std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "vacuum state is: " << vacuum_gripper_state << std::endl;

    // if(vacuum_gripper_state.in_control_range == true){
    //     std::cout << "vacuum in correct control range: " << vacuum_gripper_state << std::endl;
    //     res.vacuumSuccess = true;
    // }
    
    return true;
}

//------------------------------------------------------------
//
//                      Turns pump off
//
//------------------------------------------------------------
bool stopPump(cobot_pump_ros::stopPump::Request &req, cobot_pump_ros::stopPump::Response &res){
    franka::VacuumGripper vacuum_gripper(frankaIP);

    try{
        res.success = vacuum_gripper.stop();
    }
    catch(franka::CommandException){
        std::cout << "STOP PUMP - Command exception" << std::endl;
    }
    catch(franka::NetworkException){
        std::cout << "STOP PUMP - Network exception" << std::endl;
    }
    
    return true;
}

//------------------------------------------------------------
//
//                      Drops objects
//
//------------------------------------------------------------
bool dropItem(cobot_pump_ros::dropItem::Request &req, cobot_pump_ros::dropItem::Response &res){
    franka::VacuumGripper vacuum_gripper(frankaIP);

    try{
        res.success = vacuum_gripper.dropOff(std::chrono::milliseconds(req.timeout_ms));
    }
    catch(franka::CommandException){
        std::cout << "DROP ITEM - Command exception" << std::endl;
    }
    catch(franka::NetworkException){
        std::cout << "DROP ITEM - Network exception" << std::endl;
    }

    // Sleep the thread for a small time to allow update to vacuum state
    //std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1000));


    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "vacuum state is: " << vacuum_gripper_state << std::endl;

    // if(vacuum_gripper_state.part_detached == true){
    //     std::cout << "item dropped: " << vacuum_gripper_state << std::endl;
    //     res.success = true;
    // }

    return true;
}

bool readState(cobot_pump_ros::readState::Request &req, cobot_pump_ros::readState::Response &res){
    franka::VacuumGripper vacuum_gripper(frankaIP);

    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "vacuum state is: " << vacuum_gripper_state << std::endl;
    res.itemAttached = vacuum_gripper_state.part_present;

    return true;

}

bool checkItemAttached(cobot_pump_ros::checkItemAttached::Request &req, cobot_pump_ros::checkItemAttached::Response &res){
    franka::VacuumGripper vacuum_gripper(frankaIP);

    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "is vacuum in control range?:  " << vacuum_gripper_state.part_present << std::endl;
    res.itemAttached = vacuum_gripper_state.in_control_range;

    return true;
}

int main(int argc, char **argv){

    std::cout << "Hello, world!, V2 I have changed this!!! "<< std::endl;

    std::cout << "franka ip is: " << argv[1] << std::endl;

    frankaIP = argv[1];

    // Print a vacuum gripper state.
    //franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    //std::cout << "Initial vacuum gripper state: " << vacuum_gripper_state << std::endl;

    // Create the ros node (cobot_pump)
    ros::init(argc, argv, "cobot_pump");
    ros::NodeHandle n;

    // Advertise services for the vacuum gripper
    ros::ServiceServer startPump_Service = n.advertiseService("startPump", startPump);
    ros::ServiceServer stopPump_Service = n.advertiseService("stopPump", stopPump);
    ros::ServiceServer dropItem_Service = n.advertiseService("dropItem", dropItem);
    ros::ServiceServer readState_Service = n.advertiseService("readState", readState);
    ros::ServiceServer checkItemAttached_Service = n.advertiseService("checkItemAttached", checkItemAttached);

    ros::spin();
}