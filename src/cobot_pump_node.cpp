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

using namespace std;

// TODO - make this ip changeable when you create the node, i.e via arguments
// Instantiate vacuum gripper with our ip address
// franka::VacuumGripper vacuum_gripper("192.168.130.1");

//------------------------------------------------------------
//
//          Turns pump on at set pressure
//
//------------------------------------------------------------
bool startPump(cobot_pump_ros::startPump::Request &req, cobot_pump_ros::startPump::Response &res){
    franka::VacuumGripper vacuum_gripper("192.168.130.1");
    
    try{
        vacuum_gripper.vacuum(req.pressure, std::chrono::milliseconds(req.timeout_ms));
    }
    catch(franka::CommandException){
        cout << "START PUMP - Command exception" << endl;
    }
    catch(franka::NetworkException){
        cout << "START PUMP - Network exception" << endl;
    }

    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "vacuum state is: " << vacuum_gripper_state << std::endl;

    if(vacuum_gripper_state.in_control_range == true){
        std::cout << "vacuum in correct control range: " << vacuum_gripper_state << std::endl;
        res.vacuumSuccess = true;
    }
    else{
        // automatically stop the pump if vacuum not established
        vacuum_gripper.stop();
        res.vacuumSuccess = false;

    }
    
    return true;
}

//------------------------------------------------------------
//
//                      Turns pump off
//
//------------------------------------------------------------
bool stopPump(cobot_pump_ros::stopPump::Request &req, cobot_pump_ros::stopPump::Response &res){
    franka::VacuumGripper vacuum_gripper("192.168.130.1");

    try{
        res.success = vacuum_gripper.stop();
    }
    catch(franka::CommandException){
        cout << "STOP PUMP - Command exception" << endl;
    }
    catch(franka::NetworkException){
        cout << "STOP PUMP - Network exception" << endl;
    }
    
    return true;
}

//------------------------------------------------------------
//
//                      Drops objects
//
//------------------------------------------------------------
bool dropItem(cobot_pump_ros::dropItem::Request &req, cobot_pump_ros::dropItem::Response &res){
    franka::VacuumGripper vacuum_gripper("192.168.130.1");
    res.success = false;

    try{
        vacuum_gripper.dropOff(std::chrono::milliseconds(req.timeout_ms));
    }
    catch(franka::CommandException){
        cout << "DROP ITEM - Command exception" << endl;
    }
    catch(franka::NetworkException){
        cout << "DROP ITEM - Network exception" << endl;
    }

    franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    std::cout << "vacuum state is: " << vacuum_gripper_state << std::endl;

    if(vacuum_gripper_state.part_detached == false){
        std::cout << "item dropped: " << vacuum_gripper_state << std::endl;
        res.success = true;
    }

    return true;
}

int main(int argc, char **argv){

    cout << "Hello, world!, Initial vacuum testing "<< endl;

    // Print a vacuum gripper state.
    //franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
    //std::cout << "Initial vacuum gripper state: " << vacuum_gripper_state << std::endl;



    // Establish an initial connection to the franka panda
    

    // Create the ros node (cobot_pump)
    ros::init(argc, argv, "cobot_pump");
    ros::NodeHandle n;

    // Advertise services for the vacuum gripper
    ros::ServiceServer startPump_Service = n.advertiseService("startPump", startPump);
    ros::ServiceServer stopPump_Service = n.advertiseService("stopPump", stopPump);
    ros::ServiceServer dropItem_Service = n.advertiseService("dropItem", dropItem);

    ros::spin();
}


  //   

  //   try {
  //   // Print a vacuum gripper state.
  //   franka::VacuumGripperState vacuum_gripper_state = vacuum_gripper.readOnce();
  //   std::cout << "Initial vacuum gripper state: " << vacuum_gripper_state << std::endl;

  //   // Vacuum the object.
  //   if (!vacuum_gripper.vacuum(500, std::chrono::milliseconds(1000))) {
  //     std::cout << "Failed to vacuum the object." << std::endl;
  //     return -1;
  //   }

  //   vacuum_gripper_state = vacuum_gripper.readOnce();
  //   std::cout << "Vacuum gripper state after applying vacuum: " << vacuum_gripper_state
  //             << std::endl;

  //   // Wait 3s and check afterwards, if the object is still grasped.
  //   std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

  //   vacuum_gripper_state = vacuum_gripper.readOnce();
  //   if (!vacuum_gripper_state.in_control_range) {
  //     std::cout << "Object lost." << std::endl;
  //     return -1;
  //   }

  //   std::cout << "Vacuumed object, will release it now." << std::endl;
  //   vacuum_gripper.dropOff(std::chrono::milliseconds(1000));
  // } catch (franka::Exception const& e) {
  //   vacuum_gripper.stop();
  //   std::cout << e.what() << std::endl;
  //   return -1;
  // }