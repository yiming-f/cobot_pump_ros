#include <iostream>
#include <chrono>
#include <thread>
#include <franka/exception.h>
#include <franka/vacuum_gripper.h>
#include "ros/ros.h"
#include "cobot_pump_ros/startPump.h"
#include "cobot_pump_ros/stopPump.h"
#include "cobot_pump_ros/dropItem.h"
#include "cobot_pump_ros/readState.h"
#include "cobot_pump_ros/checkItemAttached.h"

franka::VacuumGripper vacuum_gripper("panda-control");

bool drop(cobot_pump_ros::dropItem::Request &req, cobot_pump_ros::dropItem::Response &res) {
    try {
    	res.success = vacuum_gripper.dropOff(std::chrono::milliseconds(req.timeout_ms));
    } catch(franka::Exception const& e) {
    	std::cout << "Drop Exception: " << e.what() << std::endl;
	return false;
    }

    return true;
}

bool stop(cobot_pump_ros::stopPump::Request &req, cobot_pump_ros::stopPump::Response &res) {
    try {
	res.success = vacuum_gripper.stop();
    } catch(franka::Exception const& e) {
    	std::cout << "Stop Exception: " << e.what() << std::endl;
	return false;
    }

    return true;
}

franka::VacuumGripperState state(cobot_pump_ros::readState::Request &req, cobot_pump_ros::readState::Response &res) {
    franka::VacuumGripperState vacuum_gripper_state;
    try {
        vacuum_gripper_state = vacuum_gripper.readOnce();
	res.itemAttached = vacuum_gripper_state.part_present;
    } catch(franka::Exception const& e) {
    	std::cout << "State Exception: " << e.what() << std::endl;
    }

    return vacuum_gripper_state;
}

bool start(cobot_pump_ros::startPump::Request &req, cobot_pump_ros::startPump::Response &res) {
    try {
        bool success = vacuum_gripper.vacuum(req.pressure, std::chrono::milliseconds(req.timeout_ms));
	res.vacuumSuccess = success;
    } catch(franka::Exception const& e) {
    	std::cout << "Start Exception: " << e.what() << std::endl;
	res.vacuumSuccess = false;
    }

    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "cobot_pump");
    ros::NodeHandle nh;

    auto startPump_Service = nh.advertiseService("startPump", start);
    auto stopPump_Service = nh.advertiseService("stopPump", stop);
    auto dropItem_Service = nh.advertiseService("dropItem", drop);
    //auto readState_Service = nh.advertiseService("readState", state);
    //checkItemAttached_Service = nh.advertiseService("checkItemAttached", checkItemAttached);

    ros::spin();
    return 0;
}
