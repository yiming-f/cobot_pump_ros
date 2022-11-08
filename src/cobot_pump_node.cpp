

#include "cobot_pump_node.h"


frankaPump::frankaPump(std::string _frankaIP, ros::NodeHandle *n){
    frankaIP = _frankaIP;

    // Advertise services for the vacuum gripper
    startPump_Service = n->advertiseService("startPump", &frankaPump::startPump, this);
    stopPump_Service = n->advertiseService("stopPump", &frankaPump::stopPump, this);
    dropItem_Service = n->advertiseService("dropItem", &frankaPump::dropItem, this);
    readState_Service = n->advertiseService("readState", &frankaPump::readState, this);
    checkItemAttached_Service = n->advertiseService("checkItemAttached", &frankaPump::checkItemAttached, this);

}

//------------------------------------------------------------
//
//          Turns pump on at set pressure
//
//------------------------------------------------------------
bool frankaPump::startPump(cobot_pump_ros::startPump::Request &req, cobot_pump_ros::startPump::Response &res){
    vacuumPump = new franka::VacuumGripper(frankaIP);
    
    try{
        res.vacuumSuccess = vacuumPump->vacuum(req.pressure, std::chrono::milliseconds(req.timeout_ms));

    }
    catch(franka::CommandException){
        std::cout << "START PUMP - Command exception" << std::endl;
    }
    catch(franka::NetworkException){
        std::cout << "START PUMP - Network exception" << std::endl;
    }

    if(res.vacuumSuccess == false){
        vacuumPump->stop();
    }

    vacuumPump->~VacuumGripper();
    
    return true;
}

//------------------------------------------------------------
//
//                      Turns pump off
//
//------------------------------------------------------------
bool frankaPump::stopPump(cobot_pump_ros::stopPump::Request &req, cobot_pump_ros::stopPump::Response &res){
    vacuumPump = new franka::VacuumGripper(frankaIP);

    try{
        res.success = vacuumPump->stop();
    }
    catch(franka::CommandException){
        std::cout << "STOP PUMP - Command exception" << std::endl;
    }
    catch(franka::NetworkException){
        std::cout << "STOP PUMP - Network exception" << std::endl;
    }

    vacuumPump->~VacuumGripper();
    
    return true;
}

//------------------------------------------------------------
//
//                      Drops objects
//
//------------------------------------------------------------
bool frankaPump::dropItem(cobot_pump_ros::dropItem::Request &req, cobot_pump_ros::dropItem::Response &res){
    vacuumPump = new franka::VacuumGripper(frankaIP);

    try{
        res.success = vacuumPump->dropOff(std::chrono::milliseconds(req.timeout_ms));
    }
    catch(franka::CommandException){
        std::cout << "DROP ITEM - Command exception" << std::endl;
    }
    catch(franka::NetworkException){
        std::cout << "DROP ITEM - Network exception" << std::endl;
    }

    vacuumPump->~VacuumGripper();

    return true;
}

//------------------------------------------------------------
//
//                Reads state of the pump (TODO)
//
//------------------------------------------------------------
bool frankaPump::readState(cobot_pump_ros::readState::Request &req, cobot_pump_ros::readState::Response &res){
    vacuumPump = new franka::VacuumGripper(frankaIP);

    franka::VacuumGripperState vacuum_gripper_state = vacuumPump->readOnce();
    std::cout << "vacuum state is: " << vacuum_gripper_state << std::endl;
    res.itemAttached = vacuum_gripper_state.part_present;

    vacuumPump->~VacuumGripper();

    return true;
}

//------------------------------------------------------------
//
//  Checks if an item is attached to pump based on vacuum strength
//
//------------------------------------------------------------
bool frankaPump::checkItemAttached(cobot_pump_ros::checkItemAttached::Request &req, cobot_pump_ros::checkItemAttached::Response &res){
    vacuumPump = new franka::VacuumGripper(frankaIP);
    //franka::VacuumGripper vacuumPump(frankaIP);

    res.itemAttached  = false;

    franka::VacuumGripperState vacuum_gripper_state = vacuumPump->readOnce();
    std::cout << "vacuum gripper state: " << vacuum_gripper_state << std::endl;

    // Additional method of checking if item attached as "in_control_range" doesnt always appear to work
    std::cout << "vacuum strength: " << vacuum_gripper_state.vacuum << std::endl;
    if(vacuum_gripper_state.vacuum >= VACUUM_THRESHOLD_ITEM_ATTACHED){
        res.itemAttached = true;
    }

    vacuumPump->~VacuumGripper();

    return true;
}

int main(int argc, char **argv){

    // Create the ros node (cobot_pump)
    ros::init(argc, argv, "cobot_pump");
    ros::NodeHandle n;

    std::string _frankaIP;
    
    ros::param::get("/franka_ip", _frankaIP);

    std::cout << "franka ip is: " << _frankaIP << std::endl;

    frankaPump frankaPump(_frankaIP, &n);

    ros::spin();

    return 0;
}


