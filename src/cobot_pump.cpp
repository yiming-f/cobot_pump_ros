#include "../include/cobot_pump_ros/cobot_pump.h"


int main(int argc, char **argv){

  cout << "Hello, world!" << endl;

    
  //   franka::VacuumGripper vacuum_gripper("192.168.130.1");

  //   vacuum_gripper.stop();

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
    

  //   return 0;
}