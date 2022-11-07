# cobot_pump_ros
ROS package for the cobot pump using the libfranka middleware. Has multiple services that enable the user to control the pump from python or c++ service calls.

## Services

Implemented services provide a minimal method of controlling individual features of the pump, the current services are:

### startPump
Turns the vacuum pump on at the desired vacuum strength with a specified timeout. If a vacuum is not established within the timeout, the service returns false and stops the pump.
#### Inputs
- pressure - Specified pressure of the vacuum pump.
- timeout_ms - Timeout for the vacuum pump before turning the pump off.

#### Returns
- vacuumSuccess - true if vacuum established, false otherwise

### stopPump
Stops the vacuum pump.
#### Returns
- success - true if pump stopped, false otherwise.

### dropItem
Ejects any items off the vacuum pump, return true if object is ejcted and false otherwise.
#### Inputs
- timeout_ms - Timeout for pump to drop item.

#### Returns
- success - true if object dropped, false otherwise (BUGGED - returns false despite dropping the object often)

### readState
TODO - Will read the state of the vacuum gripper and compose a custom ROS message to be returned.
#### Returns
- vacuumState as specified by libfranka (TODO - not fully implemented)

### checkItemAttached
Checks whether there is still an object present on the vacuum gripper.
#### Returns
- itemAttached - true if object attached to vacuum gripper, false otherwise.

## Demo Scripts
basic_demo.py Simply turns on the vacuum pump using the startPump service. Then it checks whether it was successful. If an object was grapsed it will wait for 3 seconds, check the status of the pump to see if the object is still being grapsed and then drop the object. Finally it will check if the object is attached after dropping it.

## Using this package

### Note
due to the current bug with the drop item service, you should not rely on the return statement for any logical conditions in your code as it often returns false when it should reutrn true,instead use the dropItem service and afterwards call "checkItemAttached" to see if there is any item present on the pump as this works correctly.

### To run the node
Use a launch file to launch the cobot_pump_node executable using the ROS parameter server to tell the script the ip-address o your franka-panda robot. An example of this cn be seen in the lunch folder under "example.launch"

### To run the demo script
```
rosrun cobot_pump_ros basic_demo.py
```

## Authors
[David Russell][el16dmcr@leeds.ac.uk]




