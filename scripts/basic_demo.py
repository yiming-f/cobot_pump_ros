#!/usr/bin/env python

import sys
import rospy 
from cobot_pump_ros.srv import *
import time


def basic_demo():
    rospy.wait_for_service('startPump')
    start_pump = rospy.ServiceProxy('startPump', startPump)
    drop_item = rospy.ServiceProxy('dropItem', dropItem)
    check_item_attached = rospy.ServiceProxy('checkItemAttached', checkItemAttached)

    try:
        
        response = start_pump(300, 1000)
        print("status of vacuum is " + str(response.vacuumSuccess))

        if(response.vacuumSuccess == True):
            # move end effector up slightly

            # check item still attached
            response = check_item_attached()
            print("item is attached: " + str(response.itemAttached))

            # wait 3 seconds
            time.sleep(3)

            # drop item 
            response = drop_item(100)

            # check item dropped
            response = check_item_attached()
            print("item is attached: " + str(response.itemAttached))

        else: 
            print("vacuum not established")


    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)


if __name__ == "__main__":
    basic_demo()