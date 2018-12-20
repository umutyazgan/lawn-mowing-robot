#!/usr/bin/env python
## AK
## explorer_node_py.py
##
## BLG456E Assignment 1 skeleton
##
## Instructions: Change the laser_callback function to make the robot explore more
## intelligently, using its sensory data (the laser range array).
##
## Advanced: If you want to make use of the robot's mapping subsystem then you can
## make use of the map in the mapping_callback function.
##
## 

## Common ROS headers.
import rospy
## Required for some printing options
import sys
## Required for trigonametry and sqrt functions
import math

## This is needed for the data structure containing the motor command.
from geometry_msgs.msg import Twist
## This is needed for the data structure containing the laser scan
from sensor_msgs.msg import LaserScan
## This is needed for the data structure containing the map (which you may not use).
from nav_msgs.msg import OccupancyGrid

## The following function is a "callback" function that is called back whenever a new laser scan is available.
## That is, this function will be called for every new laser scan.
##
## --------------------------------------------------------------------------------
## ----------CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY----------
## --------------------------------------------------------------------------------
##
def laser_callback(data):
    ## Lets fill a twist message for motor command
    motor_command = Twist()
    mid_laser = len(data.ranges)/2

    ## A non-trigonametric approach. This works to some extent.
    def blocked():
        """
        Checks if the robot is blocked. If it is, then finds out
        from which direction it is blocked.
        """
        for i, r in enumerate(data.ranges):
            if r < 0.7:
                if i > mid_laser:
                    return 1  # blocked from left
                else:
                    return 0  # blocked from right
        return -1  # not blocked

    if blocked() == 1:  # if blocked from left
        motor_command.linear.x = 0     # stop and
        motor_command.angular.z= -0.3  # turn right
        motor_command_publisher.publish(motor_command)
    elif blocked() == 0:  # if blocked from right
        motor_command.linear.x = 0    # stop and
        motor_command.angular.z= 0.3  # turn left
        motor_command_publisher.publish(motor_command)
    else:  # if not blocked
        motor_command.linear.x = 0.3  # keep going
        motor_command.angular.z= 0    # straight
        motor_command_publisher.publish(motor_command)

    ## Tried a trigonametric approach. It didn't work.
    ## I used laws of Sines and Cosines on triangles to calculate how
    ## much robot has to turn to be parallel to wall. However, I could
    ## not make the robot turn the amount I calculated.
    # if in_range is True:
    #     motor_command.linear.x = 0
    #     a = data.ranges[len(data.ranges)/2]
    #     if wall_on_left is True:
    #         b = data.ranges[len(data.ranges)/2 + 10]
    #     else:
    #         b = data.ranges[len(data.ranges)/2 - 10]
    #     C = data.angle_increment*10
    #     c = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(C))
    #     A = math.asin(a*math.sin(C)/c)
    #     if a > b:
    #         motor_command.angular.z = -A  # this is an angle so using it for
                                            # angular velocity is a bad idea.
    #     else:
    #         motor_command.angular.z = A
    # else:
    #     motor_command.linear.x= 0.3
    # ## Lets publish that command so that the robot follows it
    # global motor_command_publisher
    # motor_command_publisher.publish(motor_command)
    
    ## Alternatively we could have looked at the laser scan BEFORE we made this decision
    ## Well Lets see how we might use a laser scan
    ## Laser scan is an array of distances
    print 'Number of points in laser scan is: ', len(data.ranges)
    print 'The distance to the rightmost scanned point is: ', data.ranges[0]
    print 'The distance to the leftmost scanned point is: ', data.ranges[-1]
    print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
    ## You can use basic trigonometry with the above scan array and the following information to find out exactly where the laser scan found something
    print 'The minimum angle scanned by the laser is: ', data.angle_min
    print 'The maximum angle scanned by the laser is: ', data.angle_max
    print 'The increment in the angles scanned by the laser is: ', data.angle_increment
    ## angle_max = angle_min+angle_increment*len(data.ranges)
    print 'The minimum range (distance) the laser can perceive is: ', data.range_min
    print 'The maximum range (distance) the laser can perceive is: ', data.range_max
    
## You can also make use of the map which is being built by the "gslam_mapping" subsystem
## There is some code here to help but you can understand the API also by looking up the OccupancyGrid message and its members (this is the API for the message)
## If you want me to explain the data structure, I will - just ask me in advance of class
def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        ## Here x and y has been incremented with five to make it fit in the terminal
        ## Note that we have lost some map information by shrinking the data
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    ## This square is occupied
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    ## This square is unoccupied
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"
    
## This is the method we initilize everything
def explorer_node():
    ## We must always do this when starting a ROS node - and it should be the first thing to happen
    rospy.init_node('amble')
    
    ## Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi. It is defined as global because we are going to use this publisher in the laser_callback.
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    
    ## Here we set the function laser_callback to recieve new laser messages when they arrive
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)
    
    ## Here we set the function map_callback to recieve new map messages when they arrive from the mapping subsystem
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size = 1000)
    
    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    explorer_node()
