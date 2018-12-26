#!/usr/bin/env python

# *** BELOW CODE TAKEN FROM https://github.com/markwsilliman/turtlebot/blob/master/go_to_specific_point_on_map.py
# *** AND MODIFIED BY MEMBERS OF BeeRob

# *** To sum up, we used this piece of code to understand how to move our robot to a single specific location, and
# *** modified it to follow a route we determined. We also made some other slight modifications for our needs and
# *** inserted clarifying comments to explain how everything works. We could write it from scratch after understanding
# *** how it works, but we would be essentially writing the same script with different variable names and log messages.

"""
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32MultiArray


class GoToPose:
    def __init__(self):
        # goal not sent when object is initialized
        self.goal_sent = False
        # execute shutdown() method of this class in case of a shutdown
        rospy.on_shutdown(self.shutdown)
        # initialize a member object, move_base, as a SimpleActionClient object which uses "move_base" as name and
        # MoveBaseAction as handle
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # allow action server for 3 seconds to come up and inform user about it using loginfo method of rospy
        rospy.loginfo("Wait for the action server to come up")
        # this was originally 5 seconds, we modified it in an attempt to reduce waiting times
        self.move_base.wait_for_server(rospy.Duration(3))

    def goto(self, pos):
        # a goal is sent now
        self.goal_sent = True
        # goal is a MoveBaseGoal() object, which in fact is a Python object for move_base_msgs/MoveBaseGoal.msg ROS
        # message: http://docs.ros.org/groovy/api/move_base_msgs/html/msg/MoveBaseGoal.html
        goal = MoveBaseGoal()
        # set up goal messages header with frame id and timestamp
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        # Pose() is a Python object for geometry_msgs/Pose ROS message which describes coordinates and orientation of a
        # robot: http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
        # set pose to send in message with Point and Quaternion given in parameters, z coordinate is 0 by default
        # since turtlebot cannot operate in z direction
        # we don't really care about orientation so we can give a default Quaternion for all goals
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(0.000, 0.000, 0.000, 1.000))
        # send goal message to move_base
        self.move_base.send_goal(goal)
        # Allow TurtleBot up to 20 seconds to complete task, store result of task. We reduced this value from 60 to 20
        # in order to reduce waiting times
        success = self.move_base.wait_for_result(rospy.Duration(20))
        # store robots state after task is completed (successfully or not)
        state = self.move_base.get_state()
        result = False
        # if both robots state after task and tasks success is OK (uses GoalStatus.SUCCEEDED instead of just True to be
        # more portable). Meanings of GoalStatus.SUCCEEDED values can be inspected here:
        # http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html Any result other than 0 is OK here
        if success and state == GoalStatus.SUCCEEDED:
            # Result is success
            result = True
        else:
            # Result is failure. Cancel the goal
            self.move_base.cancel_goal()
        # set goal_sent back to its default value
        self.goal_sent = False
        return result

    def shutdown(self):
        # if sending goal during shutdown
        if self.goal_sent:
            # cancel goal
            self.move_base.cancel_goal()
        # give information to user
        rospy.loginfo("Stop")
        # give a second for shutdown process to complete
        rospy.sleep(1)


# *** BELOW CODE ADDED BY BeeRob MEMBERS.

received_poses = []

def incoming_coordinate(data):
    pose = {'x': data.data[0], 'y': data.data[1]}
    received_poses.append(pose)

if __name__ == '__main__':
    try:
        # initialize our ROS node, lawn_mower
        rospy.init_node('lawn_mower', anonymous=False)
        rospy.Subscriber("coordinates", Float32MultiArray, incoming_coordinate)
        # initialize navigator object we will use to reach our goals
        navigator = GoToPose()
        # for each goal we need to go
        while True:
            if len(received_poses) > 0:
                goal = received_poses[-1]
                if goal['x'] == '-99':
                    break
                # give user info about next goal
                rospy.loginfo("Go to (%s, %s) pose", goal['x'], goal['y'])
                # call goto() method of navigator with current goal and store its result
                success = navigator.goto(goal)
                if success:
                    rospy.loginfo("Hooray, reached the desired pose")
                else:
                    rospy.loginfo("The base failed to reach the desired pose")
                # Sleep for 1 second to give the last log messages time to be sent
                received_poses.pop()
                rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
