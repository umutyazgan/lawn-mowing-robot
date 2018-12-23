#!/usr/bin/env python


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
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


class GoToPose():
    def __init__(self):

        self.goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        self.move_base.wait_for_server(rospy.Duration(3))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(20))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


goal_poses = [{'x': -2.95, 'y': -0.45},{'x': -4.4, 'y': -1.1},{'x': -5.25, 'y': 0.5},{'x': -6.7, 'y': 5.3},{'x': -9.25, 'y': 4.35},{'x': -11.15, 'y': 3.75},
              {'x': -13.05, 'y': 3.1},{'x': -14.35, 'y': 2.05},{'x': -6.75, 'y': 4.65},{'x': -6.65, 'y': 4.25},{'x': -14.0, 'y': 1.55},{'x': -14.0, 'y': 1.15},
              {'x': -6.3, 'y': 3.9},{'x': -9.95, 'y': 2.5},{'x': -8.65, 'y': 2.25},{'x': -9.7, 'y': 1.14},{'x': -8.4, 'y': 1.3},{'x': -9.3, 'y': 0.35},
              {'x': -5.65, 'y': 1.65},{'x': -12.95, 'y': -1.65},{'x': -12.75, 'y': -2.15},{'x': -5.25, 'y': 0.85},{'x': -5.85, 'y':-0.15},{'x': -9.0, 'y': -1.55},
              {'x': -8.6, 'y': -2.2},{'x': -10.85, 'y': -2.35},{'x': -12.45, 'y':-3.05},{'x': -12.25, 'y':-3.95},{'x': -4.45, 'y': -0.75},{'x': -4.05, 'y': -1.65},
              {'x': -12.25, 'y':-5.15},{'x': -12.0, 'y': -6.0},{'x': -2.35, 'y': -2.2},{'x': -2.0, 'y': -3.05},{'x': -11.2, 'y':-6.85},{'x': -7.2, 'y':-5.4},
              {'x': -5.8, 'y': -5.15},{'x': -1.75, 'y':-3.45},{'x': -4.9, 'y':-5.35},{'x': -7.65, 'y':-6.35},{'x': -7.4, 'y': -7.0},{'x': -4.55, 'y': -6.1},
              {'x': -4.5, 'y':-7.0},{'x': -7.05, 'y':-8.1},{'x': -10.35, 'y':-7.6},{'x': -10.85, 'y':-7.6},{'x': -10.01, 'y':-10.05},{'x': -0.8, 'y':-6.45},
              {'x': -0.4, 'y':-7.3},{'x': -10.25, 'y':-10.7},{'x': -10.0, 'y':-11.4},{'x': -5.55, 'y':-10.05},{'x': -4.05, 'y':-9.3},{'x': -0.15, 'y':-7.9},{'x': -0.12, 'y':-8.8},
              {'x': -0.65, 'y':-10.55},{'x': -2.95, 'y':-10.2},{'x': -6.05, 'y':-11.3},{'x': -5.85, 'y':-11.95},{'x': -2.95, 'y':-10.9},{'x': -2.65, 'y':-10.8},{'x': -2.3, 'y':-13.0},
              {'x': -8.25, 'y':-13.9},{'x': -9.2, 'y':-12.2},{'x': -8.45, 'y':-14.8},{'x': -0.85, 'y':-11.55},{'x': -3.0, 'y':-0.5}]

if __name__ == '__main__':
    for goal in goal_poses:
        try:
            rospy.init_node('nav_test', anonymous=False)
            navigator = GoToPose()

            # Customize the following values so they are appropriate for your location
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

            rospy.loginfo("Go to (%s, %s) pose", goal['x'], goal['y'])
            success = navigator.goto(goal, quaternion)

            if success:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)

        except rospy.ROSInterruptException:
            rospy.loginfo("Ctrl-C caught. Quitting")
