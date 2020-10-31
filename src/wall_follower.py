#!/usr/bin/env python
import numpy as np
import rospy
import math
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from artest import main
from ar_track_alvar_msgs.msg import AlvarMarkers

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT

    SCAN_TOPIC = "scan"
    DRIVE_TOPIC = "mobile_base/commands/velocity"
    SIDE = 1
    VELOCITY = 0.2
    DESIRED_DISTANCE = 0.5
    AR_TOPIC = "/ar_pose_marker"
    ARTAG = -1
    VALID_AR_TAGS = [0,1,2,5]

    def __init__(self):
        # Initialize your publishers and
        # subscribers
        self.data = None    
        self.angle = 0
        self.cmd = Twist()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, Twist, queue_size=1)
        self.ar_sub = rospy.Subscriber(self.AR_TOPIC, AlvarMarkers, self.arCallback)

    def scan(self, data):
    #stores the lidar data so you can work with it
        self.data = data
    #calls function that controls driving
        #self.drive()
    
    def drive(self):
    #controls driving
    #gets the angle required
        self.angle = self.find_wall()
    #sets speed and driving angle
        self.cmd.linear.x = self.VELOCITY
        self.cmd.angular.z = self.angle
        #publishes the command
        self.drive_pub.publish(self.cmd)
        print("*************** WALL FOLLOWER ************")

    #disclaimer: the variables and comments are backwards
    def find_wall(self):
    # if lidar data has not been received, do nothing
    #under ideal conditions, turnAngle is 0
        turnAngle = 0
    # follow on left side
        if self.SIDE == -1:
            dataside = len(self.data.ranges)*6/8-1
            datadiagonal = len(self.data.ranges)*11/16-1
        # follow on right side
        elif self.SIDE == 1:
            dataside = len(self.data.ranges)*2/8-1
            datadiagonal = len(self.data.ranges)*5/16-1
            
        left = self.data.ranges[dataside]
        e1 = left - self.DESIRED_DISTANCE
        forwardleft = self.data.ranges[datadiagonal]
        e2 = forwardleft - (self.DESIRED_DISTANCE / (math.cos((2*math.pi)/16)))

    #e2 goes against DESIRED_DISTANCE * sqrt(2) because of 45 45 90 triangle formula
    #in ideal conditions e1 and e2 should be zero (perfect distance, heading straight)
    
    #positive turnAngle should be towards the wall, negative should be away
        #print("I got the message! I'm now wall following!")
        #car is farther than desired distance, need to turn towards wall
        
        if self.SIDE == -1:
            print("I'M FOLLOWING ON THE LEFT ********************")
            if e1 > 0 and e2 > 0:
                turnAngle = 0.5
                print("toward")
            #car is pointed towards wall, need to turn away from wall
            elif e1 > 0 and e2 < 0:
                turnAngle = -0.3
                print("away")
            #car is pointed away from wall, needs to turn towards wall
            elif e1 < 0 and e2 > 0:
                turnAngle = 0.3
                print("toward")
            #car is too close to wall, need to turn away from wall
            elif e1 < 0 and e2 < 0:
                turnAngle = -0.5
                print("away")

        elif self.SIDE == 1:
            print("I'M FOLLOWING ON THE RIGHT ********************")
            if e1 > 0 and e2 > 0:
                turnAngle = -0.5
                print("toward")
            #car is pointed towards wall, need to turn away from wall
            elif e1 > 0 and e2 < 0:
                turnAngle = 0.3
                print("away")
            #car is pointed away from wall, needs to turn towards wall
            elif e1 < 0 and e2 > 0:
                turnAngle = -0.3
                print("toward")
            #car is too close to wall, need to turn away from wall
            elif e1 < 0 and e2 < 0:
                turnAngle = 0.5
                print("away")

        #if none of the if or elif statements are run, turnAngle stays 0, ideal situation
        print(turnAngle, forwardleft, left)
        return turnAngle
        #turnAngle is the angle which the car will turn by

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        #TODO: Write your state changes here
        print("ARTAG NUMBER: " + str(self.ARTAG))
        if len(tags.markers) == 1:
            for i in self.VALID_AR_TAGS: 
                if i == tags.markers[0].id:
                    self.ARTAG = tags.markers[0].id
                    print("VALID AR TAG DETECTED! " + str(self.ARTAG))
        if self.ARTAG == 0:
            self.SIDE = 1
            self.drive()
        elif self.ARTAG == 1:
            self.SIDE = -1
            self.drive()
        
        pass

"""Lidar data is now stored in self.data, which can be accessed
using self.data.ranges (in simulation, returns an array).
Lidar data at an index is the distance to the nearest detected object
self.data.ranges[0] gives the leftmost lidar point
self.data.ranges[100] gives the rightmost lidar point
self.data.ranges[50] gives the forward lidar point
"""

#returns the output of your alg, the new angle to drive in

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
rospy.spin()
