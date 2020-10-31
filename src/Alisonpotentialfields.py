#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class PotentialField:
    '''
    SCAN_TOPIC = rospy.get_param("scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    '''
    
    SCAN_TOPIC = "scan"
    DRIVE_TOPIC = "mobile_base/commands/velocity"
    AR_TOPIC = "/ar_pose_marker"
    ARTAG = -1
    VALID_AR_TAGS = [0,1,2,5]
    
    def __init__(self):
        self.data = None
        self.cmd = Twist()

        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, Twist, queue_size=1)
        self.ar_sub = rospy.Subscriber(self.AR_TOPIC, AlvarMarkers, self.arCallback)
        #cartesian points -- to be filled (tuples)
        self.cartPoints = [None for x in range(314-44)] #changed from 100

        #[speed, angle]
        self.finalVector = [1.0, 0]

    def scan_callback(self, data):
        '''Checks LIDAR data'''
        #rospy.loginfo('scan_callback called!' + str(len(data.ranges)))
        self.data = data.ranges[44:314]
        print(len(self.data))
	#rospy.loginfo(self.data)

    def drive_callback(self, FinalAngle, FinalVelocity):
        '''Publishes drive commands'''
        #make sure to publish cmd here
        self.cmd.linear.x = FinalVelocity
        self.cmd.angular.z = FinalAngle
        self.drive_pub.publish(self.cmd)

    def convertPoints(self, points):
        '''Convert all current LIDAR data to cartesian coordinates'''
        sinarray = np.arange(len(self.data)) #creates two arrays from 0 to 360 (i.e. the length of the data)
        cosarray = np.arange(len(self.data))

        '''2.7 is 270/100, or the size of each sector in degrees. Apparently the sin and cos functions take radian values,
        so all angle values are converted to radians

        ***EDIT: Now, because the LIDAR takes 360 degrees and there are 360 divisions, each sector is now just 1.0 degree, or 360/360***

        135 because you want the angle value you take the sin of to be 45 degrees less than what it 
        actually is e.g. the rightmost value is 0 degrees in the data, but actually should be -45 degrees. 
        Then you want the vector to be pointing toward the car instead of away so add 180 degrees to this new 
        angle value will reverse the direction of the vector

        *** EDIT: This section is no longer relevant because we have the full 360 degrees**'''
                
        sinarray = np.sin((sinarray * math.pi * 1.0/180) + (math.pi * 135/180))
        cosarray = np.cos((cosarray * math.pi * 1.0/180) + (math.pi * 135/180))
        #print(cosarray)
        #print(self.data)
        dataarray = np.arange(len(self.data))
        dataarray = np.reciprocal(self.data)
        #print(dataarray)

        xcoordinatearray = np.arange(len(self.data))
        ycoordinatearray = np.arange(len(self.data))
        xcoordinatearray = dataarray * cosarray
        ycoordinatearray = dataarray * sinarray
        xcoordinatevalue = sum(xcoordinatearray)/len(xcoordinatearray)
        ycoordinatevalue = sum(ycoordinatearray)/len(ycoordinatearray)
        vectorvalue = math.sqrt(xcoordinatevalue ** 2 + ycoordinatevalue ** 2)
        anglevalue = math.atan(ycoordinatevalue/xcoordinatevalue)
        self.calcFinalVector(vectorvalue, anglevalue)

    def calcFinalVector(self, vectorvalue, anglevalue): #initially was points changed to vectorvalue and anglevalue? Not sure what I'm doing
        '''Calculate the final driving speed and angle'''
        FinalVelocity = 0.9 * vectorvalue
        FinalAngle = 0.5 * anglevalue
        print("Angle" + str(FinalAngle))
        print("Velocity" + str(FinalVelocity))
        self.drive_callback(FinalAngle, FinalVelocity)

    def arCallback(self, tags):
        '''Callback when an AR tag is detected'''
        #TODO: Write your state changes here
        print("ARTAG NUMBER: " + str(self.ARTAG))
        if len(tags.markers) == 1:
            print("THIS IS THE ID: " + str(tags.markers[0].id))
            for i in self.VALID_AR_TAGS: 
                if i == tags.markers[0].id:
                    self.ARTAG = tags.markers[0].id
                    print("VALID AR TAG DETECTED! " + str(self.ARTAG))
            self.ARTAG = tags.markers[0].id
        if self.ARTAG == 5:
            self.convertPoints(self.data)

if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.loginfo('alison')
    rospy.spin()

