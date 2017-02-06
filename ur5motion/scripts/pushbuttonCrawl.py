#!/usr/bin/env python
import time
import rospy
import roslib 
import numpy as np
import cv2
import actionlib

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from actionlib_msgs.msg import *
from math import *

import ur5Lib
ur5Lib.setParams()

# dummy = rospy.wait_for_message('/joint_states',JointState)

def main():

    ####### Initializations

    rospy.init_node('alpha', anonymous=True)

    sm = ur5Lib.ur5Class()

    print "Waiting for actionLib server..."
    sm.client.wait_for_server()
    print "Connected to server."
    sm.client.cancel_all_goals()

    rate = rospy.Rate(rospy.get_param('/ur5/fbRate'))

    #######

    dwellTime = 5.0
    velocity = 50.0

    if rospy.get_param('/ur5/onHusky'):
        dwellTime = 15.0
        velocity = 25.0
        velCmd = velocity

    time.sleep(1.0)
    print "Wake up..."
    sm.jointGoto(rospy.get_param('/ur5/poseWakeup'),dwellTime)
    sm.client.wait_for_result()

    print "Get ready..."
    sm.jointGoto(rospy.get_param('/ur5/poseReady'),dwellTime)
    sm.client.wait_for_result()

    # Initialize logical variables
    Crawling = True
    BackPlane = False

    while not rospy.is_shutdown():

        if Crawling: # CRAWL MODE
            Aligned = False
            x,y,z = sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
            print "Current: ", x, y, z
            print "Shifting: ", -50, 0, 0
            flag = sm.xyzShift(-50.0, 0.0, 0.0, velocity)
            Arrived = False
            while not Arrived:
                if sm.anyButton():
                    sm.cancel()
                    print "STOPPING!!!"
                    Arrived = True
                    Crawling = False
                elif sm.client.get_state() == 3: # SUCCEEDED
                    Arrived = True
                else:
                    xNow,yNow,zNow = sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
                    print "x/y/z: ", xNow, yNow, zNow
                rate.sleep()

        elif not Aligned: # ALIGN MODE
            Aligned = False
            while not Aligned:
                greenSide = (sm.buttons[0] or sm.buttons[1])
                blueSide = (sm.buttons[2] or sm.buttons[3])
                dq = 1.0/180.0*np.pi # one degree shift per second
                Q = sm.jointPosition
                print Q[4]
                if (greenSide and blueSide):
                    Aligned = True
                    Crawling = False
                    sm.q5offset = Q[4] - rospy.get_param('/ur5/poseReady')[4]
                    print "Aligned! dQ = ", sm.q5offset
                elif (greenSide or blueSide):
                    if greenSide: # Assumes green side on right when facing panel
                        q4 = Q[4] - dq       
                    else: # blueSide
                        q4 = Q[4] + dq
                    Qnew = Q[0], Q[1], Q[2], Q[3], q4, Q[5]
                    sm.jointGoto(Qnew,1.0)
                    sm.client.wait_for_result(rospy.Duration(1.0))
                else: # Neither side is pressed
                    Aligned = True # break from while loop
                    Crawling = True # go back to CRAWL mode
                    sm.q5offset = Q[4] - rospy.get_param('/ur5/poseReady')[4]
                    print "***", Q[4], sm.q5offset
                    flag = sm.xyzShift(20.0,0,0,velocity) # retract slightly
                    sm.client.wait_for_result()

        elif Aligned and not BackPlane:  #Aligned; Switch to search mode
            print "ARM: Moving back: [x,y,z] = [150,0,0]"
            flag = sm.xyzShift(150.0, 0.0, 0.0, velCmd)
            sm.client.wait_for_result()
            BackPlane = True

        
	elif BackPlane == True:
	    print(sm.pixel_x,sm.pixel_y,sm.pixel_z)
	    flag = sm.xyzShift(0.0,0.0,sm.pixel_z, velocity)  
            #flag = sm.xyzShift(sm.pixel_x, sm.pixel_y, sm.pixel_z, velocity)
            sm.client.wait_for_result()
	    time.sleep(5.0)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

