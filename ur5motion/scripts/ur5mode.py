#!/usr/bin/env python
import time
import rospy
import roslib 
import numpy as np
import cv2
import actionlib
import math

from std_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from actionlib_msgs.msg import *
from math import *

import ur5libmode
ur5libmode.setParams()

# dummy = rospy.wait_for_message('/joint_states',JointState)

ur_mode_pub = rospy.Publisher('/mode_u_main', String , queue_size=10)
ur_task_pub = rospy.Publisher('/task_u_main',Int32 , queue_size=10)

def main():

    ####### Initializations

    rospy.init_node('alpha', anonymous=True)

    sm = ur5libmode.ur5Class()

    print "Waiting for actionLib server..."
    sm.client.wait_for_server()
    print "Connected to server."
    sm.client.cancel_all_goals()

    rate = rospy.Rate(rospy.get_param('/ur5/fbRate'))

    #######


    if rospy.get_param('/ur5/onHusky'):
	dwellTime   	       = 15.0
	velocity  	       = 15.0
	velCmd      	       = 15.0
	degShift   	       = 0.25
	xBackPlane             = 250.0
	yBackPlane             = -150.0

	xValveToTools	       = 0.0
	yValveToTools	       = 450.0
	zValveToTools	       = 50.0
	nWayPoints	       = 4
	
	countSearchValve	=0

	xToolsToValve          = 0.0
	yToolsToValve          = -350.0


	xTOvalve	      = -20.0
	yTOvalve	      = 0.0
	zTOvalve	      = 0.0
	
	
	GripperLen	      = 180.0
	Margin		      = 70.0
	PinNumber             = 0
	ToolNumber            = 0


	time.sleep(1.0)
	
	state_topic           ="Idle"
	task_topic           = 0    

    # Initialize logical variables
    Crawling 	     	= True
    ReadyLoacl 	        = True
    MoveToValve         = True
    BackPlane	        = True
    MoveTOTools         = True
    Approching          = True
    Intitial		= True
    Scaning             = 1
    GoGripping 		= True

    while not rospy.is_shutdown():
###################################################
#####  Mode: Get ready, aligned and backplane ###
###################################################
	if sm.ur_mode=="Idle":
	   time.sleep(0.1)
	   print "I am in sleeping mode, ur_mode is Idle"
	   

	elif ReadyLoacl and sm.ur_mode=="Ready":
		if Intitial:
	       		print "sm.ur_mode=Ready, Get ready..."
			state_topic = "Ready" 
			sm.jointGoto(rospy.get_param('/ur5/poseReady'),dwellTime)
			time.sleep(dwellTime)

			sm.client.wait_for_result()

			flag = sm.xyzShift(-180, 0.0, 0.0, 0.5*velCmd)
			sm.client.wait_for_result()

			Intitial=False
		if Crawling: # CRAWL MODE
		    Aligned = False
		    x,y,z = sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		    print "Current: ", x, y, z
		    Arrived = False
		    while not Arrived:
		        flag = sm.xyzShift(-2, 0.0, 0.0, velCmd)
		        time.sleep(0.2)
		        sm.client.wait_for_result()
		        if sm.anyButton():
		            print "STOPPING!!!"
		            Arrived = True
		            Crawling = False
		        else:
		            xNow,yNow,zNow = sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		            print "x/y/z: ", xNow, yNow, zNow
		elif not Aligned: # ALIGN MODE
		    Aligned = False
		    while not Aligned:
		        blueSide = (sm.buttons[0] or sm.buttons[1])
		        greenSide = (sm.buttons[2] or sm.buttons[3])
		        dq = degShift/180.0*np.pi # degShift  degree shift per second
		        Q = sm.jointPosition
		        print Q[4]
		        if (greenSide and blueSide):
		            Aligned = True
		            Crawling = False
		            sm.q5offset = Q[4] - rospy.get_param('/ur5/poseReady')[4]
		            print "Aligned! dQ = ", sm.q5offset
                           
			    xTouch,yTouch,zTouch = sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
			
		        elif (greenSide or blueSide):
			    print "One side pushbutton pressed"
		            flag = sm.xyzShift(15.0, 0.0, 0.0, velCmd) # If key pressed, come back
		            sm.client.wait_for_result()

		            if greenSide: # Assumes green side on right when facing panel
		                sm.q5offset = sm.q5offset + dq
		            else: # blueSide
		                sm.q5offset = sm.q5offset - dq

			    Crawling = True # Move forward again
		            Aligned = True
		        else:
		            print "No buttons pressed?"
		            flag = sm.xyzShift(10.0, 0.0, 0.0, velCmd)
		            sm.client.wait_for_result()
		            
		elif Aligned and  BackPlane and sm.ur_mode=="Ready":  #Aligned; Switch to search mode
		    print "ARM: Moving back: [x,y,z] = [200,0,0]"
		    flag = sm.xyzShift(200.0,0.0, 0.0, velCmd)
		    
		    sm.client.wait_for_result()
		    print "I am in BackPlane"
		    flag = sm.xyzShift(0.0, 0.0, 150.0, velCmd)
		    sm.client.wait_for_result()
    		    state_topic = "urAligned"
    		    task_topic = 1
		    ReadyLoacl=False









###################################################
##########  Mode : search for Valve       ##########
###################################################
	           

	elif  sm.ur_mode=="searchValve":
		print "I am in searchVave"
		xCur,yCur,zCur= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		xCmd =xCur #xTouch+100+Margin 
		print xCmd
		zCmd = 975.0-360.0-70.0
		if ( countSearchValve == 0):
			yCmd = yCur-10.0
				
		else:
			yCmd = yCur+10.0
			
			
			
	
		flag = sm.xyzGoto(xCmd,yCmd , zCmd, 0.5*velCmd)
		sm.client.wait_for_result()
		xCur,yCur,zCur= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])

		if countSearchValve == 0 and abs(yCur-yTouch)>=200:
			countSearchValve =1
		elif countSearchValve == 1 and abs(yCur-yTouch)>200:
			countSearchValve =0
			
		state_topic = "searchValve"
		task_topic = 1
		print " countSearchValve  ",countSearchValve
###################################################
##########  Mode : Aligne to Valve       ##########
###################################################
	           
	elif  sm.ur_mode=="valveAlign":
		#TODO search
		flag = sm.xyzShift(0.2*sm.pixel_x, 0.2*sm.pixel_y, 0.2*sm.pixel_z, 0.1*velCmd)
		sm.client.wait_for_result()
		#xValve,yValve,zValve= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		#print "xValve/yValve/zValve: ", xValve,yValve,zValve
		state_topic = "valveAlign"
		task_topic = 1

###################################################
############## Mode : Valve Sizing            ####
###################################################


	elif sm.ur_mode=="valveSizing":
		#flag = sm.xyzShiftWayPoints(-100.0, 0.0,-80.0,nWayPoints, 0.5*velCmd)
		#flag = sm.xyzGoto(xValve-170.0,yValve,zValve-140.0, velocity)
		#sm.client.wait_for_result()
		#TODO something form the gripper/vision to detect the valve size
        	state_topic = "sizingDone"
        	task_topic = 1
		#Approching=False




###################################################
############## Mode :           pinching      ####
###################################################


	#elif  sm.ur_mode==10:
		#Q = sm.jointPosition
		#Q[5]=Q[5]+15*np.pi/180
		#Qtarget=[Q[0],Q[1],Q[2],Q[3],Q[4],Q[5]]
		#sm.jointGoto(Qtarget,5.0)
		#sm.client.wait_for_result()
		#state_topic = 11






###################################################
##############  Mode :  BackBlane    ###########
###################################################  
	#elif BackPlane and  sm.ur_mode==12:
		#flag = sm.xyzShift(xBackBlane,0,0, velocity)
		#sm.client.wait_for_result()
	    	#state_topic = 13
	    	#BackPlane = False
	    	
###################################################
##############  Mode :  Back to tools   ###########
###################################################  	    	
	    	
	    	
	elif MoveTOTools and  sm.ur_mode=="goTools":
		#flag = sm.xyzShift(xBackPlane,0,0, velocity)
		#sm.client.wait_for_result()
		
		xValve,yValve,zValve= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
                print "xValve/yValve/zValve: ", xValve,yValve,zValve
		time.sleep(2.0)
		flag = sm.xyzShiftWayPoints(0,450,10,nWayPoints, velocity)
		sm.client.wait_for_result()
		state_topic = "atTools"
		task_topic = 1
		MoveTOTools = False
		
		
		
		
		
		
		
		
###################################################
##############  Mode :  Tools detection   ###########
###################################################  	    	
	    	
	    	
	elif   sm.ur_mode=="scanTools":
	
		flag = sm.xyzShift(0.0,(Scaning)*70,0.0,0.5*velocity)
		sm.client.wait_for_result()
		Scaning=-Scaning
		state_topic = "atTools"
		task_topic = 1

	  		
		



		
###################################################
##############  Mode: Tool picking   ###########
###################################################  	    	
	    	
	    	
	elif   sm.ur_mode=="goCorrectTool":
		xCmd = xTouch+100+Margin
		yCmd = yValve+348.0+50.0*(sm.PinNumber-1)
		zCmd = zValve + (225-(185+10.0*(sm.ToolSize- 16)))

		flag = sm.xyzGoto(xCmd,yCmd,zCmd, velocity)
		sm.client.wait_for_result()
		state_topic = "atCorrectTool"
		task_topic = 1





	  
###################################################
########  Mode 11  Align with the correct tool   #####
###################################################  
	elif  sm.ur_mode=="alignCorrectTool":
		flag = sm.xyzShift(0*sm.pixel_x, 0*sm.pixel_y, 0*sm.pixel_z, 0.5*velCmd)
		xGrip,yGrip,zGrip= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		sm.client.wait_for_result()
		state_topic = "alignCorrectTool"
		task_topic = 1
		
		
		
		

	  
###################################################
########  Mode:   Grip the correct  tool   #####
###################################################  
	elif  GoGripping  and sm.ur_mode=="goGripping":

		xcmdg=xTouch+30-xGrip
		print "I am movint to the tool by ", xcmdg
		flag = sm.xyzShiftWayPoints(xcmdg, 0, 0,4, 0.3*velCmd)

		sm.client.wait_for_result()
		GoGripping = False 
		state_topic = "atGrip"
		task_topic = 1		
		

		
		
###################################################
########## Mode: Move back to the valve #####
###################################################  
	elif  BackPlane and sm.ur_mode=="moveValve":
		flag = sm.xyzShiftWayPoints(150.0,0.0, 0.0,4, velCmd)
		sm.client.wait_for_result()


                flag = sm.xyzShiftWayPoints(0.0,-300, 0.0,4, velCmd)
                sm.client.wait_for_result()

		flag = sm.xyzGoto(xValve,yValve,zValve+50, velocity)
		sm.client.wait_for_result()

		flag = sm.xyzShiftWayPoints(xTouch-xValve+70.0,0,0,4, velocity)
                sm.client.wait_for_result()

		state_topic = "atValve"
		task_topic = 1
		BackPlane = False
							
	
	
	
	
	
	
	
	
###################################################
#######          Mode: insert tool   ####
###################################################
	elif  sm.ur_mode=="insertTool":
		#Q = sm.jointPosition
		#Q[5]=Q[5]+15*np.pi/180
		#Qtarget=[Q[0],Q[1],Q[2],Q[3],Q[4],Q[5]]
		#sm.jointGoto(self,Qtarget,5.0)
		#sm.client.wait_for_result()
		state_topic = "toolInserted"
		task_topic = 1
		
		
	
###################################################
#######          Mode: Rotation    ####
###################################################
	elif  sm.ur_mode=="rotateValve":
		#Q = sm.jointPosition
		#Q[5]=Q[5]+15*np.pi/180
		#Qtarget=[Q[0],Q[1],Q[2],Q[3],Q[4],Q[5]]
		#sm.jointGoto(self,Qtarget,5.0)
		#sm.client.wait_for_result()
		state_topic = "valveRotated"
		task_topic = 1





	

	ur_mode_pub.publish(state_topic)
	ur_task_pub.publish(task_topic)








		


if __name__ == '__main__':
    try:
        main()
	
    except rospy.ROSInterruptException:
        pass
