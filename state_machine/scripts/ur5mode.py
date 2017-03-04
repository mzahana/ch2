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
    time.sleep(1.0)
    rate = rospy.Rate(rospy.get_param('/ur5/fbRate'))

    #######


    if rospy.get_param('/ur5/onHusky'):
	dwellTime   	       = 15.0
	velocity  	       = 17.0
	velCmd      	       = 17.0
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
	sm.ToolSize              = 19


	time.sleep(1.0)
	
	state_topic         ="Idle"
	task_topic          = 0    
	q6offset            = (15+10)*np.pi/180
        ToolFace            ="right"
	rotateInit          = True 	

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
    Engaged             = False
    EngagedTouch        = False 
    TouchAligen         = True
    GoEngage            = True 
    valveStates         = "Horizental"
    rotateMode          = " rotateOnly"
    insertToolLocal     = True

    confairmEngage      = True




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

			flag = sm.xyzShift(-280, 0.0, -100.0,1.3* velCmd)
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
		    flag = sm.xyzShift(230.0,0.0, 0.0, velCmd)
		    
		    sm.client.wait_for_result()
		    print "I am in BackPlane"
		    flag = sm.xyzShift(0.0, 0.0, 250.0, velCmd)
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
			yCmd = yCur-22.0
				
		else:
			yCmd = yCur+22.0
			
			
			
	
		flag = sm.xyzGoto(xCmd,yCmd , zCmd, 0.7*velCmd)
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
		flag = sm.xyzShift(0.4*sm.pixel_x, 0.4*sm.pixel_y, 0.4*sm.pixel_z, 0.3*velCmd)
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
		time.sleep(2.0)		
		xValve,yValve,zValve= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		yVave=yValve-5.0

                q1Valve,q2Valve,q3Valve,q4Valve,q5Valve,q6Valve = sm.jointPosition

                print "xValve/yValve/zValve: ", xValve,yValve,zValve
		time.sleep(2.0)
		flag = sm.xyzShiftWayPoints(0,500.0,50,nWayPoints, velocity)
		sm.client.wait_for_result()
		state_topic = "atTools"
		task_topic = 1
		MoveTOTools = False
		
		
		
		
		
		
		
		
###################################################
##############  Mode :  Tools detection   ###########
###################################################  	    	
	    	
	    	
	elif   sm.ur_mode=="scanTools":
	
		#flag = sm.xyzShiftWayPoints(0.0,050,0.0,3,0.3*velocity)
		flag = sm.xyzShiftWayPoints(0.0,(Scaning)*20,0.0,2,0.3*velocity)
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
		zCmd = zValve + (225-(190+8.5*(sm.ToolSize- 16)))

		flag = sm.xyzGoto(xCmd,yCmd,zCmd, velocity)
		sm.client.wait_for_result()
		state_topic = "atCorrectTool"
		task_topic = 1





	  
###################################################
########  Mode 11  Align with the correct tool   #####
###################################################  
	elif  sm.ur_mode=="alignCorrectTool":
		flag = sm.xyzShift(0.5*sm.pixel_x, 0.5*sm.pixel_y, 0.5*sm.pixel_z, 0.2*velCmd)
		xGrip,yGrip,zGrip= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		sm.client.wait_for_result()
		state_topic = "alignCorrectTool"
		task_topic = 1
		
		
		
		

	  
###################################################
########  Mode:   Grip the correct  tool   #####
###################################################  
	elif  GoGripping  and sm.ur_mode=="goGripping":

		xcmdg=xTouch+45-xGrip
		print "I am movint to the tool by ", xcmdg
		flag = sm.xyzShiftWayPoints(xcmdg, 0.0, 0.0,4, 0.3*velCmd)

		sm.client.wait_for_result()
		GoGripping = False 
		state_topic = "atGrip"
		task_topic = 1		
		

		
		
###################################################
########## Mode: Move back to the valve #####
###################################################  
	elif  BackPlane and sm.ur_mode=="moveValve":


		flag = sm.xyzShift(0.0,0.0, 4.0, velCmd)
                sm.client.wait_for_result()

		flag = sm.xyzShiftWayPoints(150.0,0.0, 0.0,4, velCmd)
		sm.client.wait_for_result()


                flag = sm.xyzShiftWayPoints(0.0,-400, 0.0,4, velCmd)
                sm.client.wait_for_result()

		#flag = sm.xyzGoto(xValve,yValve,zValve+50, velocity)
		#sm.client.wait_for_result()


 		QValve=[q1Valve,q2Valve,q3Valve,q4Valve,q5Valve,q6Valve]
                sm.jointGoto(QValve,6.0)
                sm.client.wait_for_result()


		upValve=50.0
                flag = sm.xyzShift(0.0,0.0, upValve, velCmd)
                sm.client.wait_for_result()



		flag = sm.xyzShiftWayPoints(xTouch-xValve+70.0,0.0,0.0,4, velocity)
                sm.client.wait_for_result()



		print "xValve/yValve/zValve: ", xValve,yValve,zValve
                xNow,yNow,zNow= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
                print "xNow,yNow,zNow", xNow-(xTouch-xValve+70),yNow,zNow-50



                state_topic = "atValve"
                task_topic = 1
                BackPlane = False
		insertToolLocal= True



###################################################
#######          Mode: insert tool   ####
###################################################


        elif   sm.ur_mode=="insertTool" and insertToolLocal:

##################################################

		if valveStates=="Horizental":
			   thetaValve = 0.0
			   print "do nothing"
		

		if valveStates=="Digonal":
			thetaValve = 45.0
	                q1,q2,q3,q4,q5,q6 = sm.jointPosition	     
               	        q6=q6+thetaValve*np.pi/180
                       	Qtarget=[q1,q2,q3,q4,q5,q6]
                       	sm.jointGoto(Qtarget,10.0)
                       	sm.client.wait_for_result()

		upValve = 0 

		if TouchAligen:
                        q1,q2,q3,q4,q5,q6 = sm.jointPosition
		        flag = sm.xyzShiftRotate(0.0,-30*sin(thetaValve*np.pi/180+90*np.pi/180), -30*cos(thetaValve*np.pi/180+90*np.pi/180),q6, velCmd)
#                        flag = sm.xyzShift(0.0,-60, 0.0, velCmd)

	                sm.client.wait_for_result()
			time.sleep(2)

			print " I am off the valve"
                        flag = sm.xyzShiftRotate(0.0,-(upValve+15)*sin(thetaValve*np.pi/180),- (upValve+15)*cos(thetaValve*np.pi/180),q6, velCmd)

#			flag = sm.xyzShift(0.0,0.0, -upValve-15, velCmd)
                        sm.client.wait_for_result()


	                ValveAligned =False
                        print "I an not alligned to the valve, I Touch to aligen"
                        yTorqueref=sm.yTorque
                        while not ValveAligned:
#                                 flag = sm.xyzShift(0.0,2.0, 0.0,velCmd)

	                         flag = sm.xyzShiftRotate(0.0,2*sin(thetaValve*np.pi/180+90*np.pi/180), 2*cos(thetaValve*np.pi/180+90*np.pi/180),q6, velCmd)
                                 sm.client.wait_for_result()
                                 if abs(sm.yTorque-yTorqueref)> 0.2:

#                                        flag = sm.xyzShift(0.0,-30.0, 0.0,velCmd)

                                        flag = sm.xyzShiftRotate(0.0,-30*sin(thetaValve*np.pi/180+90*np.pi/180), -30*cos(thetaValve*np.pi/180+90*np.pi/180),q6, velCmd)
                                        sm.client.wait_for_result()

 #                                       flag = sm.xyzShift(0.0,0.0,50,velCmd)
                                        flag = sm.xyzShiftRotate(0.0,50*sin(thetaValve*np.pi/180),50*cos(thetaValve*np.pi/180),q6, velCmd)


                                        sm.client.wait_for_result()
					
					ycomd= 30.0+1.3*sm.ToolSize
                                        print "ycom", ycomd

#					flag = sm.xyzShift(0.0,ycomd,0.0,velCmd)

                                        flag = sm.xyzShiftRotate(0.0,ycomd*sin(thetaValve*np.pi/180+90*np.pi/180), ycomd*cos(thetaValve*np.pi/180+90*np.pi/180),q6, velCmd)

					time.sleep(3)
                                        sm.client.wait_for_result()
                                        q1ref,q2ref,q3ref,q4ref,q5ref,q6ref = sm.jointPosition
                                        print "Now I am aligened"
                                        ValveAligned =True
			                TouchAligen = False
###############################################

                q1,q2,q3,q4,q5,q6 = sm.jointPosition
                if ToolFace=="right":
                        q6=q6+10.0*np.pi/180
                        Qtarget=[q1,q2,q3,q4,q5,q6]
                        sm.jointGoto(Qtarget,1.0)
                        sm.client.wait_for_result()

                if ToolFace=="left":
                        q6=q6-12.0*np.pi/180
                        Qtarget=[q1,q2,q3,q4,q5,q6]
                        sm.jointGoto(Qtarget,1.0)
                        sm.client.wait_for_result()



############################################


                if GoEngage:
                           xTorqueref=sm.xTorque
                           while not Engaged:
                              q1,q2,q3,q4,q5,q6 = sm.jointPosition
                              flag = sm.xyzShiftRotate(0.0,-2*sin(q6-q6offset), -2*cos(q6-q6offset),q6, velCmd)
                              time.sleep(1)
                              sm.client.wait_for_result()
                              print "Torque Difference: ",abs( sm.xTorque-xTorqueref)
                              if abs(sm.xTorque-xTorqueref)> 1.6 or sm.xTorque> 4:
                                 flag = sm.xyzShiftRotate(0.0,2*sin(q6-q6offset), 2*cos(q6-q6offset),q6, velCmd)
                                 sm.client.wait_for_result()

                                 flag = sm.xyzShiftRotate(0.0,-2*sin(q6-q6offset+90*np.pi/180), -2*cos(q6-q6offset+90*np.pi/180),q6, 1.5*velCmd)
                                 sm.client.wait_for_result()
                                 flag = sm.xyzShiftRotate(0.0,3*sin(q6-q6offset+90*np.pi/180), 3*cos(q6-q6offset+90*np.pi/180),q6, 1.5*velCmd)
                                 sm.client.wait_for_result()

                                 flag = sm.xyzShiftRotate(0.0,-1*sin(q6-q6offset+90*np.pi/180), -1*cos(q6-q6offset+90*np.pi/180),q6, 1.5*velCmd)
                                 sm.client.wait_for_result()



                                 print "STOPPING!!!"
                                 Engaged = True
                                 GoEngage = False
                                 moveRight = True
                                 moveLeft = True
                                 countR=0
                                 countL= 0
                                 print "Torque Difference: ",abs( sm.xTorque-xTorqueref)
                                 print "I supose that I am engaged, I will try to confirm!"




             




                if confairmEngage:
                          flag = sm.xyzShiftRotate(0.0,10*sin(q6-q6offset), 10*cos(q6-q6offset),q6, velCmd)
                          sm.client.wait_for_result()
                          time.sleep(3)


                          yTorqueref=sm.yTorque
			  while moveRight:
                               flag = sm.xyzShiftRotate(0.0,1*sin(thetaValve*np.pi/180+90*np.pi/180), 1*cos(thetaValve*np.pi/180+90*np.pi/180),q6, velCmd)
                               sm.client.wait_for_result()
                               time.sleep(1)

			       countR = countR+1
			      
			       print "countR", countR
                               if abs(sm.yTorque-yTorqueref)> 0.4:
					moveRight = False
					print " Engaged from left"
			       if countR>5:
					print " I am not engaged"
                                        moveRight = False
		       	  while moveLeft:
                               
			       flag = sm.xyzShiftRotate(0.0,-1*sin(thetaValve*np.pi/180+90*np.pi/180), -1*cos(thetaValve*np.pi/180+90*np.pi/180),q6, velCmd)
                               time.sleep(1)
			       sm.client.wait_for_result()
                               countL = countL+1
                               if abs(sm.yTorque-yTorqueref)> 0.4:
                                        moveLeft = False
                                        print " Engaged from right"
                               if countR>5:
                                        print " I am not engaged"
                                        moveLeft = False

						





############################################
		state_topic = "toolInserted"
		task_topic = 1
                insertToolLocal= False

							
	
	
	
	
	
	
	
	
###################################################
#######          Mode: rotate valve            ####
###################################################
	elif  sm.ur_mode=="rotateValve":


   		if rotateMode ==" rotateOnly":
                        q1,q2,q3,q4,q5,q6 = sm.jointPosition

                        q6=q6+5.0*np.pi/180
                        Qtarget=[q1,q2,q3,q4,q5,q6]
                        flag=sm.jointGoto(Qtarget,2.0)
                        time.sleep(1)
                        sm.client.wait_for_result()


                if rotateMode=="rotateWithAdjasment":
               	    xTorqueref=sm.xTorque
               	    while not Engaged:
                        q1,q2,q3,q4,q5,q6 = sm.jointPosition
                        flag = sm.xyzShiftRotate(0.0,-1*sin(q6-q6offset), -1*cos(q6-q6offset),q6, velCmd)
                        time.sleep(1)
                        sm.client.wait_for_result()
                        print "Torque Difference: ",abs( sm.xTorque-xTorqueref)
                        if abs(sm.xTorque-xTorqueref)> .9 or sm.xTorque> 4:
                                print "STOPPING!!!"
                                Engaged = True
	                print "Torque Difference: ",abs( sm.xTorque-xTorqueref)
                    while   sm.xTorque>9:
                        q1,q2,q3,q4,q5,q6 = sm.jointPosition
                        flag = sm.xyzShiftRotate(0.0, 2*sin(q6-q6offset), 2*cos(q6-q6offset),q6, velCmd)
                        time.sleep(.1)
                        sm.client.wait_for_result()
                        print "Back in  y/z :",  2*sin(q6-q6offset), 2*cos(q6-q6offset)
                    while abs(sm.yTorque) >6:
                          orth_shift =1* sm.yTorque/(abs(sm.yTorque)+.001)

                          flag = sm.xyzShiftRotate(0.0, orth_shift*sin(q6-q6offset+90*np.pi/180), orth_shift*cos(q6-q6offset+90*np.pi/180),q6, 0.7*velCmd)
                          time.sleep(.2)
                          sm.client.wait_for_result()
                          print "moving oth to tool with y/z",  orth_shift*sin(q6-q6offset+90*np.pi/180), orth_shift*cos(q6-q6offset+90*np.pi/180)
                    q1,q2,q3,q4,q5,q6 = sm.jointPosition
                    q6=q6+5.0*np.pi/180
                    Qtarget=[q1,q2,q3,q4,q5,q6]
                    flag=sm.jointGoto(Qtarget,2.0)
                    time.sleep(.2)
                    sm.client.wait_for_result()
                    state_topic = "valveRotated"
                    #xref2,yref2,zref2= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
                    q1,q2,q3,q4,q5,q6 = sm.jointPosition

					
		
	
###################################################
#######          Mode: Rotation    ####
###################################################
	elif  sm.ur_mode=="rotateValve2":

		time.sleep(4)
		if rotateInit:
		        q1,q2,q3,q4,q5,q6 = sm.jointPosition
	
                	q6=q6+15.0*np.pi/180
                	Qtarget=[q1,q2,q3,q4,q5,q6]
               	        flag=sm.jointGoto(Qtarget,6.0)
                	time.sleep(2)
                	sm.client.wait_for_result()
			rotateInit= False 
                xTorqueref=sm.xTorque


		if sm.xTorque<-8:

                	q1,q2,q3,q4,q5,q6 = sm.jointPosition
                	q6=q6-3.0*np.pi/180
                	Qtarget=[q1,q2,q3,q4,q5,q6]
                	flag=sm.jointGoto(Qtarget,6.0)
                	time.sleep(.2)
                	sm.client.wait_for_result()
         
		print Engaged
                while not Engaged:
                        q1,q2,q3,q4,q5,q6 = sm.jointPosition
                        flag = sm.xyzShiftRotate(0.0,-1*sin(q6-q6offset), -1*cos(q6-q6offset),q6, velCmd)
                        time.sleep(1)
                        sm.client.wait_for_result()
                        print "Torque Difference: ",abs( sm.xTorque-xTorqueref)
                        if abs(sm.xTorque-xTorqueref)> .9 or sm.xTorque> 4:
                                print "STOPPING!!!"
                                Engaged = True
	        print "Torque Difference: ",abs( sm.xTorque-xTorqueref)
                while   sm.xTorque>9:
                        q1,q2,q3,q4,q5,q6 = sm.jointPosition
                        flag = sm.xyzShiftRotate(0.0, 2*sin(q6-q6offset), 2*cos(q6-q6offset),q6, velCmd)
                        time.sleep(.1)
			sm.client.wait_for_result()
                        print "Back in  y/z :",  2*sin(q6-q6offset), 2*cos(q6-q6offset)
		while abs(sm.yTorque) >6:
			  orth_shift =2* sm.yTorque/(abs(sm.yTorque)+.001)

            		  flag = sm.xyzShiftRotate(0.0, orth_shift*sin(q6-q6offset+90*np.pi/180), orth_shift*cos(q6-q6offset+90*np.pi/180),q6, 0.7*velCmd)
			  time.sleep(.2)
		          sm.client.wait_for_result()
			  print "moving oth to tool with y/z",  orth_shift*sin(q6-q6offset+90*np.pi/180), orth_shift*cos(q6-q6offset+90*np.pi/180)
                q1,q2,q3,q4,q5,q6 = sm.jointPosition
                q6=q6+15.0*np.pi/180
                Qtarget=[q1,q2,q3,q4,q5,q6]
                flag=sm.jointGoto(Qtarget,5.0)
		time.sleep(.2)
                sm.client.wait_for_result()
                state_topic = "valveRotated"
                #xref2,yref2,zref2= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
                q1,q2,q3,q4,q5,q6 = sm.jointPosition

                task_topic = 1
                Engaged = False
			




	ur_mode_pub.publish(state_topic)
	ur_task_pub.publish(task_topic)








		


if __name__ == '__main__':
    try:
        main()
	
    except rospy.ROSInterruptException:
        pass







