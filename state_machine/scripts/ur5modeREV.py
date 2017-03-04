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

import ur5libmodeREV
ur5libmodeREV.setParams()

# dummy = rospy.wait_for_message('/joint_states',JointState)

ur_mode_pub = rospy.Publisher('/mode_u_main', String , queue_size=10)
ur_task_pub = rospy.Publisher('/task_u_main',Int32 , queue_size=10)

def main():

    ####### Initializations

    rospy.init_node('alpha', anonymous=True)

    sm = ur5libmodeREV.ur5Class()

    print "Waiting for actionLib server..."
    sm.client.wait_for_server()
    print "Connected to server."
    sm.client.cancel_all_goals()
    time.sleep(1.0)
    rate = rospy.Rate(rospy.get_param('/ur5/fbRate'))

    #######


    if rospy.get_param('/ur5/onHusky'):
	dwellTime   	       = 15.0
	velocity  	       = 20.0
	velCmd      	       = 20.0
	degShift   	       = 0.50
	xBackPlane             = 250.0
	yBackPlane             = -150.0

	xValveToTools	       = 0.0
	yValveToTools	       = 450.0
	zValveToTools	       = 50.0
	nWayPoints	       = 4
	
	countSearchValve	=0
	
	
	GripperLen	      = 180.0
	Margin		      = 70.0
	Scaning               = 1


#from Jeff Modefication 
	xTorqueMax 	     = 3.0
	yTorqueMax           = 1.0
	
	headTilt 	= 15.0*np.pi/180.0
	q6rotation 	= 10.0*np.pi/180.0
	yImpact 	= np.inf
	Waiting 	= 0.3
	L		= 1.0
	backRotation 	= 5.0*np.pi/180
        TurnBackAngle 	= 2.0
        AngleMax 	= 200*np.pi/180


	time.sleep(1.0)
	
	state_topic         ="Idle"
	task_topic          = 0    
        sm.ToolFace         ="Right"
	rotateInit          = True 	
	NumberOfSmallCycles = 6

    # Initialize logical variables
    Crawling 	     	= True
    ReadyLoacl 	        = True
    MoveToValve         = True
    BackPlane	        = True
    MoveToTools         = True
    Approching          = True
    Intitial		= True
    GoGripping          = True
    insertToolLocal     = True     
    MoveToPins         = True
    valveSizingLoacl = True

    Reset              = True    
    DoWeNeedAlignedUpDown = False 
   
   
   



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
                            AlignedUpDown = False 
			    xTouch,yTouch,zTouch = sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
			    print"xTouch,yTouch,zTouch" , xTouch,yTouch,zTouch
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
		            
		            
		            
		            
		            
		            
#######################################################################################
###############################THIS IS NOT NEEDED  #####################################

		elif not AlignedUpDown and Aligned and DoWeNeedAlignedUpDown:
			flag = sm.xyzShift(30.0, 0.0, 0.0, velCmd) 
		        sm.client.wait_for_result()
		        Arrived=False
			while not Arrived:
				flag = sm.xyzShift(-2.0, 0.0, 0.0, velCmd)
				time.sleep(0.2)
				sm.client.wait_for_result()
				if sm.anyButton():
				    print "STOPPING!!!"
				    Arrived = True
				else:
				    xNow,yNow,zNow = sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
				    print "x/y/z: ", xNow, yNow, zNow
			if not AlignedUpDown and Arrived: # ALIGN MODE
			    while not AlignedUpDown and Arrived:
				upSide = (sm.buttons[0] or sm.buttons[2])
				downSide = (sm.buttons[1] or sm.buttons[3])
				dq = degShift/180.0*np.pi # degShift  degree shift per second
				Q = sm.jointPosition

				if (upSide and downSide):
				    AlignedUpDown = True
				    sm.q4offset = Q[3] - rospy.get_param('/ur5/poseReady')[3]
				    print "AlignedUpDown! dQ = ", sm.q4offset
				elif (upSide or downSide):
				    print "One side pushbutton pressed"
				    flag = sm.xyzShift(7.0, 0.0, 0.0, velCmd) # If key pressed, come back
				    sm.client.wait_for_result()

				    if upSide: # Assumes green side on right when facing panel
				        sm.q4offset = sm.q4offset + dq
				    else: # blueSide
				        sm.q4offset = sm.q4offset - dq
		        	    Arrived = False 
		        	    
         
		

########################################################################################
########################################################################################
########################################################################################
########################################################################################		            
		            
		            
		            
		            
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
		flag = sm.xyzShift(0.2*sm.pixel_x, 0.2*sm.pixel_y, 0.2*sm.pixel_z, 0.6*velCmd)
		sm.client.wait_for_result()
		#xValve,yValve,zValve= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		#print "xValve/yValve/zValve: ", xValve,yValve,zValve
		state_topic = "valveAlign"
		task_topic = 1
		valveSizingLoacl = True

###################################################
############## Mode : Valve Sizing            ####
###################################################


	elif valveSizingLoacl and sm.ur_mode=="valveSizing":

		
                time.sleep(1.0)
                xValve,yValve,zValve= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
                yValve=yValve-7.0

                q1Valve,q2Valve,q3Valve,q4Valve,q5Valve,q6Valve = sm.jointPosition

                print "xValve/yValve/zValve: ", xValve,yValve,zValve

		flag = sm.xyzShift(0.0, 0.0,-245, velCmd)
	
		sm.client.wait_for_result()

		flag = sm.xyzShift(xTouch-xValve+55,-5,0.0, 0.7*velCmd)

                sm.client.wait_for_result()
		valveSizingLoacl =False


        	state_topic = "sizingDone"
        	task_topic = 1
		#Approching=False

###################################################
###########  Mode :  Go to pins to align theta 6##
###################################################             


        elif  sm.ur_mode=="goPins":
		if MoveToPins: 
                        flag = sm.xyzShift(-xTouch+xValve-55,5.0,0.0, velCmd)

                        sm.client.wait_for_result()


	                flag = sm.xyzShift(0.0, 0.0,245,1.4* velCmd)

        	        sm.client.wait_for_result()

               

                	flag = sm.xyzShiftWayPoints(0,500.0,150,nWayPoints, velocity)
                	sm.client.wait_for_result()
                	state_topic = "atPins"
                	task_topic = 1
	                MoveToPins = False

                Q = sm.jointPosition

                q = Q[0], Q[1], Q[2], Q[3], Q[4], Q[5]- sm.camOffset*np.pi/180
                sm.jointGoto(q,3.0)
                time.sleep(3.0)
                sm.client.wait_for_result()
                sm.q6offset = Q[5]

                state_topic = "atPins"
                task_topic = 1







	    	
###################################################
##############  Mode :  Back to tools   ###########
###################################################  	    	
	    	
	    	
	elif MoveToTools and  sm.ur_mode=="goTools":


                Q = sm.jointPosition
                sm.q6offset = Q[5]                
		time.sleep(2.0)
		flag = sm.xyzShiftWayPoints(15,0.0,-100,nWayPoints, velocity)
		sm.client.wait_for_result()
		state_topic = "atTools"
		task_topic = 1
		MoveToTools = False
		
		
		
		
		
		
		
		
###################################################
##############  Mode :  Tools detection   #########
###################################################  	    	
	    	
	    	
	elif   sm.ur_mode=="scanTools":
		time.sleep(2.0)
		flag = sm.xyzShiftWayPoints(0.0,(Scaning)*2.0,0.0,2,0.3*velocity)
		sm.client.wait_for_result()
		Scaning=-Scaning
		state_topic = "atTools"
		task_topic = 1

	  		
		



		
###################################################
##############  Mode: Tool picking   ##############
###################################################  	    	
	    	
	    	
	elif   sm.ur_mode=="goCorrectTool":
		xCmd = xTouch+100+Margin
		yCmd = yValve+348.0+50.0*(sm.PinNumber-1)
		zCmd = zValve + (225-(180+8.5*(sm.ToolSize- 16)))

		flag = sm.xyzGoto(xCmd,yCmd,zCmd, velocity)
		sm.client.wait_for_result()
		state_topic = "atCorrectTool"
		task_topic = 1

		print" tool size", sm.ToolSize
		print "Pin Number", sm.PinNumber



	  
###################################################
########  Mode 11  Align with the correct tool   #####
###################################################  
	elif  sm.ur_mode=="alignCorrectTool":
		flag = sm.xyzShift(0.0*sm.pixel_x, 0.0*sm.pixel_y, 0.0*sm.pixel_z, 0.2*velCmd)
		xGrip,yGrip,zGrip= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		sm.client.wait_for_result()
		state_topic = "alignCorrectTool"
		task_topic = 1
		GoGripping = True
		
		
		

	  
###################################################
########  Mode:   Grip the correct  tool   #####
###################################################  
	elif  GoGripping  and sm.ur_mode=="goGripping":

		xcmdg=xTouch+27-xGrip
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

		flag = sm.xyzGoto(xValve,yValve,zValve+50, velocity)
		sm.client.wait_for_result()


 		#QValve=[q1Valve,q2Valve,q3Valve,q4Valve,q5Valve,q6Valve]
                #sm.jointGoto(QValve,10.0)
		#time.sleep(10)
                sm.client.wait_for_result()
                print " I am after the Qs, I will get colser to the valve"

#		upValve=50.0
#                flag = sm.xyzShift(0.0,0.0, upValve, velCmd)
#                sm.client.wait_for_result()


		print " xTouch-xValve+70.0,yValve-70.0,-70.0" ,xTouch-xValve+70.0,yValve-70.0,-70.0
		flag = sm.xyzShiftWayPoints(xTouch-xValve+70.0,-50.0,-50.0,2, velocity)
		time.sleep(10)
                sm.client.wait_for_result()
		print "I am at the valve!"



		if sm.ToolFace=="Right":
			sm.q6offset=sm.q6offset#-sm.ValveAngle*np.pi/180
			xinit,yinit,zinit= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		        Qinit = sm.jointPosition

		if sm.ToolFace == "Left":
			sm.q6offset=sm.q6offset - 2*headTilt#-sm.ValveAngle*np.pi/180
			Q = sm.jointPosition
			Qnew = Q[0], Q[1], Q[2], Q[3], Q[4], Q[5]- 2*headTilt
		        sm.jointGoto(Qnew,5.0)
		        time.sleep(5.0)
		        sm.client.wait_for_result()

			xinit,yinit,zinit= sm.wristFK(sm.jointPosition[0],sm.jointPosition[1],sm.jointPosition[2])
		        Qinit = sm.jointPosition


                state_topic = "atValve"
                task_topic = 1
                BackPlane = False
		insertToolLocal= True
	        Reset= True


###################################################
#######          Mode: insert tool   ####
###################################################

	elif  insertToolLocal and sm.ur_mode=="rotateValve":
		

	    # MODES:
	    # ShiftRight
	    # ShiftUp
	    # PreEngage
	    # Cycle: Push + Retract + Rotate


	    
	    
	    ModifiedShiftRight= False
	    
	    
	     
	    if Reset: 
		print "Prep q6"
        	qNew = Qinit[0],Qinit[1],Qinit[2],Qinit[3],Qinit[4],sm.q6offset-backRotation
        #        qNew = sm.jointPosition

		sm.jointGoto(qNew,5.0)
		time.sleep(5.0)
		sm.client.wait_for_result()
		
		
		NumberOfTurn=0
		Reset =False 
		ShiftRight = True
		ShiftUp = False
		PreEngage = False
		Cycle = False
		MadeContact = False
	        q6rotation      = 10.0*np.pi/180.0
	


            if ShiftRight and not ModifiedShiftRight:

                print " "
                print "ShiftRight..."
                print "FirstContact: ", MadeContact

                Q = sm.jointPosition
                x0, y0, z0 = sm.wristFK(Q[0],Q[1],Q[2]) #current position
                yTorque0 = sm.torque.y # unloaded y torque

                # Move right until contact
                Arrived = False
                while not Arrived and not rospy.is_shutdown(): 
                 
	            Q = sm.jointPosition
        	    headAngle = Q[5] - sm.q6offset
                    dy = L*cos(headAngle)
              	    dz =- L*sin(headAngle)


                    flag = sm.xyzShiftRotate2(0.0,dy,dz,0.0,velocity) # move right 1mm
                    time.sleep(1.0*Waiting)
                    sm.client.wait_for_result()
                  
                    print "yTorque/target: ", sm.torque.y, " < ",  yTorque0-yTorqueMax


                    if sm.torque.y < yTorque0 - yTorqueMax:
                        Arrived = True
		                   
                time.sleep(1.0*Waiting)
                Q = sm.jointPosition
                x1, y1, z1 = sm.wristFK(Q[0],Q[1],Q[2])
                yShift = y1 - y0 # amount of y-shift
		print "MadeContact, and  y1-yImpact",MadeContact, y1-yImpact

                print "yShift/yTotal: ", yShift, y1-yImpact

                if MadeContact == False:
                    MadeContact = True # have made first contact
                    ShiftUp = True
                    yImpact = y1
	

                elif yShift > 0.5*sm.ToolSize: # caught opposite lip of wrench
                    PreEngage = True


                elif y1-yImpact > 1*sm.ToolSize: # caught opposite lip of wrench
		    print" I have traveld more than  the   valve size "
                    PreEngage = True
                   # Cycle = True
                    count=0
                else:
                    ShiftUp = True

                ShiftRight = False

##########################################################################
##########################################################################
######################    Modefied ShiftRight       ######################
######################                              ######################

            if ShiftRight and ModifiedShiftRight:

                print " "
                print "ShiftRight..."
                print "FirstContact: ", MadeContact

                Q = sm.jointPosition
                x0, y0, z0 = sm.wristFK(Q[0],Q[1],Q[2]) #current position
                yTorque0 = sm.torque.y # unloaded y torque

                # Move right until contact
                Arrived = False
                while not Arrived and not rospy.is_shutdown(): 
                 
	            Q = sm.jointPosition
        	    headAngle = Q[5] - sm.q6offset
                    dy =   L*cos(headAngle)
              	    dz = - L*sin(headAngle)


                    flag = sm.xyzShiftRotate2(0.0,dy,dz,0.0,velocity) # move right 1mm
                    time.sleep(1.0*Waiting)
                    sm.client.wait_for_result()
                    Q = sm.jointPosition
                    xNow, yNow, zNow = sm.wristFK(Q[0],Q[1],Q[2])

                    print "yTorque/target: ", sm.torque.y, " < ",  yTorque0-yTorqueMax
                    print" yNow -yImpact", yNow- yImpact

                    if sm.torque.y < yTorque0 - yTorqueMax:
                        Arrived = True
                 
                time.sleep(1.0*Waiting)
                Q = sm.jointPosition
                x1, y1, z1 = sm.wristFK(Q[0],Q[1],Q[2])
                yShift = y1 - y0 # amount of y-shift
		print "MadeContact, and     y1-yImpact",MadeContact, y1-yImpact

                print "yShift/yTotal: ", yShift,yShift*cos(headAngle), y1-yImpact

                if MadeContact == False:
                    MadeContact = True # have made first contact
                    ShiftUp = True
                    yImpact = y1
	

                elif yShift > 0.4*valveSize*cos(headAngle) : # caught opposite lip of wrench
                    PreEngage = True


                elif y1-yImpact > 0.75*valveSize: # caught opposite lip of wrench
		    print" I have traveld more than  the 0.75  valve size "
                    PreEngage = False
                    Cycle = True
                    count=0
                else:
                    ShiftUp = True

                ShiftRight = False

##########################################################################
##########################################################################

            if ShiftUp:

                print " "
                print "ShiftUp..."
                Arrived = False
                while not Arrived and not rospy.is_shutdown():
                    print "yTorque/target: ", sm.torque.y, " > ", yTorque0-yTorqueMax/5.0
                    flag = sm.xyzShiftRotate2(0.0,0.0,1.0,0.0,velocity) # move up 1mm
                    time.sleep(1.0*Waiting)
                    sm.client.wait_for_result()
                    if sm.torque.y > yTorque0 - yTorqueMax/10.0:
                        Arrived = True
                       

                time.sleep(1.0*Waiting)
                ShiftUp = False
                TurnBack = True

	    print "Turning Back.."
            if TurnBack:
		NumberOfTurn= NumberOfTurn+1
    	        Q = sm.jointPosition
                q6new = Q[5]- TurnBackAngle*np.pi/180 
                Qnew = Q[0], Q[1], Q[2], Q[3], Q[4], q6new
                sm.jointGoto(Qnew,3.0)
                time.sleep(3.0)
                sm.client.wait_for_result()

                ShiftRight = True
                TurnBack = False 




            if PreEngage:

                print " "
                print "PreEngage..."

                Q = sm.jointPosition
                q6new = Q[5] + q6rotation*0.0 + NumberOfTurn*TurnBackAngle*np.pi/180 # or use headTilt???
                Qnew = Q[0], Q[1], Q[2], Q[3], Q[4], q6new
                sm.jointGoto(Qnew,3.0)
                time.sleep(3.0*Waiting)
                sm.client.wait_for_result()

                PreEngage = False
                Cycle = True
		count=0




            if Cycle and not rospy.is_shutdown():
		count =count +1
                print " "
                print "Cycle: Pushing in..."

                # Prepare for push in
                xTorque0 = sm.torque.x # unloaded x torque
                
                Q = sm.jointPosition
                headAngle = Q[5] - sm.q6offset - headTilt
                dz = -L*cos(headAngle)
                dy = -L*sin(headAngle)

                # Push in
                Arrived = False
                while not Arrived and not rospy.is_shutdown():
                    flag = sm.xyzShiftRotate2(0.0,dy,dz,0.0,velocity)
                    time.sleep(1.0*Waiting)
                    sm.client.wait_for_result()
                    print "xTorque/target: ", sm.torque.x, " > ", xTorque0 + xTorqueMax
                    if sm.torque.x > xTorque0 + xTorqueMax or sm.torque.x > 8.0:
                        Arrived = True

                print " "
                print "Cycle: Sliding right..."

                # Prepare for slide
                yTorque0 = sm.torque.y # unloaded y torque
               
                Q = sm.jointPosition
                headAngle = Q[5] - sm.q6offset - headTilt
                dy = L*cos(headAngle)
                dz = -L*sin(headAngle)

                # Slide right until contact
                Arrived = False
                while not Arrived and not rospy.is_shutdown():
                    print "yTorque/target: ", sm.torque.y, " < ",  yTorque0-yTorqueMax
                    flag = sm.xyzShiftRotate2(0.0,dy,dz,0.0,velocity) # move right 1mm
                    time.sleep(1.0*Waiting)
                    sm.client.wait_for_result()
                    if sm.torque.y < yTorque0 - yTorqueMax:
                        Arrived = True

                # Unload slide pressure while monitoring sm.torque.x
                print " "
                print "Cycle: Unload pressure..."
                Arrived = False
                while not Arrived and not rospy.is_shutdown():
                    print "yTorque/target: ", sm.torque.y, " > ",  yTorque0-yTorqueMax/10.0
                    flag = sm.xyzShiftRotate2(0.0,-dy,-dz,0.0,velocity) # move right 1mm
                    time.sleep(1.0*Waiting)
                    sm.client.wait_for_result()
                    if sm.torque.y > yTorque0 - yTorqueMax/10.0 and sm.torque.y > -6.0: #########CRITICAL################ -5.0?
                        Arrived = True
                    elif sm.torque.x > 8.0:
                        Arrived = True

                # Slightly pull back
                print " "
                print "Cycle: Pulling back..."
               # L = 1.0 
                Q = sm.jointPosition
                headAngle = Q[5] - sm.q6offset - headTilt
                dz = -L*cos(headAngle)
                dy = -L*sin(headAngle)

                # pull back 2mm
                flag = sm.xyzShiftRotate2(0.0,-dy*2.0,-dz*2.0,0.0,velocity)
                time.sleep(1.0*Waiting)
                sm.client.wait_for_result()

                # plus 1mm and further if high torque
                Arrived = False
                while not Arrived and not rospy.is_shutdown():
                    flag = sm.xyzShiftRotate2(0.0,-dy,-dz,0.0,velocity) 
                    time.sleep(1.0*Waiting)
                    sm.client.wait_for_result()
                    print "xTorque/target: ", sm.torque.x, " < ", 5.0
                    if sm.torque.x < 8.0:
                        Arrived = True

                # Rotating
                print " "
                print "Cycle: Rotating..."

		print "count", count
		rotationTime =3
        	if count >NumberOfSmallCycles:
			q6rotation =50*np.pi/180
			rotationTime =7
                Q = sm.jointPosition
                qNew = Q[0], Q[1], Q[2], Q[3], Q[4], Q[5] + q6rotation
                sm.jointGoto(qNew,rotationTime)
                time.sleep(15.0*Waiting)
                sm.client.wait_for_result()

                # Rotating in reverse
                print " "
                print "Cycle: Reverse rotating..."
                Q = sm.jointPosition
                qNew = Q[0], Q[1], Q[2], Q[3], Q[4], Q[5] - q6rotation/6
                sm.jointGoto(qNew,3.0)
                time.sleep(3.0*Waiting)
                sm.client.wait_for_result()

                if Q[5] > AngleMax:
                        Cycle = False
                       #  pull back
                        print " "
                        print " Pulling back to reset..."
                        L = 1.0 
                        Q = sm.jointPosition
                        headAngle = Q[5] - sm.q6offset - headTilt
                        dz = -L*cos(headAngle)
                        dy = -L*sin(headAngle)

                     
                        flag = sm.xyzShiftRotate2(0.0,-dy*100.0,-dz*100.0,0.0,velocity)
                        time.sleep(2.0*Waiting)
                        sm.client.wait_for_result()

                        flag = sm.xyzShiftRotate2(100,0.0,0.0,0.0,velocity)
                        time.sleep(2.0*Waiting)
                        sm.client.wait_for_result()

                        Q = sm.jointPosition
                        headAngle = Q[5] - sm.q6offset - headTilt
     
                        qNew = Q[0], Q[1], Q[2], Q[3], Q[4], sm.q6offset
                        sm.jointGoto(qNew,15.0)
                        time.sleep(10.0*Waiting)
                        sm.client.wait_for_result()

                        flag = sm.xyzShiftRotate2(0.0,-50,0.0,0.0,velocity)
                        time.sleep(5.0*Waiting)
                        sm.client.wait_for_result()
                      

		        flag = sm.xyzShiftRotate2(-100,0.0,100.0,0.0,velocity)
                        time.sleep(15.0*Waiting)
                        sm.client.wait_for_result()
                     


       			qinit = Qinit[0],Qinit[1],Qinit[2],Qinit[3],Qinit[4],Qinit[5]#-NumberOfTurn*AngleBack*np.pi/180
        	        sm.jointGoto(qinit,15.0)
        	        sm.client.wait_for_result()
			time.sleep(10.0*Waiting)
                        Reset = True
                        










	else:
		 sm.ur_mode="Idle"



	ur_mode_pub.publish(state_topic)
	ur_task_pub.publish(task_topic)








		


if __name__ == '__main__':
    try:
        main()
	
    except rospy.ROSInterruptException:
        pass







