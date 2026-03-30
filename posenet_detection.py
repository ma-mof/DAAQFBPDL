#import sys
#import argparse
#import cv2
#import sys,time
#import numpy as np
import drone
import control
import time

from jetson_inference import poseNet
from jetson_utils import videoSource, videoOutput, Log, cudaDrawLine, cudaFont


def initalize_NN():

    # load the pose estimation model
    net = poseNet("resnet18-body", 0.15) #nn, treshold

    # create video sources & outputs
    print("Setting input video source. ")
    control.debug_writer_general("Setting input video source. ")
    input = videoSource('/dev/video0',argv=['--input-flip=rotate-180'])#(args.input, argv=sys.argv)
    output = videoOutput("")
    print("Input & output video source DEFINED.")
    control.debug_writer_general("Input & output video source DEFINED.")
    font = cudaFont(size=32 )
    return (net, input,output, font)



def Pose_detection(net, input, output,font, visualize=False, control_bool=False):
    # process frames until EOS or the user exits
    while True:
        # capture the next image
        img = input.Capture()
        frame_width=img.width
        frame_height=img.height
        #print("IMG width, height: ", frame_width," ", frame_height)
        if img is None: # timeout
            continue  

        # perform pose estimation (with overlay)
        #poses = net.Process(img, overlay=args.overlay) #original
        #mateo 15.8
        poses = net.Process(img, overlay="links,keypoints")
        
	# print the pose results
        poses_no= len(poses)
        print("Detected {:d} objects in image".format(len(poses)))
        control.debug_writer_general("Detected {:d} objects in image".format(len(poses)))

        x= [None] * 18
        y= [None] * 18

        for pose in poses:
            if pose.ID==0:
                print("Pose: ", pose)
                print("Pose keypoints: ", pose.Keypoints)
                print("Len keypoints: ", len(pose.Keypoints))
                control.debug_writer_general(f"Pose: {pose}; Pose keypoints: {pose.Keypoints}; Len keypoints: {len(pose.Keypoints)}")

            ### dodano 15.8. mateo
                if (len(pose.Keypoints)) is not None:       
                    for i in pose.Keypoints:
                        if i is not None and i.ID is not None and (i.ID == 5):
                            x[5]=i.x
                            y[5]=i.y
                            print("ID 5 detected! X,Y: ", x[5], ", ", y[5])
                            control.debug_writer_general(f"ID 5 detected! X,Y: {x[5]}, {y[5]}")

                        if i is not None and i.ID is not None and (i.ID == 6): #ID 6 right shoulder
                            x[6]=i.x
                            y[6]=i.y
                            print("ID 8 detected! X,Y: ", x[6], ", ", y[6])
                            control.debug_writer_general(f"ID 6 detected! X,Y: {x[6]}, {y[6]}")

                        if i is not None and i.ID is not None and (i.ID == 8): #ID=8 right elbow
                            x[8]=i.x
                            y[8]=i.y
                            print("ID 8 detected! X,Y: ", x[8], ", ", y[8])
                            control.debug_writer_general(f"ID 8 detected! X,Y: {x[8]}, {y[8]}")

                        if i is not None and i.ID is not None and (i.ID == 10): #ID=10 right wrist
                            x[10]=i.x
                            y[10]=i.y
                            print("ID 10 detected! X,Y: ", x[10], ", ", y[10])      
                            control.debug_writer_general(f"ID 10 detected! X,Y: {x[10]}, {y[10]}")

        if (x[5] is not None) and (x[6] is not None):
            # mjerenje udaljenosti lijevog i desnog ramena u [px]
            pix_dist_x=x[5]-x[6]
            pix_dist_y=y[5]-y[6]
            
            # estimacija udaljenosti lijevog i desnog ramena u [cm] --> coef_dist_shoulders
            real_dist_x=pix_dist_x/control.coef_dist_shoulders
            real_dist_y=pix_dist_y/control.coef_dist_shoulders

            # estimacija udaljenosti čovjeka u [cm] --> y=552-x*7.2
            real_dist_z=control.dist_human_b-real_dist_x*control.dist_human_a

            directions_pix=(pix_dist_x,pix_dist_y,0)
            directions_real=(real_dist_x,real_dist_y,real_dist_z)

            body_centre_x=(x[5]+x[6])/2
            body_centre_y=(y[5]+y[6])/2

            heading_rel= (body_centre_x - (frame_width/2))/frame_width
            pitch_rel=(-body_centre_y + (frame_height/2))/frame_height

            heading_abs=heading_rel*360
            
	        # vizualizacija udaljenosti središta ramena naspram centra slike
            cudaDrawLine(img, (frame_width/2,frame_height/2), (body_centre_x ,frame_height/2), (200,0,200,200), 5)
            cudaDrawLine(img, (5,frame_height/2), (5 ,body_centre_y), (100,0,255,200), 10)

            print("X, y, z [px]:", directions_pix)
            print("ESTIMATION: X, y, z [cm]:", directions_real)
            control.debug_writer_general(f"X, y, z [px]: {directions_pix}; ESTIMATION: X, y, z [cm]: {directions_real}")

            #print("Sending movement command YAW:", heading_abs)
            if (y[8] is not None) and  (y[10] is not None) and (y[8]>y[10]):    
                print("Right wrist is above elbow, sending COMANDS to DRONE") 
                control.debug_writer_general("Right wrist is above elbow, sending COMANDS to DRONE")    
        
                if control_bool==True:
                    control.setXdelta(heading_rel)
                    control.debug_writer_general(f"Current heading: {drone.get_heading():.2f}")
                    control.debug_writer_general(f"Going to relative heading: {heading_abs:.2f}")
                    #control.setZDelta(real_dist_z/100-4) #real_dist_z[m]
                    control.setZDelta(pitch_rel) #real_dist_z[m]
                    control.control_drone()
                    font.OverlayText(img, frame_width, frame_height, "CONTROL", round(frame_width/2), round((3*frame_height)/4),  font.White, font.Gray40)
                    control.debug_writer_position(drone.get_location(), drone.get_altitude(), drone.get_velocity(), drone.get_heading(), poses_no, True, heading_rel, pitch_rel)
                else:
                    print("No controll sent to DRONE")
                    control.debug_writer_general("No controll sent to DRONE") 
       
            else:
                print("Right wrist is below elbow, NOT sending comands to drone")
                control.debug_writer_general("Right wrist is below elbow, NOT sending comands to drone")
                control.debug_writer_position(drone.get_location(), drone.get_altitude(), drone.get_velocity(), drone.get_heading(),poses_no, False, heading_rel, pitch_rel)

            ###
        else:
            heading_rel=0
            pitch_rel=0
            control.debug_writer_position(drone.get_location(), drone.get_altitude(), drone.get_velocity(), drone.get_heading(),poses_no, False, heading_rel, pitch_rel)

        if visualize==True:
            initialize_video_stream(net, output, img)
        # exit on input/output EOS

        if not input.IsStreaming() or not output.IsStreaming():
            drone.return_to_launch_location()

            if input.IsStreaming():
                input.Close()  
            if output.IsStreaming():
                output.Close()

            while True:
                print (" Altitude: ", drone.get_altitude())
                control.debug_writer_general(f"Altitude: {drone.get_altitude():.2f}")
                #Break and return from function just below target altitude.
                if drone.get_altitude()<=0.25:
                    print ("Reached target altitude")
                    control.debug_writer_general("Reached target altitude")
                    break
                time.sleep(1)

            print("Drone LANDED")
            control.debug_writer_general("Drone LANDED")
            break

        if drone.get_mode() == "STABILIZE":
            drone.set_flight_mode("RTL")
            print("Program STOPED, changed MODE >> STABILIZE!")
            control.debug_writer_general("Program STOPED, changed MODE >> STABILIZE!")
    	# stop Jetson Inference
            if input.IsStreaming():
                input.Close()
            if output.IsStreaming():
                output.Close()
            
            time.sleep(0.001)
            while True:
                print (" Altitude: ", drone.get_altitude())
                control.debug_writer_general(f"Altitude: {drone.get_altitude():.2f}")
                #Break and return from function just below target altitude.
                if drone.get_altitude()<=0.25:
                    print ("Drone LANDED")
                    control.debug_writer_general("Drone LANDED")
                    break
                time.sleep(1)
            break

        elif drone.get_mode() == "RTL":
            print("Program STOPED, changed MODE >> RTL!")
            control.debug_writer_general("Program STOPED, changed MODE >> RTL!")
    	# stop Jetson Inference
            if input.IsStreaming():
                input.Close()
            if output.IsStreaming():
                output.Close()
            drone.set_flight_mode("RTL")
            time.sleep(0.001)
            while True:
                print (" Altitude: ", drone.get_altitude())
                control.debug_writer_general(f"Altitude: {drone.get_altitude():.2f}")
                #Break and return from function just below target altitude.
                if drone.get_altitude()<=0.25:
                    print ("Drone LANDED")
                    control.debug_writer_general("Drone LANDED")
                    break
                time.sleep(1)
            break

def initialize_video_stream(net, output, img):
    # render the image
    output.Render(img)

    # update the title bar
    output.SetStatus("{:s} | Network {:.0f} FPS".format("resnet18-body", net.GetNetworkFPS()))

    # print out performance info
    #net.PrintProfilerTimes()

################### POCETAK SKRIPTE ##################

vehicle = drone.connect_drone('/dev/ttyACM0', 57600) # ('udp:0.0.0.0:14550',14550) #('/dev/ttyACM0', 57600) #('udp:0.0.0.0',14550)
control.configure_PID("PID")
control.initialize_debug_logs("/home/jetson/DroneKit/test/f_08_AUTO_TEST_FULLY_CENTERED/debug/test1")

#drone.land()
#time.sleep(25)
while drone.get_mode() != "GUIDED":
    time.sleep(1)
    print("Drone is not in GUIDED mode!")
    control.debug_writer_general("Drone is not in GUIDED mode!")
print("Drone mode: GUIDED!")
control.debug_writer_general("Drone mode: GUIDED!")
time.sleep(10)
drone.arm_and_takeoff(5)

net, input,output, font=initalize_NN()
Pose_detection(net, input, output,font, visualize=True, control_bool=True)

print("Jetson is INACTIVE; start again program to controll")
control.debug_writer_general("Jetson is INACTIVE; start again program to controll")
time.sleep(10)
    
	
