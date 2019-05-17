import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import os
from sys import platform
import argparse
import time
from openpose import pyopenpose as op

# Import Openpose (Windows/Ubuntu/OSX)
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append('Documents/openpose/python');


# Flags
parser = argparse.ArgumentParser()
parser.add_argument("--image_path", default="../../examples/media/COCO_val2014_000000000192.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
args = parser.parse_known_args()

# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "Documents/openpose/models/"

# Add others in path?
for i in range(0, len(args[1])):
    curr_item = args[1][i]
    if i != len(args[1])-1: next_item = args[1][i+1]
    else: next_item = "1"
    if "--" in curr_item and "--" in next_item:
        key = curr_item.replace('-','')
        if key not in params:  params[key] = "1"
    elif "--" in curr_item and "--" not in next_item:
        key = curr_item.replace('-','')
        if key not in params: params[key] = next_item


# Construct it from system arguments
# op.init_argv(args[1])
# oppython = op.OpenposePython()

# Configure depth and color streams
pipeline = rs.pipeline()
bonjour = True
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile=pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_center = 0


#def trouver_distance():
while bonjour:
	frames = pipeline.wait_for_frames()
	depth_frame = frames.get_depth_frame()
	color_frame = frames.get_color_frame()
	if not depth_frame or not color_frame:
		continue

	# Convert images to numpy arrays
	depth_image = np.asanyarray(depth_frame.get_data())
	depth_data=depth_frame.get_data()
	color_image = np.asanyarray(color_frame.get_data())
   

	# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
	depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

	# Stack both images horizontally
	images = np.hstack((color_image, depth_colormap))

	# Show images
	cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
	cv2.imshow('RealSense',images)
	cv2.waitKey(1)
	
	
	if time.time()%3 < 0.1 :
		#cv2.namedWindow('Image ponctuelle',cv2.WINDOW_AUTOSIZE)
		#cv2.imshow('Image ponctuelle',color_image)
		cv2.imwrite('Coloredimage.jpg', color_image)
		img = cv2.imread('Coloredimage.jpg')
		hc = cv2.CascadeClassifier('haarcascade_frontalface_alt2.xml')
		faces = hc.detectMultiScale(img)
		print(faces)
		for face in faces:
			print("regarde ici\n")
			print(color_image[face[1]+face[3]/2,face[0]+face[2]/2])
		
		for face in faces:
			cv2.rectangle(img, (face[0], face[1]), (face[0] + face[2], face[1] + face[3]), (255, 0, 0), 3)
			x1=face[0]
			y1=face[1]
			x2=face[0]+face[2]
			y2=face[1]+face[3]
			print(x1,y1,x2,y2)
			#print(depth_sensor.get_option(rs.option.depth_units))
			#depth_crop=depth_image[face[0]:face[0]+face[2],face[1]:face[1]+face[3]]
			if ((x1 & x2) < 640) &  ((y1 & y2) < 480 ):
				depth_centre=depth_image[face[1]+face[3]/2,face[0]+face[2]/2]
				cv2.circle(img,(face[0] + face[2]/2, face[1] + face[3]/2),5,(0,255,255),-1)
				print(depth_centre)
			else :
				print(x1)
				print(x2)
				print(y1)
				print(y2)
				print("Rectangle sort du cadre")
			cv2.imshow('Face Detection', img)			
			cv2.destroyWindow('Lenas face')
			cv2.imwrite('DetectedFace.jpg', img)
			bonjour = False
			


	if (depth_center!=0 and depth_center<1500) :
		print("On est sense retourner ca")
		print(depth_center)
	else :
		depth_center = 0
	

if bonjour == False :
    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    # Process Image
    datum = op.Datum()
    imageToProcess = cv2.imread('Coloredimage.jpg')
    datum.cvInputData = imageToProcess
    opWrapper.emplaceAndPop([datum])

    # Display Image
    print("Body keypoints: \n" + str(datum.poseKeypoints))
    cv2.imshow("OpenPose 1.4.0 - Tutorial Python API", datum.cvOutputData)
    cv2.waitKey(0)

