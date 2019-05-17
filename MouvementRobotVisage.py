import pyrealsense2 as rs
import numpy as np
import cv2
import time
import xlsxwriter
import rospy
from geometry_msgs.msg import Twist

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

try:
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
	
		if time.time()%1 < 0.1 :
			#cv2.namedWindow('Image ponctuelle',cv2.WINDOW_AUTOSIZE)
			#cv2.imshow('Image ponctuelle',color_image)
			cv2.imwrite('Coloredimage.jpg', color_image)
			img = cv2.imread('Coloredimage.jpg')
			hc = cv2.CascadeClassifier('haarcascade_frontalface_alt2.xml')
			faces = hc.detectMultiScale(img)
			
			for face in faces:
				cv2.rectangle(img, (face[0], face[1]), (face[0] + face[2], face[1] + face[3]), (255, 0, 0), 3)
				x1=face[0]
				y1=face[1]
				x2=face[0]+face[2]
				y2=face[1]+face[3]
				#print(x1,y1,x2,y2)
				#print(depth_sensor.get_option(rs.option.depth_units))
				depth_crop=depth_image[face[0]:face[0]+face[2],face[1]:face[1]+face[3]]
				if ((x1 & x2) < 640) &  ((y1 & y2) < 480 ):
					depth_center=depth_image[face[1]+face[3]/2,face[0]+face[2]/2]
					cv2.circle(img,(face[0] + face[2]/2, face[1] + face[3]/2),10,(0,255,255),-1)
					#print(depth_centre)
				else :
					print(x1)
					print(x2)
					print(y1)
					print(y2)
					print("Rectangle sort du cadre")
				cv2.imshow('Face Detection', img)
			
			#Calcul de la moyenne des profondeurs des pixels delimites par le rectangle:
			#somme=0
			#n=0
			#if ((x1 & x2) < 640) &  ((y1 & y2) < 480 ) :
			#	for i in range (x1,x2):
			#		for j in range (y1,y2):
			#			somme=somme+depth_image[j,i]
			#			n=n+1
			#	moyenne=somme/n
			#	print(moyenne)	
			
			
			#Ecriture du tableau des profondeurs dans un fichier Excel
			#workbook = xlsxwriter.Workbook('arrays.xlsx')
			#worksheet = workbook.add_worksheet()
			#row = 0
			#for col, data in enumerate(depth_image):
			#	worksheet.write_column(row, col, data)
			#workbook.close()
			
			
				cv2.destroyWindow('Lenas face')
				cv2.imwrite('DetectedFace.jpg', img)
		if((depth_center!=0) and (depth_center < 1500 )):
			bonjour=False
			
	 

finally:

# Stop streaming
	pipeline.stop()
	

class GoForward():
	def __init__(self):
        # initiliaze
		rospy.init_node('GoForward', anonymous=False)
		# tell user how to stop TurtleBot
		rospy.loginfo("To stop TurtleBot CTRL + C")
		# What function to call when you ctrl + c    
		rospy.on_shutdown(self.shutdown)
			
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		# Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		 
		#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
		r = rospy.Rate(10);

		# Twist is a datatype for velocity
		move_cmd = Twist()
		move_cmd_rec=Twist()
		# let's go forward at 0.2 m/s
		vitesse = 0.1 #CE PARAMETRE EST MODIFIABLE
		move_cmd.linear.x = vitesse
		move_cmd_rec.linear.x = -vitesse
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0
		

		# as long as you haven't ctrl + c keeping doing...
		distance = depth_center*0.001
		print(depth_center)
		distance_voulue = 0.8
		bil=True
		bul=True
		difdist=distance_voulue - distance
		t0 = rospy.Time.now().to_sec()
		while not rospy.is_shutdown() and  (difdist < 0) and (bil==True):
			bil=True
			t1=rospy.Time.now().to_sec()
			
			distance=distance-abs(vitesse)*(t1-t0)
			difdist=distance_voulue - distance
			print(distance)
			# publish the velocity
			self.cmd_vel.publish(move_cmd)
			# wait for 0.1 seconds (10 HZ) and publish again
			t0=t1
			r.sleep()
			bul=False
			
				
		while not rospy.is_shutdown() and  (difdist > 0) and (bul==True):
			
			t1=rospy.Time.now().to_sec()
			distance=distance+abs(vitesse)*(t1-t0)
			difdist=distance_voulue - distance
			print(difdist)
			
			self.cmd_vel.publish(move_cmd_rec)
			t0=t1
			r.sleep()
			bil=False
		
							
        
	def shutdown(self):
		# stop turtlebot
		rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
		self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
		rospy.sleep(1)

if __name__ == '__main__':
	try:
		GoForward()
	except:
		rospy.loginfo("GoForward node terminated.")

