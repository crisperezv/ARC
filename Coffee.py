#!/usr/bin/env python

import rospy
import baxter_interface
import baxter_dataflow
import roslib
import cv
import cv2
import cv_bridge
import os
import traceback
import threading
import Queue
import rospkg
import std_msgs
import numpy as np
import math
import tf
from sensor_msgs.msg import Image, JointState
from baxter_core_msgs.srv import ListCameras, SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from baxter_core_msgs.msg import AnalogIOStates, EndEffectorState
from baxter_interface import CHECK_VERSION

'''
Basic Baxter functions
Here you'll find the following functions:
message_matrix_to_pose: Converts a Matrix into Euler angles
move_baxter: Moves an arm to the desired position given as position in space (x,y,z) and angles (x,y,z)
move_head: Lets you move Baxter's head
send_image: Shows an image on Baxter's head screen
get_angles: Prints every joint angle for the desired arm
camera_video: Captures a real time video using Baxter's arm camera
QtoE: Converts and prints Quaternion angles to Euler angles
BothArms: Lets you move both arms at the same time
Other considerations:
X is positive to the front of the robot
Y is positive to the left of the robot (from his perspective)
Z is positive up
To test each function, go to the main function at the end of this code and try running one of the functions, or uncomment one of the examples commented in the main
'''
picture = None

class move():

	def __init__(self, arm):
		
		self.limb = arm
		self.limb_interface = baxter_interface.Limb(self.limb) #Declares the limb
		self.head = baxter_interface.Head()	#Declares the head
		self.gripper = baxter_interface.Gripper(self.limb)	#Declares the gripper
		self.camera = baxter_interface.CameraController('right_hand_camera')
		self.camera.open()
		self.camera.resolution          = self.camera.MODES[0]

		self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size = 10) #Publisher for changing the screen display
	
		#Declares the other arm (The opposite arm to the one passed as argument. If you pass left, declares right as the other arm and vice versa)

		if arm == "left":
			self.other_limb = "right"
		else:
			self.other_limb = "left"

		self.other_limb_interface = baxter_interface.Limb(self.other_limb)

		self.limb_interface.set_joint_position_speed(0.5) #Sets the speed for the args arm. {0.0, 1.0}
		self.other_limb_interface.set_joint_position_speed(0.5) #Sets the speed for the other arm {0.0, 1.0}

		self.angles = self.limb_interface.joint_angles()	#Stores all the joint angles	

	def move_baxter(self, source_frame, trans, rot):
		service_name = '/ExternalTools/' + self.limb + '/PositionKinematicsNode/IKService'
		ik_service = rospy.ServiceProxy(service_name, SolvePositionIK)
		frame = source_frame

		self.limb_interface.set_joint_position_speed(0.5)
	
		matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
			tf.transformations.euler_matrix(rot[0], rot[1], rot[2]))
		
		rospy.wait_for_service(service_name, 10)
		ik_message = SolvePositionIKRequest()
		ik_message.pose_stamp.append(self.message_matrix_to_pose(matrix, frame))
		
		try:
			response = ik_service(ik_message)
		except:
			print "Movement couldn't be executed"

		print response.isValid[0]
		
		if response.isValid[0] == True:
			movimiento = dict(zip(response.joints[0].name, response.joints[0].position))
			self.limb_interface.move_to_joint_positions(movimiento)
		else:
			print "Movement couldn't be executed"

		print response.joints[0].position
		print response.joints[0].name
		
		self.gripper.calibrate()
		self.gripper.open()

	def get_angles(self):	#Muestra en consola la posicion exacta del brazo
		print self.angles

	def mueveteRapido(self,limb1, angle): #El brazo 'limb1' se mueve a velocidad maxima a la posicion 'angle'
				
				limb = baxter_interface.Limb(limb1)				
				limb.set_joint_position_speed(1)	#La velocidad de movimiento se escoge dentro de [0.0 , 1.0]
				limb.move_to_joint_positions(angle)
	
	def mueveteLento(self,limb1, angle):  #El brazo 'limb1' se mueve a velocidad lenta a la posicion 'angle'
				
				limb = baxter_interface.Limb(limb1)				
				limb.set_joint_position_speed(0.2)	#La velocidad de movimiento se escoge dentro de [0.0 , 1.0]
				limb.move_to_joint_positions(angle)

	def calibraGripper(self):		#El gripper necesita calibrarse al iniciar el robot
		self.gripper.calibrate()	
	def abreGripper(self):
		self.gripper.open()
	def cierraGripper(self):
		self.gripper.close()

def main():
	'''
	Posiciones de los brazos
	'''
	### Movimiento brazo izquierdo
	pos1left = {'left_s0': 1.7015681889618952, 'left_s1': -0.13077186216723152, 'left_e0': -1.0032234352770606, 'left_e1': 1.2927623089904325, 'left_w0': -3.02616059930095, 'left_w1': -1.1972720049445655, 'left_w2': 0.3846456825622675}
	pos2 = {'left_w0': -2.397611971464902, 'left_w1': -0.711767085578832, 'left_w2': -0.3144660615165098, 'left_e0': -1.0895098545956152, 'left_e1': 1.3763642619301875, 'left_s0': 1.3360972662481954, 'left_s1': 0.12616991980357528}
	pos3 = {'left_w0': -2.4861993619652845, 'left_w1': -0.6231796950784493, 'left_w2': -0.14534467965214296, 'left_e0': -1.2317865726719872, 'left_e1': 1.492946801809479, 'left_s0': 1.5512380717491248, 'left_s1': 0.35204859081970247}
	pos4 = {'left_w0': -3.0461023495434603, 'left_w1': -0.8555777844430895, 'left_w2': 0.20210196880390327, 'left_e0': -1.0193302335498575, 'left_e1': 1.2168302599901044, 'left_s0': 0.4555922940019679, 'left_s1': 0.08130098175792692}
	pos5 = {'left_w0': -3.008136325043296, 'left_w1': -0.934194299822217, 'left_w2': 3.049937301513174, 'left_e0': -1.0246991663074563, 'left_e1': 1.1470341341413182, 'left_s0': 0.3996019952441503, 'left_s1': 0.07094661143970038}
	pos1cafe = {'left_w0': -3.0464858447404315, 'left_w1': -1.1224904415351515, 'left_w2': 0.2515728492132078, 'left_e0': -0.9069661408372509, 'left_e1': 1.667820611628416, 'left_s0': 1.696966246598239, 'left_s1': -0.253873820395036}		
	pos6 = {'left_w0': -2.8455343615274424, 'left_w1': 0.06787864986392955, 'left_w2': -0.012655341500054663, 'left_e0': -0.7163690279424882, 'left_e1': 2.1260973720091836, 'left_s0': 1.3679273675968178, 'left_s1': -0.10967962633380708}	
	pos7 = {'left_w0': -2.51611198732905, 'left_w1': -0.18637866572807776, 'left_w2': 0.010354370318226542, 'left_e0': -0.8682331259431442, 'left_e1': 2.080844938766564, 'left_s0': 1.6900633330527546, 'left_s1': 0.12962137657631745}
	
	### Movimiento brazo derecho
	pos1right = {'right_s0': -0.6519418348513009, 'right_s1': 0.07133010663667173, 'right_w0': -3.05453924387683, 'right_w1': -1.2532623037023831, 'right_w2': 0.44447093328979864, 'right_e0': 0.46096122675956686, 'right_e1': 1.636374005476765}
	poscasi8 = {'right_s0': 0.31024761434982495, 'right_s1': 0.4636456931383663, 'right_w0': -2.90727708823983, 'right_w1': -1.037354507807511, 'right_w2': 0.9023641984735946, 'right_e0': 1.113286556807839, 'right_e1': 0.7539515572456809}
	pos8 = {'right_s0': 0.5472476460781214, 'right_s1': 0.5426457037144651, 'right_w0': -2.9532965118763927, 'right_w1': -0.9610389636102117, 'right_w2': 0.9564370212465555, 'right_e0': 1.2789564818994636, 'right_e1': 0.5698738626994312}
	poscasi9 = {'right_s0': 0.336325287743877, 'right_s1': 0.3029612056073692, 'right_w0': -2.9671023389673614, 'right_w1': -0.8341020534126937, 'right_w2': 1.0223981951256282, 'right_e0': 1.2590147316569533, 'right_e1': 1.0569127628530501}
	pos9 = {'right_s0': 0.8283496254581234, 'right_s1': 0.3041116911982833, 'right_w0': -3.0115877818160386, 'right_w1': -0.4640291883353377, 'right_w2': 1.1125195664138963, 'right_e0': 1.2866263858388909, 'right_e1': 0.5602864827751474}
	pos10 = {'right_s0': 0.8153107887610974, 'right_s1': 0.279951493789088, 'right_w0': -3.0422673975737466, 'right_w1': -0.32903887900142126, 'right_w2': -0.21053886313727305, 'right_e0': 1.219514726368904, 'right_e1': 0.6296991134269624}

	'''
	Fin posiciones
	'''

	rospy.init_node('move', anonymous = True) #Initializes a ros node
	
	mov = move('right') #Define 'mov' para movimiento en brazo derecho
	#mov.calibraGripper()
	mov.mueveteRapido('right', pos1right)	#Posicion inicial, brazo derecho
	mov.abreGripper()

	mov = move('left')  #Define 'mov' para movimiento en brazo izquierdo
	#mov.calibraGripper()
	mov.cierraGripper()
	
	mov.mueveteRapido('left', pos1left)      #Posicion inicial, brazo izquierdo
	
	cucharadas = input("Cuantas cucharadas de azucar quieres?: ")
	for i in range(cucharadas):	
		mov.mueveteLento('left', pos2)       #Entra la cuchara al azucar
		rospy.sleep(0.5)	
		mov.mueveteLento('left', pos3)       #Sale la cuchara del azucar
		rospy.sleep(0.5)
		mov.mueveteLento('left', pos4)       #LLeva la cuchara al vaso
		rospy.sleep(1)
		mov.mueveteLento('left', pos5)       #Gira la mano para echar el azucar
		rospy.sleep(0.5)
		mov.mueveteRapido('left', pos1cafe)  #Vuelve a la posicion inicial
		rospy.sleep(0.5)

	mov.mueveteLento('left', pos6)       #Entra la cuchara al cafe
	rospy.sleep(0.5)
	mov.mueveteLento('left', pos7)	     #Sale la cuchara del cafe
	rospy.sleep(0.5)
	mov.mueveteLento('left', pos4)	     #Lleva la cuchara al vaso
	rospy.sleep(1)
	mov.mueveteLento('left', pos5)	     #Gira la mano para echar el azucar
	rospy.sleep(0.5)
	mov.mueveteRapido('left', pos1left)	     #Vuelve a la posicion inicial
	

	mov = move('right')	#Define 'mov' para movimiento en brazo derecho
	
	mov.mueveteLento('right', poscasi8)	#Lleva la mano al lado del termo
	rospy.sleep(0.5)
	mov.mueveteLento('right', pos8)		#Lleva la mano al termo, listo para agarrar
	rospy.sleep(0.5)
	mov.cierraGripper()
	rospy.sleep(0.5)
	
	mov.mueveteLento('right', poscasi9)	#Levanta el termo
	rospy.sleep(0.5)
	mov.mueveteLento('right', pos9)		#Lleva el termo al lado del vaso
	rospy.sleep(0.5)
	mov.mueveteLento('right', pos10)	#Gira la mano
	rospy.sleep(0.5) 
	mov.mueveteLento('right', pos9)		#Devuelve la mano
	rospy.sleep(0.5)
	
	mov.mueveteLento('right', poscasi9)	#Lleva el termo arriba de su posicion original
	rospy.sleep(0.5)
	mov.mueveteLento('right', pos8)		#Deja el termo en su lugar
	mov.abreGripper()
	rospy.sleep(2)
	mov.mueveteLento('right', poscasi8)	#Mueve el brazo al lado del termo
	rospy.sleep(0.5)	
	mov.mueveteRapido('right', pos1right)	#El brazo vuelve a su posicion inicial

	#################################################################
	

if __name__ == "__main__":
	main()
