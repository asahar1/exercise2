#!/usr/bin/env python3
import numpy as np
import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32


init_ticks = 0 
init_left = 0 

flag1 = 1 
flag2 = 1 

ticks_left = 0 
ticks_right = 0 

distance_left = 0 
distance_right = 0 

theta = 0 

class OdometryNode(DTROS):

	def __init__(self, node_name):
    
    		


        # Initialize the DTROS parent class

		super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
		self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
		self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 350)

        # Subscribing to the wheel encoders
		self.sub_encoder_ticks_left = rospy.Subscriber('/csc22941/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_left,queue_size = 1 )
		self.sub_encoder_ticks_right = rospy.Subscriber('/csc22941/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_right, queue_size =1 )
        #self.sub_executed_commands = rospy.Subscriber(...)

        # Publishers
		self.pub = rospy.Publisher('/csc22941/wheels_driver_node/wheels_cmd',WheelsCmdStamped, queue_size = 10 )
        #self.pub_integrated_distance_right = rospy.Publisher(...)

		self.log("Initialized")
	
	def cb_encoder_left(self, msg):
		global flag1
		global init_left
		global distance_left
		global ticks_left
		
		if (flag1 == 1) :
			init_left = msg.data 
			distance_left = (2 * math.pi * 31.8 * (init_left)) / 135 
			flag1 = 0 
		else:
			ticks_left = msg.data
			distance_left = (2 * math.pi * 31.8 * (ticks_left-init_left)) / 135  
			rospy.loginfo("wheel encoder left init ticks are %f current ticks are %f and distance is %f", init_left, ticks_left , distance_left)
			flag1 = 0

	def cb_encoder_right(self, msg):
		global flag2
		global init_right
		global distance_right
		global ticks_right
		if (flag2 == 1) :
			init_right = msg.data
			distance_right = (2 * math.pi * 31.8 * (init_right)) / 135
			flag2 = 0
		else:
			ticks_right = msg.data
			distance_right = (2 * math.pi * 31.8 * (ticks_right-init_right)) / 135
			rospy.loginfo("wheel encoder right init ticks are %f and current ticks are %f and distance is %f", init_right, ticks_right , distance_right)
			flag2 = 0 
			

		
    	
if __name__ == '__main__':
	
	node = OdometryNode(node_name='my_odometry_node')
	rate = rospy.Rate(10)
	velocity = WheelsCmdStamped()
	velocity.header.stamp = rospy.Time.now()
	velocity.header.frame_id = "/vel"
	velocity.vel_left = 0.05
	velocity.vel_right = -0.05
	print("RADIUS IS:",node._radius)
	node.pub.publish(velocity)
	
	
	theta = ( distance_right - distance_left ) / (2 * 50)
	#theta = math.atan2(math.cos(theta),math.sin(theta))
	print("theta outside loop is: ",theta)
	while not rospy.is_shutdown():
		
		while(abs(theta) < (math.pi/2)):
			theta = ( distance_right - distance_left ) / (2 * 50)
			print("right dist",distance_right)
			print("left dist",distance_left)
			print("in while loop theta is ", theta)
			velocity.vel_left = 0.4
			velocity.vel_right = 0.4
			node.pub.publish(velocity)
		
	
		velocity.vel_left = 0
		velocity.vel_right =  0  
		node.pub.publish(velocity)
		rate.sleep()
	

	rospy.loginfo("wheel_encoder_node is up and running...")


    	
    

