#!/usr/bin/env python

from rbe500_project.srv import rrpIK, rrpIKResponse
import rospy
import math

def callback_server(data):
	# Getting the arm pose from input arguments topic-data_inv parametric varible for the data

	x = data.x
	y = data.y
	z = data.z

	a1 = 0.5
	a2 = 0.425
	a3 = 0.345
	a4 = 0.11

	#inverse equations for scara robot arm - 3dof
	r = math.sqrt(x*x + y*y)
	b2 = (-(a2*a2 + a3*a3 - r*r)/(2*a2*a3))
	if b2 > 1:
		b2 = 1
	else:
		b2 = b2
	q2 = math.acos(b2)

	q3 = a1-a4-z

	phi = math.acos((a2*a2 - a3*a3 + r*r)/(2*a2*r))
	q1 = math.atan2(y,x) - phi
	if q1 < 0:
		q1 = -(q1)
	else:
		q1 = q1
	#returning the joint angles 
	return rrpIKResponse(round(q1,2),round(q2,2),round(q3,2))

def server():	
	#creating node - inver_service_server for ros communication
	rospy.init_node("inver_service_server")

	#Starting the service server 'inver_server' 
	rospy.Service('inver_server', rrpIK, callback_server)
	
	#make server run continously in a loop
	rospy.spin()

if __name__ == "__main__":
	server()
