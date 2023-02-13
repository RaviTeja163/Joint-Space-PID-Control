#!/usr/bin/env python

import rospy
import sys
import tf
import time
import numpy as np
from math import pi, sqrt, atan2, cos, sin
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import *
from geometry_msgs.msg import *
from rbe500_project.srv import *

wPointSeq = 0

class Controller:
   def __init__(self, p=0.0, i=0.0, d=0.0, set_point=0):
      self.kp = p
      self.ki = i
      self.kd = d
      self.set_point = set_point
      self.set_point_values = []
      self.previous_error = 0

   def update(self, current_position, desired_position):
      error = desired_position - current_position
      p_term = self.kp * error
      i_term = self.ki * (self.previous_error + error)
      d_term = self.kd * (error - self.previous_error)
      self.previous_error = error
      return p_term + d_term + i_term

   def setpoint(self, set_point):
      self.set_point = set_point
      self.previous_error = 0
      self.x, self.y, self.z = set_point[0], set_point[1], set_point[2]
      try:
         get_desired_joint_angles = rospy.ServiceProxy('inver_server', rrpIK)
         self.set_point = get_desired_joint_angles(self.x, self.y, self.z)
         self.set_point_values = [self.set_point.q1, self.set_point.q2, self.set_point.q3]
         return self.set_point_values
      except rospy.ServiceException as e:
         print("Service call is failed: %s"%e)


class RRP_CONTROL():
   def __init__(self):
      rospy.init_node("rrp_robot")

      self.sub = rospy.Subscriber("/rrp/joint_states", JointState, self.get_joint_state)
      self.joint1_pub = rospy.Publisher('/rrp/joint1_effort_controller/command', Float64, queue_size=10)
      rospy.sleep(1)
      self.joint2_pub = rospy.Publisher('/rrp/joint2_effort_controller/command', Float64, queue_size=10)
      rospy.sleep(1)
      self.joint3_pub = rospy.Publisher('/rrp/joint3_effort_controller/command', Float64, queue_size=10)
      rospy.sleep(1)
      self.q1 = 0
      self.q2 = 0
      self.q3 = 0
      self.joint_values = [0,0,0]
      self.desired_poses = [[0, 0.77, 0.34],[-0.345, 0.425, 0.24],[-0.67, -0.245, 0.14],[0.77, 0.0, 0.39]]
      self.rate = rospy.Rate(150)

   def update_step_by_step(self):     
      global wPointSeq 
      print('desired position:', self.desired_poses[wPointSeq])
      rrp_pose_controller = Controller()     
      self.joint_values = rrp_pose_controller.setpoint(self.desired_poses[wPointSeq])      
      print('joint_values_from_service:', self.joint_values)


   def get_joint_state(self, msg):
      self.current_position = msg.position
      self.q1 = self.current_position[0]
      self.q2 = self.current_position[1]
      self.q3 = self.current_position[2]


   def run(self):

      global wPointSeq
      rrp_pose_controller = Controller()   
      self.joint_values = rrp_pose_controller.setpoint(self.desired_poses[0])
      print(self.joint_values)

      while not rospy.is_shutdown():

         q1_difference = abs(self.q1) - self.joint_values[0]
         q2_difference = abs(self.q2) - self.joint_values[1]
         q3_difference = abs(self.q3) - self.joint_values[2]

         if abs(q1_difference) <= 0.15:    # acceptable error for joint 1
            joint1_torque = 0
         else:
            obj_controller = Controller(p=5, i=0.5, d=10, set_point=self.joint_values)
            update_torque_values = obj_controller.update(self.q1, self.joint_values[0])
            if update_torque_values < -0.2:
               joint1_torque = -0.2
            elif update_torque_values >= 0.2:
               joint1_torque = 0.2
            else:
               joint1_torque = update_torque_values
         

         if abs(q2_difference) < 0.15:    # acceptable error for joint 2
            joint2_torque = 0
         else:
            obj_controller = Controller(p=5, i=0.5, d=10, set_point=self.joint_values)
            update_torque_values = obj_controller.update(self.q2, self.joint_values[1])
            if update_torque_values < -0.2:
               joint2_torque = -0.2
            elif update_torque_values >= 0.2:
               joint2_torque = 0.2
            else:
               joint2_torque = update_torque_values
         

         if abs(q3_difference) < 0.002:    # acceptable error for joint 3
            joint3_torque = 0
         else:
            obj_controller = Controller(p=10, i=0.01, d=5, set_point=self.joint_values)
            update_torque_values = obj_controller.update(self.q3, self.joint_values[2])

            if update_torque_values < -1.5:
               joint3_torque = -1.5
            elif update_torque_values >= 1.5:
               joint3_torque = 1.5
            else:
               joint3_torque = update_torque_values


         if (abs(q1_difference) < 0.1 and abs(q2_difference) < 0.1 and abs(q3_difference) < 0.1):

            print("----------Reached WayPoint - Waiting for 1 seconds---------")

            # publising zeros to stop
            for t in range(10):

               self.joint1_pub.publish(0.0)
               self.joint2_pub.publish(0.0)
               self.joint3_pub.publish(0.0)

            time.sleep(5)

            wPointSeq = wPointSeq+1
            
            if wPointSeq >= 4:

               print('Task Completed')
               break

            print('New Waypint Loaded')
            self.update_step_by_step()

         #Publish the torque values to the joints
         self.joint1_pub.publish(joint1_torque)
         self.joint2_pub.publish(joint2_torque)
         self.joint3_pub.publish(joint3_torque)
      rospy.spin()

if __name__ == '__main__':
   whatever = RRP_CONTROL()
   whatever.run()