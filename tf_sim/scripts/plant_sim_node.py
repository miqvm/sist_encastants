#!/usr/bin/env python

import math
import rospy
from tf_sim.msg import Float32Stamped

initial = True
y_prev = 0
u_prev = 0
y_k = 0

def inputClb(input_val):

  global initial
  global y_prev
  global u_prev
  global y_k

  # Y(z)/U(Z) = 40/b * (1-e^-(b/M)*T)/(z-e^-(b/M)*T)

  # Direct form I: y(k) = 40*(1-e^(-b/M*T))/b *u(k-1) + e^(-b/M*T)*y(k-1) 

  if initial:
    y_k = 0
    initial = False
  else:
    y_k = 40*(1-math.exp(-b/M*T))/b * u_prev + math.exp(-b/M*T) * y_prev 

  y_prev = y_k
  u_prev = input_val.data

def timerClb(event):

  msg = Float32Stamped()
  msg.data = y_k
  msg.header.stamp = rospy.get_rostime()
  pub.publish(msg)
    
def plant():

  global T
  global b
  global M

  global pub

  rospy.init_node('plant')

  T = 1.0/rospy.get_param('~frequency', 100)
  b = rospy.get_param('~b', 80)
  M = rospy.get_param('~M', 1000)

  rospy.Subscriber("~input", Float32Stamped, inputClb)
  rospy.Timer(rospy.Duration(T), timerClb)

  pub = rospy.Publisher('~output', Float32Stamped, queue_size=10)

  rospy.loginfo("Plant running")


  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':

  plant()
