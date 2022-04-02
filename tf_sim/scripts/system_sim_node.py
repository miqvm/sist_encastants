#!/usr/bin/env python


import rospy
from tf_sim.msg import Float32Stamped

# S'inicialitzen les variables globals
global k1
k1 = 30.0           # Grup 4 -> K=30
global y_k_1
y_k_1 = 0.0
global y_k_2
y_k_2 = 0.0
global x_k_1
x_k_1 = 0.0
global x_k_2
x_k_2 = 0.0

global k2
k2 = 30.0           # Grup 4 -> K=30
global f_k_1
f_k_1 = 0.0
global f_k_2
f_k_2 = 0.0


# Calcul de y[k] a partir de la forma Directe I
## INPUT: x[k]
## OUTPUT: y[k]
def direct_form_I(x_k):

  # S'avisa que s'usaran les variables globals
  global k1
  global y_k_1
  global y_k_2
  global x_k_1
  global x_k_2

  # Calcul de y[k] = 1.8925*y[k-1] - 0.9025*y[k-2] + 0.1*x[k-1] - 0.09*x[k-2]
  y_k = 1.8925*y_k_1 - 0.9025*y_k_2 + 0.1*x_k_1 - 0.09*x_k_2

  # Es guarden els valors actuals per la seguent iteracio
  y_k_2 = y_k_1   # y[k-1] = y[k-2]
  y_k_1 = y_k     # y[k-1] = y[k]

  x_k_2 = x_k_1   # x[k-2] = x[k-1]
  x_k_1 = x_k     # x[k-1] = x[k]

  return y_k


# Calcul de y[k] a partir de la forma Directe II
## INPUT: x[k]
## OUTPUT: y[k]
def direct_form_II(x_k):

  # S'avisa que s'usaran les variables globals
  global k2
  global f_k_1
  global f_k_2
  
  # Es calcula f[k] = x[k] + 1.8925*f[k-1] - 0.9025*f[k-2]
  f_k = x_k + 1.8925*f_k_1 - 0.9025*f_k_2

  # Es calcula y[k] = 0.1*f[k-1] - 0.09*f[k-2]
  y_k = 0.1*f_k_1 - 0.09*f_k_2

  # Es guarden els valors actuals per la seguent iteracio
  f_k_2 = f_k_1   # f[k-2] = f[k-1]
  f_k_1 = f_k     # f[k-1] = f[k]

  return y_k


def callback(input_val):

  y1 = direct_form_I(input_val.data)
  y2 = direct_form_II(input_val.data)

  now = rospy.get_rostime()

  msg1 = Float32Stamped()
  msg1.data = y1
  msg1.header.stamp = now
  pub1.publish(msg1)

  msg2 = Float32Stamped()
  msg2.data = y2
  msg2.header.stamp = now
  pub2.publish(msg2)

    
def system():

  global pub1
  global pub2

  rospy.init_node('system')

  rospy.Subscriber("~input", Float32Stamped, callback)

  pub1 = rospy.Publisher('output_val_1', Float32Stamped, queue_size=10)
  pub2 = rospy.Publisher('output_val_2', Float32Stamped, queue_size=10)

  rospy.loginfo("System running")


  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':

  system()
