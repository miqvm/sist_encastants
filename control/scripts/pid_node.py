#!/usr/bin/env python


import rospy
from tf_sim.msg import Float32Stamped

desired_val = 0.0
max_integ_error = 0.0

error_acum = 0.0
error_anterior = 0.0

def exec_pid(error):
  # Les variables globals que s'usaran en la funcio
  global T
  global Kp
  global Kd
  global Ki
  global max_integ_error
  global error_acum
  global error_anterior

  
  # Es suma l'error acumulat
  error_acum = error_acum+error

  # Si aquest error acumulat es major al maxim valor que pot prendre la integral de la part I 
  # s'ajusta aquest error acumulat a dita fita
  if(error_acum>max_integ_error):
    error_acum = max_integ_error
  elif(error_acum<-max_integ_error):
    error_acum = -max_integ_error

 # Es calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
  # u(k) = Kp*e(k)+Kd*[(e(k)-e(k-1)/T]+Ki*part integral (error_acum)
  output = Kp*error+Kd*((error-error_anterior)/T)+Ki*error_acum

  # L'output es dona com el percentatge de gas que ha de fer el cotxe, per tant el valor resultant s'ajusta entre 0 i 100
  if(output>100.0):
    output=100.0
  elif(output<0.0):
    output=0.0

  # Es guarda l'error actual en la variable 'error_anterior' per la seguent iteracio
  # e(k-1) = e(k)
  error_anterior = error
  return output

def callbackDesired(input_val):

  global desired_val

  desired_val = float(input_val.data)


def callbackMeasured(measured_val):

  global desired_val

  error = desired_val - measured_val.data

  ctrl_val = exec_pid(error)

  now = rospy.get_rostime()

  msg = Float32Stamped()
  msg.data = ctrl_val
  msg.header.stamp = now
  pub.publish(msg)
    
def PID():

  global pub
  
  global T
  global Kp
  global Kd
  global Ki
  global max_integ_error

  rospy.init_node('PID')

  freq = rospy.get_param('~frequency', 0.0)
  T = 1.0/freq

  Kp = rospy.get_param('~Kp', 0.0)
  Kd = rospy.get_param('~Kd', 0.0)
  Ki = rospy.get_param('~Ki', 0.0)
  max_integ_term = rospy.get_param('~max_integ_term', 0.0)

  if Ki > 0.0:
    max_integ_error = max_integ_term/Ki

  rospy.Subscriber("~desired_val", Float32Stamped, callbackDesired)
  rospy.Subscriber("~measured_val", Float32Stamped, callbackMeasured)

  pub = rospy.Publisher('~output', Float32Stamped, queue_size=10)

  rospy.loginfo("PID running")


  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':

  PID()
