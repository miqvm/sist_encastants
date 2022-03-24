#!/usr/bin/env python


import rospy
from tf_sim.msg import Float32Stamped

def step_gen():

  pub = rospy.Publisher('input_val', Float32Stamped, queue_size=10)

  rospy.init_node('step_sim')

  mg = rospy.get_param('~magnitude', 60)
  freq = rospy.get_param('~frequency', 10)


  rospy.loginfo('Simulating step of magnitude %d', mg)

  rate = rospy.Rate(freq)

  msg = Float32Stamped()

  while not rospy.is_shutdown():

    msg.data = mg
    msg.header.stamp = rospy.get_rostime()

    pub.publish(msg)

    rate.sleep()

if __name__ == '__main__':

  global mg

  try:
    step_gen()
  except rospy.ROSInterruptException:
    pass
