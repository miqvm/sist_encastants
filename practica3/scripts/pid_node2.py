#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

# TODO: posar o linear o lineal q sino es un lio, queda fer PID_lineal

ang_min = 10

po_x = 0.0
po_y = 0.0




def callbackPA(odom_msg):
    global ang_min

    c_lin = 0.0
    c_ang = 0.0

    pa_x = odom_msg.pose.pose.position.x
    pa_y = odom_msg.pose.pose.position.y
    linear_vel = odom_msg.twist.twist.linear.x
    angular_vel = odom_msg.twist.twist.angular.z



    e_x = po_x - pa_x
    e_y = po_y - pa_y
    
    distEucl_error = sqrt((e_x)² - (e_y)²)

    if(distEucl_error > 0.05):
        quaternion = (
            measured_odom.pose.pose.orientation.x,
            measured_odom.pose.pose.orientation.y,
            measured_odom.pose.pose.orientation.z,
            measured_odom.pose.pose.orientation.w)

        euler = tf.transformations.euler from quaternion(quaternion)
        orientacio = euler[2]

        angular_error = arccos((cos(orientacio)*e_x + sin(orientacio)*e_y)/distEucl_error) 
        c_ang = PID_angular(angular_error, angular_vel)

        if (abs(angular_error)< (ang_min*math.pi/180)):
            c_lin = PID_lineal(distEucl_error,linear_error)

    
    vel = Twist()
    vel.linear.x = c_lin
    vel.angular.z = c_ang
    
    pub = rospy.Publisher('~vel', Twist, queue_size=10)


def PID_angular(angular_error, angular_vel):
    # Les variables globals que s'usaran en la funcio
    global T
    global Kp
    global Kd
    global Ki
    global max_integ_error
    global error_anterior


    # Es suma l'error acumulat

    # Si aquest error acumulat es major al maxim valor que pot prendre la integral de la part I 
    # s'ajusta aquest error acumulat a dita fita
    if(error_acum>max_integ_error):
        error_acum = max_integ_error
    elif(error_acum<-max_integ_error):
        error_acum = -max_integ_error

    # Es calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # u(k) = Kp*e(k)+Kd*[(e(k)-e(k-1)/T]+Ki*part integral (error_acum)
    output = Kp*error+Kd*angular_vel+Ki*error_acum

    # Es guarda l'error actual en la variable 'error_anterior' per la seguent iteracio
    # e(k-1) = e(k)
    return output


def PID_lineal(lineal_error,linear_vel):
    # Les variables globals que s'usaran en la funcio
    global T
    global Kp
    global Kd
    global Ki
    global max_integ_error
    global error_anterior


    # Es suma l'error acumulat

    # Si aquest error acumulat es major al maxim valor que pot prendre la integral de la part I 
    # s'ajusta aquest error acumulat a dita fita
    if(error_acum>max_integ_error):
        error_acum = max_integ_error
    elif(error_acum<-max_integ_error):
        error_acum = -max_integ_error

    # Es calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # u(k) = Kp*e(k)+Kd*[(e(k)-e(k-1)/T]+Ki*part integral (error_acum)
    output = Kp*error+Kd*linear_vel+Ki*error_acum

    # Es guarda l'error actual en la variable 'error_anterior' per la seguent iteracio
    # e(k-1) = e(k)
    return output





def callbackPO(PO):
    global po_x
    global po_y

    po_x = PO.position.x
    po_y = PO.position.y


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


    rospy.Subscriber("~odom_msg", Odometry, callbackPA)
    rospy.Subscriber("~po", Pose, callbackPO)

    rospy.loginfo("callback running")


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    PID()
