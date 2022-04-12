#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

ang_min = 10

po_x = 0.0
po_y = 0.0
error_acum_lineal = 0.0
error_acum_angular = 0.0
max_integ_error_angular = 0.0
max_integ_error_lineal = 0.0

def callbackPA(odom_msg):
    global ang_min
    global po_x
    global po_y


    c_lin = 0.0
    c_ang = 0.0

    pa_x = odom_msg.pose.pose.position.x
    pa_y = odom_msg.pose.pose.position.y
    lineal_vel = odom_msg.twist.twist.linear.x
    angular_vel = odom_msg.twist.twist.angular.z

    e_x = po_x - pa_x
    e_y = po_y - pa_y
    
    distEucl_error = math.sqrt((e_x) ** 2 + (e_y) ** 2)

    if(distEucl_error > 0.05):
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        orientacio = euler[2]
    
        angular_error = math.acos((math.cos(orientacio)*e_x + math.sin(orientacio)*e_y)/distEucl_error)
        simbol = math.cos(orientacio)*e_y-math.sin(orientacio)*e_x
        if simbol < 0:
            angular_error = -angular_error

        print(angular_error)


        c_ang = PID_angular(angular_error, angular_vel)

        if (abs(angular_error)< (ang_min*math.pi/180)):
            c_lin = PID_lineal(distEucl_error,lineal_vel)

    
    vel = Twist()
    vel.linear.x = c_lin
    vel.angular.z = c_ang
    
    pub.publish(vel)
    


def PID_angular(angular_error, angular_vel):
    # Les variables globals que s'usaran en la funcio
    global T
    global Kp_angular
    global Kd_angular
    global Ki_angular
    global max_integ_error_angular
    global error_acum_angular


    # Es suma l'error acumulat

    # Si aquest error acumulat es major al maxim valor que pot prendre la integral de la part I 
    # s'ajusta aquest error acumulat a dita fita
    if(error_acum_angular > max_integ_error_angular):
        error_acum_angular = max_integ_error_angular
    elif(error_acum_angular < -max_integ_error_angular):
        error_acum_angular = -max_integ_error_angular

    # Es calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # u(k) = Kp*e(k)+Kd*[(e(k)-e(k-1)/T]+Ki*part integral (error_acum)
    output = Kp_angular*angular_error - Kd_angular*angular_vel + Ki_angular*error_acum_angular

    # Es guarda l'error actual en la variable 'error_anterior' per la seguent iteracio
    # e(k-1) = e(k)
    return output


def PID_lineal(lineal_error,linear_vel):
    # Les variables globals que s'usaran en la funcio
    global T
    global Kp_lineal
    global Kd_lineal
    global Ki_lineal
    global max_integ_error_lineal
    global error_acum_lineal


    # Es suma l'error acumulat

    # Si aquest error acumulat es major al maxim valor que pot prendre la integral de la part I 
    # s'ajusta aquest error acumulat a dita fita
    if(error_acum_lineal > max_integ_error_lineal):
        error_acum_lineal = max_integ_error_lineal
    elif(error_acum_lineal < -max_integ_error_lineal):
        error_acum_lineal = -max_integ_error_lineal

    # Es calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # u(k) = Kp*e(k)+Kd*[(e(k)-e(k-1)/T]+Ki*part integral (error_acum)
    output = Kp_lineal*lineal_error - Kd_lineal*linear_vel + Ki_lineal*error_acum_lineal

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
    global Kp_lineal
    global Kd_lineal
    global Ki_lineal
    global max_integ_error_lineal
    global Kp_angular
    global Kd_angular
    global Ki_angular
    global max_integ_error_angular


    rospy.init_node('PID')
    freq = rospy.get_param('~frequency', 0.0)
    T = 1.0/freq

    Kd_lineal = rospy.get_param('~Kd_lineal', 0.0)
    Kp_lineal = rospy.get_param('~Kp_lineal', 0.0)
    Ki_lineal = rospy.get_param('~Ki_lineal', 0.0)
    max_integ_term_lineal = rospy.get_param('~max_integ_term_lineal', 0.0)

    Kp_angular = rospy.get_param('~Kp_angular', 0.0)
    Kd_angular = rospy.get_param('~Kd_angular', 0.0)
    Ki_angular = rospy.get_param('~Ki_angular', 0.0)
    max_integ_term_angular = rospy.get_param('~max_integ_term_angular', 0.0)



    if Ki_lineal > 0.0:
        max_integ_error_lineal = max_integ_error_lineal/Ki_lineal

    if Ki_angular > 0.0:
        max_integ_error_angular = max_integ_error_angular/Ki_angular

    rospy.Subscriber("~odom_msg", Odometry, callbackPA)
    rospy.Subscriber("~po", Pose, callbackPO)

    pub = rospy.Publisher('~vel', Twist, queue_size=10)

    rospy.loginfo("callback running")


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    PID()
