#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

# VARIABLES GLOBALS
ang_min = 10                        # Angle minim per activar el PID_lineal
po_x = 0.0                          # Punt Objectiu X
po_y = 0.0                          # Punt Objectiu Y
error_acum_lineal = 0.0             # Error acumulat lineal
error_acum_angular = 0.0            # Error acumulat angular
max_integ_error_lineal = 0.0        # Valor maxim integral lineal
max_integ_error_angular = 0.0       # Valor maxim integral angular

# Calcula els errors lineals i angulars a partir de la posicio actual
# En cas que sigui necessari corregir l'error, cridar al PID lineal o angular
def callbackPA(odom_msg):
    # Explicitar quines variables globals s'usaran
    global ang_min
    global po_x
    global po_y

    # Inicialitzar les consignes de velocitat lineal i angular
    c_lin = 0.0
    c_ang = 0.0

    # Obtenir la posicio X i Y, i la velocitat lineal i angular actual
    pa_x = odom_msg.pose.pose.position.x
    pa_y = odom_msg.pose.pose.position.y
    lineal_vel = odom_msg.twist.twist.linear.x
    angular_vel = odom_msg.twist.twist.angular.z

    # Calcul de l'error en X i Y
    e_x = po_x - pa_x
    e_y = po_y - pa_y
    
    # Calcul de la distancia euclidea (error lineal)
    distEucl_error = math.sqrt((e_x) ** 2 + (e_y) ** 2)

    # Si l'error lineal es menor a 0.05 metres
    if(distEucl_error > 0.05):
        # Obtenir la orientacio yaw del robot
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        orientacio = euler[2]
    
        # Calcul de l'error angular
        angular_error = math.acos((math.cos(orientacio)*e_x + math.sin(orientacio)*e_y)/distEucl_error)
        
        # Determinar si l'error angular es positiu o negatiu, en cas de ser-ho canviar el signe de l'error angular
        simbol = math.cos(orientacio)*e_y-math.sin(orientacio)*e_x
        if simbol < 0:
            angular_error = -angular_error

        print(angular_error)

        # Crida al PID angular i guardar el resultat en la consigna angular
        c_ang = PID_angular(angular_error, angular_vel)

        # Si l'error angular es menor a un angle minim, cridar al PID lineal
        if (abs(angular_error)< (ang_min*math.pi/180)):
            # Guardar el resultat en la consigna lineal
            c_lin = PID_lineal(distEucl_error,lineal_vel)

    # Variable de tipus Twist, i guardar les consignes angular i lineal
    vel = Twist()
    vel.linear.x = c_lin
    vel.angular.z = c_ang
    
    # Finalment publicar aquest missatge
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
    error_acum_angular = error_acum_angular + angular_error

    # Si aquest error acumulat es major al maxim valor que pot prendre la integral de la part I 
    # s'ajusta aquest error acumulat a dita fita
    if(error_acum_angular > max_integ_error_angular):
        error_acum_angular = max_integ_error_angular
    elif(error_acum_angular < -max_integ_error_angular):
        error_acum_angular = -max_integ_error_angular

    # Es calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # u(k) = Kp*e(k) - Kd*velocitat angular + Ki*part integral (error_acum)
    output = Kp_angular*angular_error - Kd_angular*angular_vel + Ki_angular*error_acum_angular

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
    error_acum_lineal = error_acum_lineal + lineal_error

    # Si aquest error acumulat es major al maxim valor que pot prendre la integral de la part i
    # s'ajusta aquest error acumulat a dita fita
    if(error_acum_lineal > max_integ_error_lineal):
        error_acum_lineal = max_integ_error_lineal
    elif(error_acum_lineal < -max_integ_error_lineal):
        error_acum_lineal = -max_integ_error_lineal

    # Es calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # u(k) = Kp*e(k) - Kd*velocitat lineal + Ki*part integral (error_acum)
    output = Kp_lineal*lineal_error - Kd_lineal*linear_vel + Ki_lineal*error_acum_lineal

    return output

# Actualitzar el Punt Objectiu X i Y
def callbackPO(PO):
    # Explicitar quines variables globals s'usaran
    global po_x
    global po_y

    po_x = PO.position.x
    po_y = PO.position.y


def PID():
    # Explicitar quines variables globals s'usaran
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

    # Inicialitzar el node amb el nom PID
    rospy.init_node('PID')

    # Obtenir la frequencia
    freq = rospy.get_param('~frequency', 0.0)
    T = 1.0/freq

    # Obtenir les constants K pels PID lineals i angulars
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

    # El node es subscriu als missatges d'odometria del kobuki (tipus Odometry) i al punt objectiu (tipus Pose) del publisher
    rospy.Subscriber("~odom_msg", Odometry, callbackPA)
    rospy.Subscriber("~po", Pose, callbackPO)

    # El node publica un missatges de la consigna de velocitat (tipus TWIST)
    pub = rospy.Publisher('~vel', Twist, queue_size=10)

    rospy.loginfo("callback running")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    PID()