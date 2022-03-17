#!/usr/bin/env python
# coding=utf-8

import rospy
import serial, time
import threading

global arduino

from sensor_msgs.msg import Range

# TO_DO: CREAR tres objetos Event, uno para cada thread de escritura por el puerto serie
event_requestMeasuresSensor0 = threading.Event()
event_requestMeasuresSensor1 = threading.Event()
event_requestMeasuresSensor2 = threading.Event()

def requestMeasuresSensor(sensor,pub): # funcion de solicitud de dato del sensor y publicacion en ROS

  # creamos un mensaje tipo Range
  msg = Range()
  msg.header.frame_id = 'kobuki'
  msg.radiation_type = 1
  msg.field_of_view = 0
  msg.min_range = 0.2
  msg.max_range = 1.5

  while not rospy.is_shutdown(): # mientras no paremos el nodo

    arduino.write(sensor) # escribimos por el serie el numero de sensor

    if sensor == '0':

      # TO_DO: BLOQUEAR el thread con el objeto Event correspondiente
      event_requestMeasuresSensor0.wait()
      msg.range = range0 # rellenamos el mensaje con el rango recibido

    elif sensor == '1':

      # TO_DO: BLOQUEAR el thread con el objeto Event correspondiente
      event_requestMeasuresSensor1.wait()
      msg.range = range1 # rellenamos el mensaje con el rango recibido

    elif sensor == '2':

      # TO_DO: BLOQUEAR el thread con el objeto Event correspondiente
      event_requestMeasuresSensor2.wait()
      msg.range = range2 # rellenamos el mensaje con el rango recibido

    # rellenamos la cabecera del mensaje con la hora actual
    msg.header.stamp = rospy.get_rostime()

    #publicamos el mensaje usando el "publisher" que nos han pasado por parametro
    pub.publish(msg)

  return

def readSensors(): # funcion de recepcion de los datos por el puerto serie

  global range0
  global range1
  global range2

  while not rospy.is_shutdown(): # mientras no paremos el nodo

    buffer = arduino.read_until()

    if len(buffer) == 8: # R X : X X X CR LF <-- estructura de string en el buffer (CR = carriage return; LF = line feed, new line)
                         # 0 1 2 3 4 5  6  7 <-- numero de byte (char)

      #EXTRAER el valor del rango recibido y ALMACENARLO en rangeM en metros
      rangeM = int(buffer[3]+buffer[4]+buffer[5])/100.0

      if buffer[1] == '0':
        # Guardamos el valor recibido en la variable del sensor 0
        range0 = rangeM
        # TO_DO: LIBERAR el thread correspondiente al sensor 0
        event_requestMeasuresSensor0.set()

      elif buffer[1] == '1':
        # Guardamos el valor recibido en la variable del sensor 1
        range1 = rangeM
        # TO_DO: LIBERAR el thread correspondiente al sensor 1
        event_requestMeasuresSensor1.set()

      elif buffer[1] == '2':
        # Guardamos el valor recibido en la variable del sensor 2
        range2 = rangeM
        # TO_DO: LIBERAR el thread correspondiente al sensor 2
        event_requestMeasuresSensor2.set()
  return

if __name__ == "__main__":

  # Inicializamos el nodo
  rospy.init_node('sharp_driver')

  # Leemos los parametros
  port = rospy.get_param('~port', '/dev/ttyUSB0')
  rospy.loginfo('Port: %s', port)

  baud = 9600

  # Creamos tres "publishers", uno para cada sensor, para publicar mensajes del tipo "Range"
  pub0 = rospy.Publisher('~range0', Range, queue_size=1)
  pub1 = rospy.Publisher('~range1', Range, queue_size=1)
  pub2 = rospy.Publisher('~range2', Range, queue_size=1)

  # Inicializamos la conexion serie
  rospy.loginfo("Connecting to the device ...")
  try:
    arduino = serial.Serial(port, baud)
    time.sleep(2)
  except serial.SerialException:
    rospy.logfatal("It was not possible to connect to the device")
    exit(0)
  rospy.loginfo("Successfully connected to the device!")

  # TO_DO: CREAR tres threads que ejecuten la funcion "requestMeasuresSensor", pasando 2 parámetros:
                                                                  # (1) char con el numero de sensor y
                                                                  # (2) el "publisher" correspondiente
  requestMeasuresSensor0 = threading.Thread(target=requestMeasuresSensor, args=('0',pub0))
  requestMeasuresSensor1 = threading.Thread(target=requestMeasuresSensor, args=('1',pub1))
  requestMeasuresSensor2 = threading.Thread(target=requestMeasuresSensor, args=('2',pub2))

  # TO_DO: INICIAR los tres threads creados
  requestMeasuresSensor0.start()
  requestMeasuresSensor1.start()
  requestMeasuresSensor2.start()

  # TO_DO: CREAR un thread que ejecute la funcion "readSensors" e INICIARLO
  readSensor = threading.Thread(target=readSensors)
  readSensor.start()

  print('Threads running')

  # "spin" hasta que paremos el nodo.
  rospy.spin() # Los threads se estan ejecutando

  # TO_DO: ESPERAR a que acaben los cuatro threads
  requestMeasuresSensor0.join()
  requestMeasuresSensor1.join()
  requestMeasuresSensor2.join()
  readSensor.join()

  # Cerramos la conexion serie
  arduino.close()

  print('All done')
