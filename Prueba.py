from pymavlink import mavutil
import socket as s
from time import sleep
import math as m
import Funciones_auxiliares as fa
import pyquaternion as pq
import numpy as np

Factor_conversion_Aceleracion=0.3048
Factor_conversion_altitud=304.8
Factor_conversion_velocidad=30.48
Factor_conversion_Velocidad_angular=0.01745329525
Factor_conversion_presion=3386.39 / 100



class simulador:

    def __init__(self):
        #IMU
        self.tiempo_unix=0
        self.Xacc=0
        self.Yacc=0
        self.Zacc=0
        self.roll=0
        self.pitch=0
        self.heading=0
        self.rollrate=0
        self.pitchrate=0
        self.headingrate=0
        self.Xgyro=0
        self.Ygyro = 0
        self.Zgyro = 0
        self.campo_mag_x=0
        self.campo_mag_y=0
        self.campo_mag_z=0
        self.presion_abs=0
        self.presion_rel=0
        self.presion_alt =0
        self.temperatura=0
        self.Campos_actualizar = int(0x1fff)
        #GPS
        self.fix_type=3
        self.lat=0
        self.lon=0
        self.alt=0
        self.eph=100
        self.epv=100
        self.velocidad_GPS=0
        self.velocidad_norte=0
        self.velocidad_este=0
        self.velocidad_abajo=0
        self.direccion_movimiento=0
        self.satelites = 10
        # Acciones
        self.controles=[0,0,0,0]
        self.modo=0
        #Conexiones
        self.udp_ip="localhost"
        self.UDP_PORT_recibir=9999
        self.UDP_PORT_enviar=9900
        self.tcp_port='tcpin:localhost:4560'

    def socket_udp(self):
        sock = s.socket(s.AF_INET, s.SOCK_DGRAM) #Abre el socket udp con FG
        return sock #Devuelve el objeto socket

    def Leer_FG(self): #Recibe los datos de FG, En la version definitiva no devolvera un vector, actualizara las propiedades del objeto
        # simulador actualizando los sensores y GPS
        sock = s.socket(s.AF_INET, s.SOCK_DGRAM)  # Abre el socket udp con FG
        sock.bind((self.udp_ip, self.UDP_PORT_recibir)) #Escucha el socket
        data, addr = sock.recvfrom(2048) #Recibe los datos de FG
        data = data.decode("utf-8") #Decodifica el mensaje
        data = data.split(',') #Separa los valores por las comas
        #IMU
        self.tiempo_unix=int(float((data[12]))*1000000)
        self.Xacc=float(data[6])*Factor_conversion_Aceleracion
        self.Yacc=float(data[7])*Factor_conversion_Aceleracion
        self.Zacc=float(data[8])*Factor_conversion_Aceleracion
        self.roll=float(data[17])
        self.pitch=float(data[18])
        self.heading=float(data[19])

        self.rollrate=float(data[9])
        self.pitchrate=float(data[10])
        self.headingrate=float(data[11])

        #Funcion para gyro
        Gyro=self.Obtener_Gyro()
        self.Xgyro=Gyro[0]
        self.Ygyro =Gyro[1]
        self.Zgyro =Gyro[2]

        mag=self.Campo_magnetico()
        self.campo_mag_x=mag[0]
        self.campo_mag_y=mag[1]
        self.campo_mag_z=mag[2]

        self.presion_abs=float(data[15])*Factor_conversion_presion
        self.presion_rel=float(data[16])-float(data[15])*Factor_conversion_presion #REVISAR
        self.presion_alt=float(float(data[14])*Factor_conversion_presion)
        self.temperatura=float(data[13])
        #GPS
        self.lat=int(float(data[0])*10000000)
        self.lon=int(float(data[1])*10000000)
        self.alt=int(float(data[2])*Factor_conversion_altitud)
        self.velocidad_norte=int(float(data[3])*Factor_conversion_velocidad)
        self.velocidad_este=int(float(data[4])*Factor_conversion_velocidad)
        self.velocidad_abajo=int(float(data[5])*Factor_conversion_velocidad)
        self.velocidad_GPS=int(m.sqrt(m.pow(self.velocidad_norte,2)+m.pow(self.velocidad_este,2)))
        aux=m.atan2(self.velocidad_norte,self.velocidad_este)*(180/m.pi)+90
        if aux<0:
            aux+=360
        self.direccion_movimiento=int(aux*100)

    def Escribe_FG(self,comunication):
        Acciones_Control = [self.controles[0], ',', self.controles[1], ',', self.controles[2], ',', self.controles[3], '\n'] #Genera un mensaje separado por comas
        mensaje = ''.join(str(e) for e in Acciones_Control)
        Enviar = str.encode(mensaje)  #Codifica el mensaje para enviarlo
        comunication.sendto(Enviar, (self.udp_ip, self.UDP_PORT_enviar)) #Envia el mensaje a traves del puerto

    def socket_TCP_PX4(self):
        comunication = mavutil.mavlink_connection(self.tcp_port, planner_format=False,notimestamps=True,robust_parsing=True) #Instancia la comunicacion con PX4
        return comunication #Devuelve el objeto socket tcp

    def Inicializar_PX4(self,comunication): #Envia la primera cadena de mensajes para establecer contacto con PX4
        flag=0
        bandera=0
        while flag==0:
            if flag == 0:
                msg = comunication.recv_msg()
                if msg is not None:
                    bandera = bandera + 1
                    if bandera == 2:
                        comunication.mav.heartbeat_send(0, 0, 0, 0, 0, mavlink_version=3)
                        comunication.mav.hil_sensor_send(self.tiempo_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro, self.Zgyro, self.campo_mag_x, self.campo_mag_y, self.campo_mag_z, self.presion_abs, self.presion_rel, self.presion_alt, self.temperatura, self.Campos_actualizar)
                        flag = 1
        return


    def Escribe_PX4(self,comunication):#Envia los primeros mensajes para desbloquear el lookstep por el socket 'comunication'
        comunication.mav.hil_sensor_send(self.tiempo_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro,self.Zgyro, self.campo_mag_x, self.campo_mag_y, self.campo_mag_z,self.presion_abs, self.presion_rel, self.presion_alt, self.temperatura, self.Campos_actualizar)
        comunication.mav.hil_gps_send(self.tiempo_unix, self.fix_type, self.lat, self.lon, self.alt, self.eph, self.epv,self.velocidad_GPS, self.velocidad_norte, self.velocidad_este,self.velocidad_abajo, self.direccion_movimiento, self.satelites, force_mavlink1=False)
        data = comunication.recv_msg()
        return data

    def Leer_PX4(self,comunication):
        data = comunication.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=True)
        if data is not None:
            self.controles = [data.controls[0], data.controls[1], data.controls[2], data.controls[3]]
            self.modo = data.mode
        return data


#--------------------------------------------------------------------------------------------------------------
#-------------------------Funciones auxiliares-----------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------

    def Campo_magnetico(self):
        Fuerza=0.01*fa.Fuerza_magnetica(self.lat,self.lon)
        Declinacion=fa.Declinacion_magnetica(self.lat,self.lon)*m.pi/180
        Inclinacion=fa.Inclinacion_magnetica(self.lat,self.lon)*m.pi/180

        H=Fuerza*m.cos(Inclinacion)
        Z=H*m.tan(Inclinacion)
        X=H*m.cos(Declinacion)
        Y=H*m.sin(Declinacion)

        mag_g=np.array([X,Y,Z])

        roll = pq.Quaternion([1, 0, 0, self.roll])
        pitch = pq.Quaternion([0, 1, 0, self.pitch])
        heading = pq.Quaternion([0, 0, 1, self.heading])
        bodyrot = roll * pitch * heading
        bodyrot = bodyrot.inverse
        mag1 = bodyrot.rotate(mag_g)

        return mag1

    def Obtener_Gyro(self):
        roll = pq.Quaternion([1, 0, 0, self.roll])
        pitch = pq.Quaternion([0, 1, 0, self.pitch])
        heading = pq.Quaternion([0, 0, 1, self.heading])
        bodyrot = roll * pitch * heading

        rollrateP=np.array([self.rollrate,0,0])

        pitchrate=np.array([0,self.pitchrate,0])
        pitchrate=heading.rotate(pitchrate)
        bodyrot_inv=bodyrot.inverse
        pitchrate=bodyrot_inv.rotate(pitchrate)

        headingrate=np.array([0,0,self.headingrate])
        headingrate=bodyrot_inv.rotate(headingrate)

        ret=np.sum((rollrateP,pitchrate,headingrate),axis=1)

        return ret