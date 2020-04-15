from pymavlink import mavutil
import socket as s
from time import sleep


Factor_conversion_Aceleracion=0.3048
Factor_conversion_altitud=304.8
Factor_conversion_velocidad=30.48
Factor_conversion_Velocidad_angular=0.01745329525
Factor_conversion_presion=1/33.864



class simulador:

    def __init__(self):
        #IMU
        self.tiempo_unix=0
        self.Xacc=0
        self.Yacc=0
        self.Zacc=0
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
        #GPS
        self.lat=0
        self.lon=0
        self.alt=0
        self.eph=65535
        self.epv=65535
        self.velocidad_GPS=65535
        self.velocidad_norte=0
        self.velocidad_este=0
        self.velocidad_abajo=0
        self.direccion_movimiento=65535
        # Acciones
        self.controles=[0,0,0,0]
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
        self.Xgyro=float(data[9])*Factor_conversion_Velocidad_angular
        self.Ygyro =float(data[10])*Factor_conversion_Velocidad_angular
        self.Zgyro = float(data[11])*Factor_conversion_Velocidad_angular
        # self.campo_mag_x=data[10000] #******
        # self.campo_mag_y=data[10000] #******
        # self.campo_mag_z=data[10000] #******
        self.presion_abs=float(data[15])*Factor_conversion_presion
        self.presion_rel=float(data[15])-float(data[14])*Factor_conversion_presion #REVISAR
        # self.presion_alt=data[10000] #******
        self.temperatura=float(data[13])
        #GPS
        self.lat=int(float(data[0]))
        self.lon=int(float(data[1]))
        self.alt=int(float(data[2])*Factor_conversion_altitud)
        self.velocidad_norte=int(float(data[3])*Factor_conversion_velocidad)
        self.velocidad_este=int(float(data[4])*Factor_conversion_velocidad)
        self.velocidad_abajo=int(float(data[5])*Factor_conversion_velocidad)


    def Escribe_FG(self,comunication):
        Acciones_Control = [self.controles[0], ',', self.controles[1], ',', self.controles[2], ',', self.controles[3], '\n'] #Genera un mensaje separado por comas
        mensaje = ''.join(str(e) for e in Acciones_Control) #Une todo en un string
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
                    print(msg)
                    bandera = bandera + 1
                    if bandera == 2:
                        comunication.mav.heartbeat_send(0, 0, 0, 0, 0, mavlink_version=3)
                        print([self.tiempo_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro, self.Zgyro, self.campo_mag_x, self.campo_mag_y, self.campo_mag_z, self.presion_abs, self.presion_rel, self.presion_alt, self.temperatura])
                        comunication.mav.hil_sensor_send(self.tiempo_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro, self.Zgyro, self.campo_mag_x, self.campo_mag_y, self.campo_mag_z, self.presion_abs, self.presion_rel, self.presion_alt, self.temperatura, 7167)
                        flag = 1
        return


    def Escribe_PX4(self,comunication):#Envia los primeros mensajes para desbloquear el lookstep por el socket 'comunication'
        comunication.mav.hil_sensor_send(self.tiempo_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro,self.Zgyro, self.campo_mag_x, self.campo_mag_y, self.campo_mag_z,self.presion_abs, self.presion_rel, self.presion_alt, self.temperatura, 7167)
        comunication.mav.hil_gps_send(self.tiempo_unix, 3, self.lat, self.lon, self.alt, self.eph, self.epv,self.velocidad_GPS, self.velocidad_norte, self.velocidad_este,self.velocidad_abajo, self.direccion_movimiento, 10, force_mavlink1=False)
        data = comunication.recv_msg()
        return data

    def Leer_PX4(self,comunication):
        data=None
        while data==None:
            data = comunication.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=True)
            if data is not None:
                self.controles = [data.controls[0], data.controls[1], data.controls[2], data.controls[3]]
                print(self.controles,'controles')
                self.modo = data.mode




