from pymavlink import mavutil
import socket as s



class simulador:

    def __init__(self):
        #Valores IMU
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
        #Valores GPS
        self.lat=0
        self.lon=0
        self.alt=0
        self.eph=0
        self.epv=0
        self.velocidad_GPS=0
        self.velocidad_norte=0
        self.velocidad_este=0
        self.velocidad_abajo=0
        self.direccion_movimiento=0
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

    def Actualizar_valores_desde_upd(self,comunication): #Recibe los datos de FG, En la version definitiva no devolvera un vector, actualizara las propiedades del objeto

        #Envio de las acciones a FG
        Acciones_Control = [self.controles[0], ',', self.controles[1], ',', self.controles[2], ',', self.controles[3], '\n'] #Genera un mensaje separado por comas
        mensaje = ''.join(str(e) for e in Acciones_Control) #Une todo en un string
        Enviar = str.encode(mensaje)  #Codifica el mensaje para enviarlo
        comunication.sendto(Enviar, (self.udp_ip, self.UDP_PORT_enviar)) #Envia el mensaje a traves del puerto

        # Actualizando los sensores y GPS desde FG
        sock = s.socket(s.AF_INET, s.SOCK_DGRAM)  # Abre el socket udp con FG
        sock.bind((self.udp_ip, self.UDP_PORT_recibir)) #Escucha el socket
        data, addr = sock.recvfrom(2048) #Recibe los datos de FG
        data = data.decode("utf-8") #Decodifica el mensaje
        data = data.split(',') #Separa los valores por las comas
        #IMU
        self.tiempo_unix=(int(data[12])*1000000)
        self.Xacc=data[6]
        self.Yacc=data[7]
        self.Zacc=data[8]
        self.Xgyro=data[9]
        self.Ygyro = data[10]
        self.Zgyro = data[11]
        self.campo_mag_x=data[1] #******
        self.campo_mag_y=data[1] #******
        self.campo_mag_z=data[1] #******
        self.presion_abs=data[1] #******
        self.presion_rel=data[1] #******
        self.presion_alt=data[1] #******
        self.temperatura=data[1] #******
        #GPS
        self.lat=data[0]
        self.lon=data[1]
        self.alt=data[2]
        self.eph=data[1] #******
        self.epv=data[1] #******
        self.velocidad_GPS=data[1] #******
        self.velocidad_norte=data[4]
        self.velocidad_este=data[5]
        self.velocidad_abajo=data[1] #******
        self.direccion_movimiento=data[3]
        return


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
                        comunication.mav.hil_sensor_send(self.tiempo_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro, self.Zgyro, self.campo_mag_x, self.campo_mag_y, self.campo_mag_z, self.presion_abs, self.presion_rel, self.presion_alt, self.temperatura, 4294967296)
                        flag = 1
        return


    def Actualizar_PX4(self,comunication):#Envia los primeros mensajes para desbloquear el lookstep por el socket 'comunication'
        comunication.mav.hil_sensor_send(self.tiempo_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro, self.Zgyro, self.campo_mag_x, self.campo_mag_y, self.campo_mag_z, self.presion_abs, self.presion_rel, self.presion_alt, self.temperatura, 4294967296)
        comunication.mav.hil_gps_send(self.tiempo_unix, 0, self.lat, self.lon, self.alt, self.eph, self.epv, self.velocidad_GPS, self.velocidad_norte, self.velocidad_este, self.velocidad_abajo, self.direccion_movimiento,0, force_mavlink1=False)
        data=None
        while data is None:
            data = comunication.recv_msg()
            if data is not None:
                self.controles = [data[1], data[1], data[1], data[1]] #Aqui en principio se deben coger los campos del mensaje recibido y guardarlos como las acciones, pero no se hace asi, estoy trabajando en ello
                self.modo = data[1]
                print(data)
        return
