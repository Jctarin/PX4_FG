from pymavlink import mavutil
import socket as s
from time import sleep
import math as m
import Mag_function as fa
import pyquaternion as pq
import numpy as np

Conversion_factor_Accel=0.3048
Conversion_factor_Alt=304.8
Conversion_factor_Vel=30.48
Conversion_factor_Vel_angular=0.01745329525
Conversion_factor_Pres=3386.39 / 100



class simulator:

    def __init__(self):
        #IMU
        self.time_unix=0
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
        self.field_mag_x=0
        self.field_mag_y=0
        self.field_mag_z=0
        self.press_abs=0
        self.press_rel=0
        self.press_alt =0
        self.temp=0
        self.Update_fields = int(0x1fff)
        #GPS
        self.fix_type=3
        self.lat=0
        self.lon=0
        self.alt=0
        self.eph=100
        self.epv=100
        self.vel_GPS=0
        self.vel_n=0
        self.vel_e=0
        self.vel_down=0
        self.direccion_movimiento=0
        self.satellites = 10
        # Accions
        self.controls=[0,0,0,0]
        self.mode=0
        #Conexions
        self.udp_ip="localhost"
        self.UDP_PORT_recibir=9999
        self.UDP_PORT_send_message=9900
        self.tcp_port='tcpin:localhost:4560'

    def socket_udp(self):
        sock = s.socket(s.AF_INET, s.SOCK_DGRAM)
        return sock #Returns the socket object

    def Read_FG(self): #Receive the FG data, In the final version it will not return a vector, it will update the properties of the object
        sock = s.socket(s.AF_INET, s.SOCK_DGRAM)  # Open the udp socket with FG
        sock.bind((self.udp_ip, self.UDP_PORT_recibir)) #Listen to the socket
        data, addr = sock.recvfrom(2048) #Receive the FG data
        data = data.decode("utf-8") #Decode message
        data = data.split(',') #Separate values by commas
        #IMU
        self.time_unix=int(float((data[12]))*1000000)
        self.Xacc=float(data[6])*Conversion_factor_Accel
        self.Yacc=float(data[7])*Conversion_factor_Accel
        self.Zacc=float(data[8])*Conversion_factor_Accel
        self.roll=float(data[17])
        self.pitch=float(data[18])
        self.heading=float(data[19])

        self.rollrate=float(data[9])
        self.pitchrate=float(data[10])
        self.headingrate=float(data[11])

        #Function gyro
        Gyro=self.Gyro()
        self.Xgyro=Gyro[0]
        self.Ygyro =Gyro[1]
        self.Zgyro =Gyro[2]

        mag=self.Magnetic_field()
        self.field_mag_x=mag[0]
        self.field_mag_y=mag[1]
        self.field_mag_z=mag[2]

        self.press_abs=float(data[15])*Conversion_factor_Pres
        self.press_rel=float(data[16])-float(data[15])*Conversion_factor_Pres
        self.press_alt=float(float(data[14])*Conversion_factor_Pres)
        self.temp=float(data[13])
        #GPS
        self.lat=int(float(data[0])*10000000)
        self.lon=int(float(data[1])*10000000)
        self.alt=int(float(data[2])*Conversion_factor_Alt)
        self.vel_n=int(float(data[3])*Conversion_factor_Vel)
        self.vel_e=int(float(data[4])*Conversion_factor_Vel)
        self.vel_down=int(float(data[5])*Conversion_factor_Vel)
        self.vel_GPS=int(m.sqrt(m.pow(self.vel_n,2)+m.pow(self.vel_e,2)))
        aux=m.atan2(self.vel_n,self.vel_e)*(180/m.pi)+90
        if aux<0:
            aux+=360
        self.direccion_movimiento=int(aux*100)

    def Write_FG(self,comunication):
        Control_Actions = [self.controls[0], ',', self.controls[1], ',', self.controls[2], ',', self.controls[3], '\n'] #Generate a comma separated message
        message = ''.join(str(e) for e in Control_Actions)
        send_message = str.encode(message)  #Encrypt the message to send it
        comunication.sendto(send_message, (self.udp_ip, self.UDP_PORT_send_message)) #Send the message through the port

    def socket_TCP_PX4(self):
        comunication = mavutil.mavlink_connection(self.tcp_port, planner_format=False,notimestamps=True,robust_parsing=True) #Instance communication with PX4
        return comunication #Returns the socket tcp object

    def Inicialice_PX4(self,comunication): #Send the first chain of messages to contact PX4
        flag=0
        flag_2=0
        while flag==0:
            if flag == 0:
                msg = comunication.recv_msg()
                if msg is not None:
                    flag_2 = flag_2 + 1
                    if flag_2 == 2:
                        comunication.mav.heartbeat_send(0, 0, 0, 0, 0, mavlink_version=3)
                        comunication.mav.hil_sensor_send(self.time_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro, self.Zgyro, self.field_mag_x, self.field_mag_y, self.field_mag_z, self.press_abs, self.press_rel, self.press_alt, self.temp, self.Update_fields)
                        flag = 1
        return


    def Write_PX4(self,comunication):#Send the first messages to unlock the lookstep through the 'communication' socket
        comunication.mav.hil_sensor_send(self.time_unix, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro,self.Zgyro, self.field_mag_x, self.field_mag_y, self.field_mag_z,self.press_abs, self.press_rel, self.press_alt, self.temp, self.Update_fields)
        comunication.mav.hil_gps_send(self.time_unix, self.fix_type, self.lat, self.lon, self.alt, self.eph, self.epv,self.vel_GPS, self.vel_n, self.vel_e,self.vel_down, self.direccion_movimiento, self.satellites, force_mavlink1=False)
        data = comunication.recv_msg()
        return data

    def Read_PX4(self,comunication):
        data = comunication.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=True)
        if data is not None:
            self.controls = [data.controls[0], data.controls[1], data.controls[2], data.controls[3]]
            self.mode = data.mode
        return data


#--------------------------------------------------------------------------------------------------------------
#-------------------------New Functions------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------

    def Magnetic_field(self):
        strength=0.01*fa.mag_strength(self.lat,self.lon)
        Declination=fa.mag_Declination(self.lat,self.lon)*m.pi/180
        Inclination=fa.mag_Inclination(self.lat,self.lon)*m.pi/180

        H=strength*m.cos(Inclination)
        Z=H*m.tan(Inclination)
        X=H*m.cos(Declination)
        Y=H*m.sin(Declination)

        mag_g=np.array([X,Y,Z])

        roll = pq.Quaternion([1, 0, 0, self.roll])
        pitch = pq.Quaternion([0, 1, 0, self.pitch])
        heading = pq.Quaternion([0, 0, 1, self.heading])
        bodyrot = roll * pitch * heading
        bodyrot = bodyrot.inverse
        mag1 = bodyrot.rotate(mag_g)

        return mag1

    def Gyro(self):
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