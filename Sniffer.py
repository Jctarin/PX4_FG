from pymavlink import mavutil
from time import sleep

#Genera la comunicacion TCP con PX4 y devuelve una instancia del objeto tcpin
msrc = mavutil.mavlink_connection('tcpin:localhost:4560', planner_format=False, robust_parsing=True,source_system=0)

flag=0
bandera=0
n=0
while True:
    #Se envian los comandos de heartbeat por primera vez despues de que PX4 haya enviado los dos primeros mensajes que dan comienzo a la comunicacion
    if flag==0:
        msg = msrc.recv_msg()
        if msg is not None:
           print(msg)
           bandera=bandera+1 #Alerta de un mensaje recibido
           if bandera==2: #Una vez recibido se envia Heartbeat y el primer mensaje de sensores
               msrc.mav.heartbeat_send(0, 0, 0, 0, 0, mavlink_version=3)
               msrc.mav.hil_sensor_send(1585357843637000, 0.3, 0.02, -9.8, 0.007838902, -0.00140664866, 0.00710141938, 0.211196527, 0.011697528, 0.42982325, 955.9833, 0.011697528, 488.0451, 0.0, 7167)
               flag=1
    else:
        n+=5000 #Esto simula el paso del tiempo del simulador para engañar a px4 (las unidades de tiempo son microsegundos)
        msrc.mav.hil_sensor_send(1585178126095000+n, 0.3, 0.02, -9.8, 0.007838902, -0.00140664866, 0.00710141938, 0.211196527, 0.011697528, 0.42982325, 955.9833, 0.011697528, 488.0451, 0.0, 7167)
        msg = msrc.recv_msg()
        if msg is not None:
            print(msg) #Se printean los mensajes recibidos


    sleep(0.0005)
