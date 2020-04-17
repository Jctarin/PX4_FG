import Prueba as P
from time import sleep

data=None
#Instancia un objeto simulador
simulador=P.simulador()

#Se crean las comunicaciones con PX4 y flightGear
socket_udp=simulador.socket_udp()
socket_tcp=simulador.socket_TCP_PX4()
# #Lookstep
simulador.Inicializar_PX4(socket_tcp)
while data==None:
    simulador.Leer_FG()
    data=simulador.Escribe_PX4(socket_tcp)
#Bucle de comunicacion
while True: #Mismo porceso anterior pero no inicializa la comunicacion con PX4
    simulador.Leer_FG()
    simulador.Escribe_PX4(socket_tcp)
    data=simulador.Leer_PX4(socket_tcp)
    if data is not None:
        simulador.Escribe_FG(socket_udp)
    sleep(0.005)









