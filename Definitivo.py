import Puente as P
from time import sleep

data=None
#Instancia un objeto simulador
simulador=P.simulador()

#Se crean las comunicaciones con PX4 y flightGear
socket_udp=simulador.socket_udp()
print('Paso_1')
socket_tcp=simulador.socket_TCP_PX4()
print('Paso_2')
#Lookstep
simulador.Inicializar_PX4(socket_tcp)
while data==None:
    simulador.Leer_FG()
    data=simulador.Escribe_PX4(socket_tcp)
    print(data)
    sleep(0.0005)
#Bucle de comunicacion
while True: #Mismo porceso anterior pero no inicializa la comunicacion con PX4
    simulador.Leer_FG()
    simulador.Escribe_PX4(socket_tcp)
    simulador.Leer_PX4(socket_tcp) #NO puede ir separado de la recepcion, demasiado rapid, debe enviar y actualizar al instante
    simulador.Escribe_FG(socket_udp)
    sleep(0.0005)









