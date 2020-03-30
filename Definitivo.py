import Prueba as P
from time import sleep

n=0
#Instancia un objeto simulador
simulador=P.simulador()

#Se crean las comunicaciones con PX4 y flightGear
socket_udp=simulador.socket_udp()
socket_tcp=simulador.socket_TCP_PX4()

#Lookstep
simulador.Actualizar_valores_desde_upd(socket_udp) #Lee FG por primera vez
simulador.Inicializar_PX4(socket_tcp) #Envia los primeros comandos para establecer comunicacion con PX4 y Envia los valores de los sensores a PX4


#Bucle de comunicacion
while True: #Mismo porceso anterior pero no inicializa la comunicacion con PX4
    simulador.Actualizar_valores_desde_upd(socket_udp)
    simulador.Actualizar_PX4(socket_tcp) #NO puede ir separado de la recepcion, demasiado rapid, debe enviar y actualizar al instante
    sleep(1)









