import Simulator_class as P
from time import sleep

data=None
#Instances a simulator object
simulator=P.simulator()

#Communications are created with PX4 and flightGear
socket_udp=simulator.socket_udp()
socket_tcp=simulator.socket_TCP_PX4()
#Lookstep
simulator.Inicialice_PX4(socket_tcp)
while data==None:
    simulator.Read_FG()
    data=simulator.Write_PX4(socket_tcp)
#Communication loop
while True:
    simulator.Read_FG()
    simulator.Write_PX4(socket_tcp)
    data=simulator.Read_PX4(socket_tcp)
    if data is not None:
        simulator.Write_FG(socket_udp)
    sleep(0.005)









