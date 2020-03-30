# import socket as s
# import requests
#
# # import ast
# #
# #INSTANCIA DE LAS ACCIONES DE CONTROL
# Aleron=-0.0
# Acelerador=+0.0
# Timon=-0.0
# Elevador=-0.0
# Angulo_imagen=0
# #
# #
# #INSTANCIACION DE LAS COMUNICACIONESp
# UDP_IP="localhost"
# UDP_PORT_recibir=9999
# UDP_PORT_enviar=9900
# sock=s.socket(s.AF_INET,s.SOCK_DGRAM)
# sock.bind((UDP_IP, UDP_PORT_recibir))
# #
# # LECTURA DEL SIMULADOR
# data, addr = sock.recvfrom(2048)
# data = data.decode("utf-8",errors='ignore')
# data = data.split(';')
# print(data,'datos')
#
# # # ENVIO DE LAS ACCIONES DE CONTROL
# # Acciones_Control = [Aleron, ',', Elevador, ',', Timon, ',', Acelerador, '\n']
# # mensaje = ''.join(str(e) for e in Acciones_Control)
# # Enviar = str.encode(mensaje)
# # sock.sendto(Enviar, (UDP_IP, UDP_PORT_enviar))


# data=ast.literal_eval('0x0005a1dfe31cb2a8')
# # data1=ast.literal_eval('0x0005a1b27289aff0')
# #
# print(data)
# print(data1)
# print((data1-data),'diferencia')

