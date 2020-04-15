from pymavlink.dialects.v20 import ardupilotmega
from pymavlink import mavutil
from time import sleep
import socket

msrc = mavutil.mavlink_connection('tcpin:localhost:4560', planner_format=False,
                                  robust_parsing=True,source_system=0)

flag=0
bandera=0
n=0
while True:
    if flag==0:
        msg = msrc.recv_msg()
        if msg is not None:
           print(msg)
           bandera=bandera+1
           if bandera==2:
               msrc.mav.heartbeat_send(0, 0, 0, 0, 0, mavlink_version=3)
               msrc.mav.hil_sensor_send(1585357843637000, 0.3, 0.02, -9.8, 0.007838902, -0.00140664866, 0.00710141938, 0.211196527, 0.011697528, 0.42982325, 955.9833, 0.011697528, 488.0451, 0.0, 7167)
               msrc.mav.system_time_send(1585357844074000, 514911413)
               flag=1
    else:
        n+=5000
        msrc.mav.hil_sensor_send(1585178126095000+n, 0.3, 0.02, -9.8, 0.007838902, -0.00140664866, 0.00710141938, 0.211196527, 0.011697528, 0.42982325, 955.9833, 0.011697528, 488.0451, 0.0, 7167)
        # msrc.mav.hil_gps_send(1585178126095000+n, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, force_mavlink1=False)
        msg = msrc.recv_msg()
        if msg is not None:
            print(msg)


    sleep(0.0005)


#t1 -->fd 40 00 00 26 01 33 6b0000 50a08972b2a10500 91c8913d798bd2bcd80e1dc198f32f3c755f8abc29120c3cc146643e34201e3cd496d63e3bff6e4400000000b803f44300000000ff1b00005e0b
#t2 -->fd 40 00 00 27 01 33 6b0000 f0af8972b2a10500 b5e8adbd3b9f36bcb1b21dc1cb1662bccb1d47bc30d38bbcef19613edaedf73b7da4df3e7eff6e4400000000c3fef34300000000ff1b0000fd27
#t3'-->fd 40 00 00 28 01 33 730000 f0af8972b2a10500 0000803f0000000000000000000000000000000000000000000000004b52401c44f417050000000000000000000000002600000000000000aeb6
#t3 -->fd 40 00 00 29 01 33 6b0000 90bf8972b2a10500 450d2bbdbc69aabcdce31bc11792783cf9d119bb3f5937bc9a38623edbd7b13b476dda3ec7ff6e4400000000c5f9f34300000000ff1b0000a605

#Tsys -->fd 0c 00 00 03 01 33 020000 98921c0bb6a10500 9fa6fa13 2484

#Mi -->fd 40 00 00 26 00 00 6b0000 05a108a906832d00 9a99993e0ad7a33ccdcc1cc1bd6e003c4c5fb8ba06b3e83be743583efda63f3ccb11dc3eeefe6e44fda63f3cc605f443000000000000ff1b29d1