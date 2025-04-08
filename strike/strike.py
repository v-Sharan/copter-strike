from dronekit import connect, VehicleMode
from gimbal_cls import Gimbal
from kamikaze import Kamikaze
import time, os, csv
from target import Target

current_directory = ""

if __name__ == "__main__":
    vehicle = connect("udpin:192.168.6.145:14551", heartbeat_timeout=3)
    print(vehicle)
    # gimbal = Gimbal(host="192.168.6.213")
    strike = Kamikaze(vehicle=vehicle)
    target = Target()
    tlat, tlon = target.return_coords()
    while True:
        try:
            tlat, tlon = target.return_coords()
            print(tlat, tlon)
            # if tlat > 0 and tlon > 0:
            strike.update_target(lat=tlat, lon=tlon)
            # if gimbal.tlat > 0 and gimbal.tlon > 0:
            alt = strike.move_to_target()
            # if alt <= 0:
            #     vehicle.mode = VehicleMode("RTL")
            #     time.sleep(0.5)
            #     vehicle.mode = VehicleMode("RTL")
            #     time.sleep(0.5)
            #     vehicle.mode = VehicleMode("RTL")
            #     time.sleep(0.5)
            #     break
        except KeyboardInterrupt:
            vehicle.close()
            # gimbal.stop()
        time.sleep(0.1)

    # vehicle.close()
    # gimbal.stop()
    # print("Connection close")
