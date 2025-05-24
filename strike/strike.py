from dronekit import connect, VehicleMode
from gimbal_cls import Gimbal
from kamikaze import Kamikaze
import time

if __name__ == "__main__":
    vehicle = connect("udpin:172.24.192.1:14552", heartbeat_timeout=3)
    # vehicle = connect("udpin:192.168.6.145:14555", heartbeat_timeout=3)
    print(vehicle)
    gimbal = Gimbal(host="192.168.6.119")
    strike = Kamikaze(vehicle=vehicle)
    alt =30
    while True:
        try:
            alt = vehicle.location.global_relative_frame.alt
            tlat,tlon = gimbal.get_target_coords()
            # print(tlat,tlon)
            if tlat > 0 and tlon > 0:
                strike.update_target(lat=tlat, lon=tlon)
                speed = strike.move_to_target()
                print(f"Altitude: {alt:.1f}m, Speed: {speed:.1f}m/s", end="\r")
            # if alt <= 15:
            #     vehicle.mode = VehicleMode("RTL")
            #     time.sleep(0.5)
            #     vehicle.mode = VehicleMode("RTL")
            #     time.sleep(0.5)
            #     vehicle.mode = VehicleMode("RTL")
            #     time.sleep(0.5)
            #     break
            if(alt == 5):
                break
        except KeyboardInterrupt:
            vehicle.close()
            gimbal.stop()
        time.sleep(0.01)
