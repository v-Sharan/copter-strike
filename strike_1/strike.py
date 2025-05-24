from dronekit import connect, VehicleMode
from gimbal_cls import Gimbal
from kamikaze import Kamikaze
import time, csv

current_directory = "C:/Users/vshar/OneDrive/Documents/strike/strike/strike_1"

if __name__ == "__main__":
    vehicle = connect("udpin:172.24.192.1:14555")
    print(vehicle)
    gimbal = Gimbal(host="192.168.6.215")
    strike = Kamikaze(vehicle=vehicle)
    flag = 0
    while True:
        # with open(current_directory + "/flag.csv", "r") as csvfile:
        #     csvreader = csv.reader(csvfile)
        #     for row in csvreader:
        #         flag = int(row[0])
        # if flag != 0:
        #     break
        try:
            tlat, tlon = gimbal.get_target_coords()
            if tlat > 0 and tlon > 0:
                strike.update_target(lat=tlat, lon=tlon)
            if gimbal.tlat > 0 and gimbal.tlon > 0:
                alt = strike.move_to_target()
                if alt <= 20:
                    vehicle.mode = VehicleMode("RTL")
                    time.sleep(0.5)
                    vehicle.mode = VehicleMode("RTL")
                    time.sleep(0.5)
                    vehicle.mode = VehicleMode("RTL")
                    time.sleep(0.5)
                    break
        except KeyboardInterrupt:
            vehicle.close()
            gimbal.stop()
        time.sleep(0.1)

    vehicle.close()
    gimbal.stop()
    print("Connection close")
