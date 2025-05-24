import socket
import time

import binascii, socket, time
from functools import lru_cache
import threading


class Gimbal:
    """
    Class for Gimbal Connection and Calculation

    params:
        host: string
        port: int
    """
    def __init__(self, host, port=2000):
        self.host = host
        self.port = port
        self.tlat = 13.3898388
        self.tlon = 80.2309978
        self.connected = False
        self.bearing = 0
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        packet = bytearray(
            [
                0xEB,
                0x90,
                0x07,
                0x55,
                0xAA,
                0xDC,
                0x04,
                0x10,
                0x00,
                0x14,
                0x3,
            ]
        )
        self.bytePacket = bytes(packet)
        # self.connect_to_gimbal()
        # self.thread = threading.Thread(target=self.calculate_coords, daemon=True)
        # self.thread.start()
        self.thread1 = threading.Thread(target=self.change_target,daemon=True)
        self.thread1.start()
        
    def change_target(self):
        latlons = [(13.3898388,80.2309978),(13.3907938,80.2308476),(13.3915348,80.2307779)]
        for lat,lon in latlons:
            self.tlat = lat
            self.tlon = lon
            print(f"Target: {self.tlat}, {self.tlon}")
            time.sleep(30)

    def calculate_coords(self):
        while self.connected:
            self.socket.sendall(self.bytePacket)
            if not self.is_connected():
                print("Not socket connected")
                break
            data, address = self.socket.recvfrom(1024)
            self.get_latlon(data)

    def is_connected(self):
        try:
            self.socket.send(b"")
            self.connected = True
        except socket.error:
            self.connected = False
            return False
        return True

    def connect_to_gimbal(self):
        try:
            self.socket.connect((self.host, self.port))
            time.sleep(1)
            print("Connected to gimbal")
            self.connected = True
        except socket.error as e:
            print(f"Connection failed: {e}")
            self.connected = False

    def stop(self):
        if self.thread.is_alive():
            self.thread.join()
            print("Thread is stoped")
        self.connected = False
        self.socket.close()
        print("socket is closed")

    @lru_cache(maxsize=None)
    def hex_to_signed_int(self, hex_str):
        """
        Converts a little-endian hexadecimal string to a signed integer.
        """
        value = int(hex_str, 16)
        if value >= 2 ** (len(hex_str) * 4 - 1):
            value -= 2 ** (len(hex_str) * 4)
        return value

    def extract_imu_angle(self, data):
        """
        Extracts and calculates the IMU angles from the provided data string.
        """
        if len(data) in [94]:
            tlat = data[34:42]
            tlon = data[42:50]
            tlat = self.hex_to_signed_int(tlat)
            tlon = self.hex_to_signed_int(tlon)

            return tlat / 1e7, tlon / 1e7

    def get_latlon(self, data_1):
        response_hex = binascii.hexlify(data_1).decode("utf-8")
        if len(response_hex) in [94]:
            tlat, tlon = self.extract_imu_angle(response_hex)
            self.tlat = tlat
            self.tlon = tlon

    def get_target_coords(self) -> tuple[float, float]:
        """
        Returns the Target Latitude and Longitude of the Gimbal.
        """
        return self.tlat, self.tlon

if __name__ == "__main__":
    gimbal = Gimbal(host="192.168.6.119")
    while True:
        if(gimbal.is_connected):
            print(gimbal.get_target_coords())
        else:
            print(gimbal.is_connected())    
        time.sleep(0.1)
            