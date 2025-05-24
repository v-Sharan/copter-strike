import time
from math import sin, cos, sqrt, atan2, radians, degrees, atan
from dronekit import Vehicle,connect
from pymavlink import mavutil
import pid

class Kamikaze:
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.tlat = 0
        self.tlon = 0

        self.dist_to_vel = 0.30  ##0.15 m/s  ###

        self.descent_radius = 1  # in m/s
        self.last_set_velocity = 0
        self.vel_update_rate = 0.1  ##sec
        self.descent_rate = 0.3  # 0.5
        self.radius_of_earth = 6378100.0  # in meters

        self.dat = 1.0
        xy_p = 1.0
        xy_i = 0.0
        xy_d = 0.0
        xy_imax = 10.0
        self.vel_xy_pid = pid.pid(xy_p, xy_i, xy_d, radians(xy_imax))

        self.vel_speed_min = 20
        self.vel_speed_max = 20
        self.vel_accel = 0.3

    def condition_yaw(self, heading: float, relative: bool = False) -> None:

        is_relative = 1 if relative else 0

        msg = self.vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            0,
            1,
            is_relative,
            0,
            0,
            0,
        )

        self.vehicle.send_mavlink(msg)

    def shift_to_origin(
        self, pt: list[float], width: float, height: float
    ) -> None:
        return ((pt[0] - width / 2.0), (-1 * pt[1] + height / 2.0))

    def set_velocity(
        self,
        velocity_x: float,
        velocity_y: float,
        velocity_z: float,
        yaw_angle: float,
    ) -> None:
        if (time.time() - 0) > self.vel_update_rate:
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0x01C7,
                0,
                0,
                0,
                velocity_x,
                velocity_y,
                velocity_z,
                0,
                0,
                0,
                yaw_angle,
                0,
            )
            # send command to vehicle
            self.vehicle.send_mavlink(msg)

    def get_ef_velocity_vector(
        self, pitch: float, yaw: float, speed: float
    ) -> tuple[float]:
        cos_pitch = cos(pitch)
        x = speed * cos(yaw) * cos_pitch
        y = speed * sin(yaw) * cos_pitch
        z = speed * sin(pitch)
        return x, y, z

    def gps_distance(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """return distance between two points in meters,
        coordinates are in degrees
        thanks to http://www.movable-type.co.uk/scripts/latlong.html"""

        lat1 = radians(lat1)
        lat2 = radians(lat2)
        lon1 = radians(lon1)
        lon2 = radians(lon2)
        dLat = lat2 - lat1
        dLon = lon2 - lon1

        a = sin(0.5 * dLat) ** 2 + sin(0.5 * dLon) ** 2 * cos(lat1) * cos(lat2)
        c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a))
        # print ("...c", c)
        return self.radius_of_earth * c

    def gps_bearing(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """return bearing between two points in degrees, in range 0-360
        thanks to http://www.movable-type.co.uk/scripts/latlong.html"""

        lat1 = radians(lat1)
        lat2 = radians(lat2)
        lon1 = radians(lon1)
        lon2 = radians(lon2)
        dLat = lat2 - lat1
        dLon = lon2 - lon1
        y = sin(dLon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
        bearing = degrees(atan2(y, x))
        if bearing < 0:
            bearing += 360.0
        return bearing

    def update_target(self, lat: float, lon: float) -> None:
        self.tlat = lat
        self.tlon = lon

    def move_to_target(self) -> float:

        lat1 = self.vehicle.location.global_frame.lat
        lon1 = self.vehicle.location.global_frame.lon
        
        Tdist = self.gps_distance(lat1, lon1, self.tlat, self.tlon)

        Tbearing = self.gps_bearing(lat1, lon1, self.tlat, self.tlon)
        alt = self.vehicle.location.global_relative_frame.alt

        pitch_angle = degrees(atan(Tdist / alt))
        pitch_angle = abs(90 - pitch_angle)

        pitch_angle, yaw_angle = pitch_angle, Tbearing

        pitch_final = radians(pitch_angle)
        yaw_final = radians(yaw_angle)

        speed = Tdist * self.dist_to_vel

        dt = self.vel_xy_pid.get_dt(1.5)

        speed = min(speed, self.vel_speed_max)
        speed = max(speed, self.vel_speed_min)

        speed_chg_max = self.vel_accel * dt

        speed = min(speed, self.dat + speed_chg_max)
        speed = max(speed, self.dat - speed_chg_max)

        self.dat = speed

        guided_target_vel = self.get_ef_velocity_vector(pitch_final, yaw_final, speed)
        self.set_velocity(
            guided_target_vel[0],
            guided_target_vel[1],
            guided_target_vel[2],
            yaw_final,
        )
        return speed



if __name__ == "__main__":
    vehicle = connect("172.24.192.1:14552")
    print(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon)
