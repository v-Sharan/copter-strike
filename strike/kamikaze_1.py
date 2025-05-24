#!/usr/local/bin/python
# shell script not need
import time
from math import sin, cos, sqrt, atan2, radians
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from math import sqrt
import time
from multiprocessing import Queue
import math
import pid


R = 6373.0
flag = 0

# Global Variables and Flags
vehicle = None
ignore_target = True
tangential_speed = 50  # cm/s
# circle_period = sys.maxint
home_location = None
last_centering_time = 0


shell_commands = Queue()
last_image_location = Queue(maxsize=1)


camera_width = 1920
camera_height = 1080
camera_vfov = 33.9234  # 51.9784
camera_hfov = 56.07  # 72.5845

dist_to_vel = 0.15  ##0.15 m/s  ###

descent_radius = 1  # in m/s
last_set_velocity = 0
vel_update_rate = 0.1  ##sec
descent_rate = 0.3  # 0.5

vel_speed_min = 19
vel_speed_max = 29  # default 4  .........change this parameter to adjust decent speed

vel_accel = 0.3  # 0.5 - maximum acceleration in m/s/s


# Simulator flag
SIM = False

update_rate = 0.01
rcCMD = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]

vehicle = connect("udpin:172.24.192.1:14552", wait_ready=True)
print(vehicle)
# ....uav...inital pos.....
lat1 = vehicle.location.global_frame.lat
lon1 = vehicle.location.global_frame.lon
# ..strike..target.....pos
"""
lat2 = 12.948080
lon2 = 80.142066
"""
"""
lat2 = 12.947395
lon2 = 80.138257
"""

radius_of_earth = 6378100.0  # in meters


dat = 0  # ..vel_speed_last


# horizontal velocity pid controller.  maximum effect is 10 degree lean
xy_p = 1.0
xy_i = 0.0
xy_d = 0.0
xy_imax = 10.0
vel_xy_pid = pid.pid(xy_p, xy_i, xy_d, math.radians(xy_imax))


def condition_yaw(heading, relative=False):
    global vehicle

    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0,
        0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0,
        0,
        0,
    )  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def shift_to_origin(pt, width, height):
    # print ("pt", pt)
    # print ("width", width)
    # print ("height", height)

    return ((pt[0] - width / 2.0), (-1 * pt[1] + height / 2.0))


def pixel_point_to_position_xy(pixel_position, distance):
    thetaX = pixel_position[0] * camera_hfov / camera_width
    # print ("thetaX", thetaX)

    thetaY = pixel_position[1] * camera_vfov / camera_height
    # print ("thetaY", thetaY)
    x = distance * math.tan(math.radians(thetaX))
    y = distance * math.tan(math.radians(thetaY))
    # print ("x,y", (x,y))

    return (x, y)


def set_velocity(velocity_x, velocity_y, velocity_z, yaw_angle):
    # only let commands through at 10hz
    if (time.time() - 0) > vel_update_rate:
        last_set_velocity = time.time()
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
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
        vehicle.send_mavlink(msg)


def get_ef_velocity_vector(pitch, yaw, speed):
    cos_pitch = math.cos(pitch)
    x = speed * math.cos(yaw) * cos_pitch
    y = speed * math.sin(yaw) * cos_pitch
    z = speed * math.sin(pitch)
    return x, y, z


def gps_distance(lat1, lon1, lat2, lon2):
    """return distance between two points in meters,
    coordinates are in degrees
    thanks to http://www.movable-type.co.uk/scripts/latlong.html"""
    from math import radians, cos, sin, sqrt, atan2

    lat1 = radians(lat1)
    lat2 = radians(lat2)
    lon1 = radians(lon1)
    lon2 = radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = sin(0.5 * dLat) ** 2 + sin(0.5 * dLon) ** 2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a))
    # print ("...c", c)
    return radius_of_earth * c


def gps_bearing(lat1, lon1, lat2, lon2):
    """return bearing between two points in degrees, in range 0-360
    thanks to http://www.movable-type.co.uk/scripts/latlong.html"""
    from math import sin, cos, atan2, radians, degrees

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


def condition_yaw(heading, relative=False):

    is_relative = 1 if relative else 0

    msg = vehicle.message_factory.command_long_encode(
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

    vehicle.send_mavlink(msg)


def move_to_target(lat, lon):

    lat1 = vehicle.location.global_frame.lat
    lon1 = vehicle.location.global_frame.lon
    global lat2, lon2, dat, flag

    lat2 = lat
    lon2 = lon

    Tdist = gps_distance(lat1, lon1, lat2, lon2)

    Tbearing = gps_bearing(lat1, lon1, lat2, lon2)
    alt = vehicle.location.global_relative_frame.alt

    pitch_angle = math.degrees(math.atan(Tdist / alt))
    pitch_angle = abs(90 - int(pitch_angle))

    vz = descent_rate

    pitch_angle, yaw_angle = int(pitch_angle), int(Tbearing)

    pitch_final = math.radians(pitch_angle)
    yaw_final = math.radians(yaw_angle)

    speed = Tdist * dist_to_vel

    dt = vel_xy_pid.get_dt(2.0)

    speed = min(speed, vel_speed_max)
    speed = max(speed, vel_speed_min)

    speed_chg_max = vel_accel * dt

    speed = min(speed, dat + speed_chg_max)
    speed = max(speed, dat - speed_chg_max)

    dat = speed

    guided_target_vel = get_ef_velocity_vector(pitch_final, yaw_final, speed)
    print("....123.....qw...", speed, abs(alt))
    set_velocity(
        guided_target_vel[0], guided_target_vel[1], guided_target_vel[2], yaw_final
    )

    # if alt < 30:
    #     vehicle.mode = VehicleMode("RTL")
    #     time.sleep(0.5)
    #     vehicle.mode = VehicleMode("RTL")
    #     time.sleep(0.5)

    #     vehicle.mode = VehicleMode("RTL")
    #     time.sleep(1)


def geo_loop(latitude, longitude, altitude):
    global vehicle
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    alt = abs(alt)

    target = LocationGlobalRelative(latitude, longitude, 0)

    min_distance = 200

    start = time.time()

    while True:
        # current = time.time() - start

        lat1 = radians(vehicle.location.global_frame.lat)
        lon1 = radians(vehicle.location.global_frame.lon)
        lat2 = radians(target.lat)
        lon2 = radians(target.lon)
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        distance = R * c
        distance = distance * 1000

        output = (
            "dist b/w vehicle to geo_search and vehicle lat & lon:",
            (distance, lat, lon),
        )
        output = str(output)

        if distance <= min_distance:
            break
        time.sleep(0.5)


def main(t_lat, t_lon):
    global flag
    vehicle.mode = "GUIDED"
    time.sleep(1)
    while True:
        if not flag:
            move_to_target(t_lat, t_lon)


if __name__ == "__main__":
    tlat = 13.3898388
    tlon = 80.2309978
    main(t_lat=tlat,t_lon=tlon)