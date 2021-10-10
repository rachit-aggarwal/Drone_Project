import asyncio
import math

from mavsdk import System
from mavsdk.follow_me import (Config, TargetLocation)
import convertcoords

import sys, getopt

default_height = 10.0   # in Meters
follow_distance = 1.0   # in Meters, this is the distance that the drone will remain away from Target while following it
target_speed = 3
direction = Config.FollowDirection.BEHIND
responsiveness = 0.02

#options = "aenu:"
#long_options = ["Address", "East", "North", "Up"]


class Position_Server:
    #Get the current position of the drone
    def __init__(self, drone):
        self.drone = drone

        #Last known position of the drone
        self.cur_pos = []

        #Last known local position
        self.cur_local_pos = []

        #Home position
        self.home_pos = []

    async def update_position(self):
        #Run to update the current position to the latest telemetry
        async for position in self.drone.telemetry.position():
            self.cur_pos = position
            self.cur_local_pos = convertcoords.geodetic2enu(position.latitude_deg,
                                                            position.longitude_deg,
                                                            position.absolute_altitude_m,
                                                            self.home_pos.latitude_deg,
                                                            self.home_pos.longitude_deg,
                                                            self.home_pos.absolute_altitude_m
                                                            )
            return

    async def set_home(self):
        #Set the home of the drone to the current position of the drone
        async for position in self.drone.telemetry.position():
            self.home_pos = position
            print("-- Home position set")
            return


def check_goal(position, goal, v_x, v_y):
    # Has the drone crossed a line perpendicular to the target position and velocity
    v_norm = math.sqrt(v_x**2 + v_y**2)

    if v_norm == 0:
        cond = math.sqrt((position[0]-goal[0])**2 + (position[1]-goal[1])**2)
        if cond < 0.5:
            return False
        return True

    v = [v_x/v_norm, v_y/v_norm]
    cond = v[0]*(goal[0]-position[0]) + v[1]*(goal[1]-position[1])
    if cond > 0:
        return False
    return True


def get_path(home):
    path = []
    local = []
    in_path = [[0, 2], [0, 4], [0, 6], [0, 8], [2, 8]]

    for i in range(len(in_path)):
        local.append([in_path[i][0], in_path[i][1], default_height])
        geo = convertcoords.enu2geodetic(in_path[i][0],
                                         in_path[i][1],
                                         default_height + home.absolute_altitude_m,
                                         home.latitude_deg,
                                         home.longitude_deg,
                                         home.absolute_altitude_m)
        path.append([geo[0], geo[1], geo[2], 0, 0])

    for i in range(len(path)-1):
        direc = [local[i+1][0] - local[i][0], local[i+1][1] - local[i][1]]
        mag = math.sqrt(direc[0]**2 + direc[1]**2)
        vel = [(direc[0]/mag)*target_speed, (direc[1]/mag)*target_speed]
        path[i][3] = vel[0]
        path[i][4] = vel[1]

    return path


async def fly_drone():
    drone = System()
    print("Attempting Connection")
    await drone.connect(system_address="udp://:14540")
    # await drone.connect(system_address="serial:///dev/ttyUSB0:921600")


    #This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    #Checking if Global Position Estimate is ok
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok and global_lock.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    position_server = Position_Server(drone)
    await position_server.set_home()

    #Build the path
    path = get_path(position_server.home_pos)

    #Arming the drone
    print ("-- Arming")
    await drone.action.arm()

    #Follow me Mode requires some configuration to be done before starting the mode
    conf = Config(default_height, follow_distance, direction, responsiveness)
    await drone.follow_me.set_config(conf)

    print ("-- Taking Off")
    await drone.action.takeoff()
    await asyncio.sleep(8)
    print ("-- Starting Follow Me Mode")
    await drone.follow_me.start()
    await asyncio.sleep(8)

    for latitude, longitude, altitude, v_x, v_y in path:
        #Create the target object and convert the coordinates into the local frame
        target = TargetLocation(latitude, longitude, altitude, v_x, v_y, 0)
        local_target = convertcoords.geodetic2enu(latitude,
                                                  longitude,
                                                  altitude + position_server.home_pos.absolute_altitude_m,
                                                  position_server.home_pos.latitude_deg,
                                                  position_server.home_pos.longitude_deg,
                                                  position_server.home_pos.absolute_altitude_m)
        #Start the drone following the target
        await drone.follow_me.set_target_location(target)
        print(local_target)
        #Follow the current target until the condition is met to continue to the next point
        following = True
        while following:
            await position_server.update_position()
            current = position_server.cur_local_pos
            following = check_goal(current, local_target, v_x, v_y)



    #Stopping the follow me mode
    print ("-- Stopping Follow Me Mode")
    await drone.follow_me.stop()
    await asyncio.sleep(5)

    print ("-- Landing")
    await drone.action.land()

if __name__ == "__main__":

#    argls = sys.argv[1:]
#    try:
#        arguments, values = getopt.getopt(argls, options, long_options)
#        for currarg, currval in arguments:
#            if

    loop = asyncio.get_event_loop()
    loop.run_until_complete(fly_drone())
