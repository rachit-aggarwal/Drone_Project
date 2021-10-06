#!/usr/bin/env python3

#This example shows how to use the follow me plugin
import asyncio
from mavsdk import System
from mavsdk.mission_raw import (MissionItem)
from FollowMe_Implementation import convertcoords


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
            self.cur_local_pos = convertcoords.geodetic2enu(
                position.latitude_deg, position.longitude_deg, position.absolute_altitude_m,
                self.home_pos.latitude_deg, self.home_pos.longitude_deg, self.home_pos.absolute_altitude_m
            )
            return

    async def set_home(self):
        #Set the home of the drone to the current position of the drone
        async for position in self.drone.telemetry.position():
            self.home_pos = position
            print("-- Home position set")
            return


async def print_mission_progress(drone):
    async for mission_progress in drone.mission_raw.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    #Monitors whether the drone is flying or not and returns after landing
    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():

        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            print("-- Landing Detected")
            return


async def run():
    #Create the drone object and connect it to a simulator or offboard
    drone = System()
    await drone.connect(system_address="udp://:14540")

    #Confirm the connection
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    #get the current home

    position_server = Position_Server(drone)

    await position_server.set_home()

    mission_items = []
    mission_items.append(MissionItem(
            0,  # 0th in sequence
            3,  # wgs84 ellipsoid coordinates with relative altitude
            22,  # MAV_CMD_NAV_TAKEOFF
            1,  # current item
            1,  # autocontinue: true
            0,  # 0 pitch ange (not sure what this means in context of takeoff)
            float('nan'),
            float('nan'),
            float('nan'),
            int(position_server.home_pos.latitude_deg * 10 ** 7),   # home latitude
            int(position_server.home_pos.longitude_deg * 10 ** 7),  # home longitude
            25,    # takeoff altitude
            0  # mission type 0
        )
    )

    mission_items.append(MissionItem(
            1,  # first in sequence
            3,  # wgs84 ellipsoid coordinates with relative altitude
            16, # MAV_CMD_NAV_WAYPOINT
            0,  # current item
            1,  # autocontinue: true
            0,  # zero hold time
            2,    # 1m acceptance radius
            2,   # positive 1m for clockwise pass radius
            float('nan'),   # yaw towards next waypoint
            473980398,  # latitude
            85455725,   # longitude
            25,    # relative altitude
            0   # mission type 0
        )
    )

    mission_items.append(MissionItem(
            2,  # 2nd in sequence
            3,  # wgs84 ellipsoid coordinates with relative altitude
            16,  # MAV_CMD_NAV_WAYPOINT
            0,  # current item
            1,  # autocontinue: true
            0,  # zero hold time
            2,  # 1m acceptance radius
            2,  # positive 1m for clockwise pass radius
            float('nan'),  # yaw towards next waypoint
            473980362,  # latitude
            85450146,  # longitude
            25,  # relative altitude
            0  # mission type 0
        )
    )

    mission_items.append(MissionItem(
            3,  # 3rd in sequence
            3,  # wgs84 ellipsoid coordinates with relative altitude
            21,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
            0,  # current item
            1,  # autocontinue: true
            float('nan'),   # empty message
            float('nan'),
            float('nan'),
            float('nan'),
            int(position_server.home_pos.latitude_deg*(10**7)),
            int(position_server.home_pos.longitude_deg*(10**7)),
            int(position_server.home_pos.relative_altitude_m),
            0  # mission type 0
        )
    )

    print("-- Uploading Mission")
    await drone.mission_raw.upload_mission(mission_items)
    print("Success!!")

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission_raw.start_mission()

    #wait until the drone has landed
    await termination_task

if __name__ == "__main__":
    #get path from planner

    #Run the mission
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())