import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.all_waypoints = self.calculate_box()
        self.current_waypoint_index = -1
        self.current_waypoint = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF or self.flight_state == States.WAYPOINT:

            if self.flight_state == States.TAKEOFF:
                altitude = -1.0 * self.local_position[2]
                if altitude > 0.95 * self.all_waypoints[0][2]:
                    self.waypoint_transition()
            elif self.flight_state == States.WAYPOINT:
                if abs(self.local_position[0] - self.current_waypoint[0]) + abs(self.local_position[1] - self.current_waypoint[1]) < 0.1:
                    if self.current_waypoint_index < 4:
                        self.waypoint_transition()
                    else:
                        self.landing_transition()


    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

    def calculate_box(self):
        waypoints = [[0.0, 0.0, 5.0],
        [10.0, 0.0, 5.0],
        [10.0, 10.0, 5.0],
        [0.0, 10.0, 5.0],
        [0.0, 0.0, 5.0]]

        return waypoints


    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()

        self.set_home_position(
            self.global_position[0],
            self.global_position[1],
            self.global_position[2]
        )

        self.flight_state = States.ARMING


    def takeoff_transition(self):
        print("takeoff transition")
        self.current_waypoint_index = 0
        self.current_waypoint = self.all_waypoints[0]
        target_altitude = self.all_waypoints[0][2]
        self.takeoff(target_altitude)

        self.flight_state = States.TAKEOFF


    def waypoint_transition(self):
        print("waypoint transition")
        self.current_waypoint_index += 1
        self.current_waypoint = self.all_waypoints[self.current_waypoint_index]

        self.cmd_position(
            self.all_waypoints[self.current_waypoint_index][0],
            self.all_waypoints[self.current_waypoint_index][1],
            self.all_waypoints[self.current_waypoint_index][2],
            0.0
        )

        self.flight_state = States.WAYPOINT


    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING


    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING


    def manual_transition(self):
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL


    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
