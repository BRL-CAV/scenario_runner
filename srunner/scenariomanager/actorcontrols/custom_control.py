#!/usr/bin/env python

# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an example control for vehicles
"""

import math
import ast
import carla
from agents.navigation.basic_agent import LocalPlanner
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.actorcontrols.basic_control import BasicControl


class CustomControl(BasicControl):

    """
    Controller class for vehicles derived from BasicControl.

    The controller makes use of the LocalPlanner implemented in CARLA.

    Args:
        actor (carla.Actor): Vehicle actor that should be controlled.
    """

    _args = {'K_P': 1.0, 'K_D': 0.01, 'K_I': 0.0, 'dt': 0.05}

    def __init__(self, actor, args=None):
        super(CustomControl, self).__init__(actor)

        self._speeds = None
        self._amountWaypoints = 0
        self._nextWaypoint = None
        self._currentWaypoint = None

        self._local_planner = LocalPlanner(  # pylint: disable=undefined-variable
            self._actor, opt_dict={
                'target_speed': self._target_speed * 3.6,
                'lateral_control_dict': self._args})
        
        if args and 'speeds' in args:
            self._speeds = ast.literal_eval(args['speeds'])
            print(self._speeds)
            

        if self._waypoints:
            self._update_plan()

    def _update_plan(self):
        """
        Update the plan (waypoint list) of the LocalPlanner
        """
        print("Updating waypoint plan!")
        plan = []
        for transform in self._waypoints:
            waypoint = CarlaDataProvider.get_map().get_waypoint(
                transform.location, project_to_road=True, lane_type=carla.LaneType.Any)
            plan.append((waypoint, RoadOption.LANEFOLLOW))

        self._local_planner.set_global_plan(plan)
        self._amountWaypoints = len(self._local_planner._waypoint_buffer) + len(self._local_planner._waypoints_queue)
        

    def _update_offset(self):
        """
        Update the plan (waypoint list) of the LocalPlanner
        """
        self._local_planner._vehicle_controller._lat_controller._offset = self._offset   # pylint: disable=protected-access

    def reset(self):
        """
        Reset the controller
        """
        if self._actor and self._actor.is_alive:
            if self._local_planner:
                self._local_planner.reset_vehicle()
                self._local_planner = None
            self._actor = None

    def run_step(self):
        """
        Execute on tick of the controller's control loop

        Note: Negative target speeds are not yet supported.
              Try using simple_vehicle_control or vehicle_longitudinal_control.

        If _waypoints are provided, the vehicle moves towards the next waypoint
        with the given _target_speed, until reaching the final waypoint. Upon reaching
        the final waypoint, _reached_goal is set to True.

        If _waypoints is empty, the vehicle moves in its current direction with
        the given _target_speed.

        If _init_speed is True, the control command is post-processed to ensure that
        the initial actor velocity is maintained independent of physics.
        """
        self._reached_goal = False

        if self._waypoints_updated:
            self._waypoints_updated = False
            self._update_plan()
            #print('Waypoints updated')

            # #print('Buffer:')
            # for elm in self._local_planner._waypoint_buffer:
            #     print(elm[0])

            # #print('Queue:')
            # for elm in self._local_planner._waypoints_queue:
            #     print(elm[0])

        if self._offset_updated:
            self._offset_updated = False
            self._update_offset()


        if self._amountWaypoints > 0:
            self._currentWaypoint = (self._amountWaypoints) - (len(self._local_planner._waypoint_buffer) + len(self._local_planner._waypoints_queue))
            self._nextWaypoint = self._currentWaypoint + 1
        

        #print("Current waypoint: %s" % self._currentWaypoint)
        #print("Next waypoint: %s" % self._nextWaypoint)
        #print("Current Speed: %s" % (math.sqrt(self._actor.get_velocity().x**2 + self._actor.get_velocity().y**2)))

        if str(self._currentWaypoint) in self._speeds.keys():
            changed_speed = float(self._speeds[str(self._currentWaypoint)])
            #print(" Target speed: %i, change by: %i" % (self._target_speed, changed_speed))
            self._target_speed = self._target_speed + changed_speed
            self._speeds.pop(str(self._currentWaypoint), None)

            
            
        target_speed = self._target_speed

        # If target speed is negavite, raise an exception
        if target_speed < 0:
            raise NotImplementedError("Negative target speeds are not yet supported")

        self._local_planner.set_speed(target_speed * 3.6)
        control = self._local_planner.run_step(debug=False)

        # Check if the actor reached the end of the plan
        if self._local_planner.done():
            self._reached_goal = True

        self._actor.apply_control(control)

        if self._init_speed:
            current_speed = math.sqrt(self._actor.get_velocity().x**2 + self._actor.get_velocity().y**2)

            # If _init_speed is set, and the PID controller is not yet up to the point to take over,
            # we manually set the vehicle to drive with the correct velocity
            if abs(target_speed - current_speed) > 3:
                yaw = self._actor.get_transform().rotation.yaw * (math.pi / 180)
                vx = math.cos(yaw) * target_speed
                vy = math.sin(yaw) * target_speed
                self._actor.set_target_velocity(carla.Vector3D(vx, vy, 0))
