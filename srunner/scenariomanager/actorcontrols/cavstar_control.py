#!/usr/bin/env python

# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an example controller for actors, that use an external
software for longitudinal and lateral control command calculation.
Examples for external controls are: Autoware, CARLA manual_control, etc.

This module is not intended for modification.
"""

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
import cavstar.integration as CAVstar
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
import sys

class CavstarControl(BasicControl):

    """
    Actor control class for actors, with externally implemented longitudinal and
    lateral controlers (e.g. Autoware).

    Args:
        actor (carla.Actor): Actor that should be controlled by the agent.
    """

    def __init__(self, actor, args=None):
        super(CavstarControl, self).__init__(actor)

        self.client = CarlaDataProvider.get_client()
        self.world = CarlaDataProvider.get_world()
        self.the_map = CarlaDataProvider.get_map()
        (viewer,) = CAVstar.setup_cavstar(self.client, self.world, self.the_map, actor)
        
        self.viewer = viewer
        # print(actor.get_location())
        sensors = CAVstar.SetupSensors(self.client, self.world, actor)
        self.sensors = sensors
        CAVstar.wait_for_start(self.world, actor)

        
        print(args)
        # if args in args:
        #     # self._speeds = ast.literal_eval(args['speeds'])
        #     print(arg)
            
    def __del__(self):
        print("Destroying sensors")
        if self.sensors.gnss is not None:
            self.sensors.gnss.destroy()
        if self.sensors.imu is not None:
            self.sensors.imu.destroy()
        if self.sensors.lid is not None:
            self.sensors.lid.destroy()
        if self.sensors.ccam is not None:
            self.sensors.ccam.destroy()
        if self.sensors.radar is not None:
            self.sensors.radar.destroy()

    def reset(self):
        """
        Reset the controller
        """
        if self._actor and self._actor.is_alive:
            self._actor = None

    def run_step(self):
        """
        The control loop and setting the actor controls is implemented externally.
        """
        CAVstar.run_step(self.world, self.the_map, self.viewer, self._actor)
        pass
