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

import carla
import math

import numpy

from agents.navigation.basic_agent import LocalPlanner
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

class NewController(BasicControl):

    """
    Actor control class for actors, with externally implemented longitudinal and
    lateral controlers (e.g. Autoware).

    Args:
        actor (carla.Actor): Actor that should be controlled by the agent.
        
    """

    _args = {'K_P': 1.0, 'K_D': 0.01, 'K_I': 0.0, 'dt': 0.05}




    def __init__(self, actor, args=None):
        super(NewController, self).__init__(actor)
        
        self.distance_threshold = 10
        
        self._obstacle_sensor = None
        self._obstacle_distance = float('inf')
        self._obstacle_actor = None

        self._tracking_actor = None

        self.world = CarlaDataProvider.get_world()
        self.map = self.world.get_map()

        self.dynamicPlan = []

        self.counter = 0
        self.counter_target = 0

        self.state = 0

        self._local_planner = LocalPlanner(  # pylint: disable=undefined-variable
            self._actor, opt_dict={
                'target_speed': 80,
                'lateral_control_dict': self._args})
        

        # Setup sensors
        # bp = CarlaDataProvider.get_world().get_blueprint_library().find('sensor.other.obstacle')

        # bp.set_attribute('distance', '30')
        # bp.set_attribute('hit_radius', '1')
        # bp.set_attribute('only_dynamics', 'True')

        # self._obstacle_sensor = CarlaDataProvider.get_world().spawn_actor(bp, carla.Transform(carla.Location(x=self._actor.bounding_box.extent.x, z=1.0)), attach_to=self._actor)
        # self._obstacle_sensor.listen(lambda event: self._on_obstacle(event))  # pylint: disable=unnecessary-lambda

    def _on_obstacle(self, event):
        """
        Callback for the obstacle sensor (NOT USED NOW)

        Sets _obstacle_distance and _obstacle_actor according to the closest obstacle
        found by the sensor.
        """
        if not event:
            return
        self._obstacle_distance = event.distance
        self._obstacle_actor = event.other_actor

#---------------------------------------------Helper functions---------------------------------------------

    def calcVector(self):
        #Get distance relative to the tracking actor, neg is behind pos is infront
        target = self._tracking_actor.get_location()
        self_location = self._actor.get_location()
        speed_vector = self._tracking_actor.get_velocity()
        return (self_location.x - target.x) * speed_vector.x + (self_location.y - target.y) * speed_vector.y + (self_location.z - target.z) * speed_vector.z


    def _getLane(self, waypoint, angle):
        #Get point angle degrees of input waypoint
        pointX = waypoint.transform.location.x + (waypoint.lane_width*math.cos(math.radians(waypoint.transform.rotation.yaw + angle))) 
        pointY = waypoint.transform.location.y + (waypoint.lane_width*math.sin(math.radians(waypoint.transform.rotation.yaw + angle)))

        #Convert point into a waypoint, will return None if there is no Lane with LaneType.Driving
        rightWaypoint = self.map.get_waypoint(carla.Location(x=pointX, y=pointY, z=0.5), project_to_road=True, lane_type=carla.LaneType.Driving)

        return rightWaypoint

    def findActor(self, loc):
        #Get all vehicle actors in the world
        actors = self.world.get_actors().filter("vehicle.*")

        #Loop over actors
        for actor in actors:
            #Check if distance between input location and actor location is smaller than 5
            if actor.get_location().distance(loc) <= 5:
                #Actor object is found for this input, set the global vars
                self._obstacle_actor = actor
                self._obstacle_distance = self._actor.get_location().distance(actor.get_location())

    def _detect_ahead(self):
        #Get point 15m infront of self
        startX = self._actor.get_location().x + (15*math.cos(math.radians(self._actor.get_transform().rotation.yaw))) 
        startY = self._actor.get_location().y + (15*math.sin(math.radians(self._actor.get_transform().rotation.yaw)))
        start_point = carla.Location(x=startX, y=startY, z=self._actor.get_location().z)


        detected_items = []
        #Look at 5m segments of the actors current lane
        for x in range(15):
            #Create point 5m ahead of the startpoint
            endX = start_point.x + (5*math.cos(math.radians(self._actor.get_transform().rotation.yaw))) 
            endY = start_point.y + (5*math.sin(math.radians(self._actor.get_transform().rotation.yaw)))
            end_point = carla.Location(x=endX, y=endY, z=self._actor.get_location().z)

            #Raycast from the start_point to end_point to detect if anything is there
            for ray in self.world.cast_ray(start_point, end_point):
                #Add dected items to list
                item = [ray.label, ray.location]
                detected_items.append(item)

            #Set the start_point to the end_point to look further along the lane
            start_point = end_point
        
        #Loop over all items that were detected
        for item in detected_items:
            #If detected item was a vehicle
            if(str(item[0]) == 'Vehicles'):
                #Find the actor linked to that locatself.distance_threshold:ion
                self.findActor(item[1])
                break
        
    def _update_plan(self, waypoint, line_color = carla.Color(0,0,255), time=0.5):
        plan = []

        #Create startpoint 15m ahead of self
        startX = waypoint.transform.location.x + (15*math.cos(math.radians(self._actor.get_transform().rotation.yaw))) 
        startY = waypoint.transform.location.y + (15*math.sin(math.radians(self._actor.get_transform().rotation.yaw)))
        start_point = carla.Location(x=startX, y=startY, z=0.5)
        
        #Loop over lane sections
        for x in range(15):
            #Create a segment 5 m ahead of previous point
            endX = start_point.x + (5*math.cos(math.radians(self._actor.get_transform().rotation.yaw))) 
            endY = start_point.y + (5*math.sin(math.radians(self._actor.get_transform().rotation.yaw)))
            end_point = carla.Location(x=endX, y=endY, z=1)

            #Create a waypoint from this location in order to snap to the lane rather than just forward of the car
            waypoint_end = self.map.get_waypoint(end_point,project_to_road=True, lane_type=(carla.LaneType.Driving))

            #Add this to the waypoint plan
            plan.append((waypoint_end, RoadOption.LANEFOLLOW))


            #Debug to draw lane
            waypoint_up = carla.Location(x = waypoint_end.transform.location.x, y = waypoint_end.transform.location.y, z=0.5)
            self.world.debug.draw_line(start_point, waypoint_up,color=line_color, life_time=time)

            #Set the previous point to the waypoint created in order to have a lane following system
            start_point = waypoint_up
        
        #Set the planner to this plan
        self._local_planner.set_global_plan(plan)


#---------------------------------------------STATES---------------------------------------------


    def _lane_follow(self):
        waypoint = self.map.get_waypoint(self._actor.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
        self._update_plan(waypoint)

    def _moveToLeftLane(self):
        waypoint_self = self.map.get_waypoint(self._actor.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
        waypoint_other = self.map.get_waypoint(self._tracking_actor.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))

        #Get waypoint in the right lane next to obstacle car
        waypoint_left = self._getLane(waypoint_other, -90)

        if waypoint_left is None:
            print("No lane to the left!")
            self.state = 3
        else:
            plan = []

            #Get waypoint 15 m ahead of self
            startX = waypoint_self.transform.location.x + (15*math.cos(math.radians(self._actor.get_transform().rotation.yaw))) 
            startY = waypoint_self.transform.location.y + (15*math.sin(math.radians(self._actor.get_transform().rotation.yaw)))
            start_waypoint = self.map.get_waypoint(carla.Location(x=startX, y=startY, z=0.5),project_to_road=True, lane_type=(carla.LaneType.Driving))

            #Get waypoint 15 m ahead of self
            mixX = waypoint_self.transform.location.x + (80*math.cos(math.radians(self._actor.get_transform().rotation.yaw))) 
            midY = waypoint_self.transform.location.y + (80*math.sin(math.radians(self._actor.get_transform().rotation.yaw)))
            mid_waypoint = self.map.get_waypoint(carla.Location(x=mixX, y=midY, z=0.5),project_to_road=True, lane_type=(carla.LaneType.Driving))

            #Get final waypoint 100m ahead of left waypoint
            endX = waypoint_left.transform.location.x + (100*math.cos(math.radians(waypoint_left.transform.rotation.yaw))) 
            endY = waypoint_left.transform.location.y + (100*math.sin(math.radians(waypoint_left.transform.rotation.yaw)))
            end_waypoint = self.map.get_waypoint(carla.Location(x=endX, y=endY, z=0.5),project_to_road=True, lane_type=(carla.LaneType.Driving))

            #Add all 3 waypoints to the plan
            plan.append((start_waypoint, RoadOption.STRAIGHT))
            plan.append((mid_waypoint, RoadOption.STRAIGHT))
            plan.append((end_waypoint, RoadOption.STRAIGHT))

            #Debug to draw and print waypoints
            self.world.debug.draw_line(start_waypoint.transform.location, mid_waypoint.transform.location,color=carla.Color(255,0,0), life_time=30)
            self.world.debug.draw_line(mid_waypoint.transform.location, end_waypoint.transform.location,color=carla.Color(255,0,0), life_time=30)
            print("Size of waypoints: %i" % len(plan))
            
            #Set the localplanner to have this plan
            self._local_planner.set_global_plan(plan)

            #Change to state 4 to wait for the transistion between lanes
            self.state = 5  

    
    def _moveToRightLane(self):
        waypoint_self = self.map.get_waypoint(self._actor.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
        waypoint_other = self.map.get_waypoint(self._obstacle_actor.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))

        #Get waypoint in the right lane next to obstacle car
        waypoint_right = self._getLane(waypoint_other, 90)

        if waypoint_right is None:
            print("No lane to the right!")
            self.state = 3
        else:
            plan = []

            #Get waypoint 15 m ahead of self
            startX = waypoint_self.transform.location.x + (15*math.cos(math.radians(self._actor.get_transform().rotation.yaw))) 
            startY = waypoint_self.transform.location.y + (15*math.sin(math.radians(self._actor.get_transform().rotation.yaw)))
            start_waypoint = self.map.get_waypoint(carla.Location(x=startX, y=startY, z=0.5),project_to_road=True, lane_type=(carla.LaneType.Driving))

            #Get final waypoint 15m ahead of right waypoint
            endX = waypoint_right.transform.location.x + (15*math.cos(math.radians(waypoint_right.transform.rotation.yaw))) 
            endY = waypoint_right.transform.location.y + (15*math.sin(math.radians(waypoint_right.transform.rotation.yaw)))
            end_waypoint = self.map.get_waypoint(carla.Location(x=endX, y=endY, z=0.5),project_to_road=True, lane_type=(carla.LaneType.Driving))

            #Add all 3 waypoints to the plan
            plan.append((start_waypoint, RoadOption.STRAIGHT))
            plan.append((waypoint_right, RoadOption.STRAIGHT))
            plan.append((end_waypoint, RoadOption.STRAIGHT))

            #Debug to draw and print waypoints
            self.world.debug.draw_line(start_waypoint.transform.location, waypoint_right.transform.location,color=carla.Color(255,0,0), life_time=30)
            self.world.debug.draw_line(waypoint_right.transform.location, end_waypoint.transform.location,color=carla.Color(255,0,0), life_time=30)
            print("Size of waypoints: %i" % len(plan))
            
            #Set the localplanner to have this plan
            self._local_planner.set_global_plan(plan)

            #Change to state 4 to wait for the transistion between lanes
            self.state = 4  


    def _stop(self):
        self._local_planner.set_speed(0)
    
#---------------------------------------------Controller functions---------------------------------------------


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

        print("State: %i" % self.state)

        if self.state == 0:
            self._detect_ahead()
            self._lane_follow()
        
        if self.state == 1:
            self._moveToRightLane()

        if self.state == 2:
            self._moveToLeftLane()

        if self.state == 3:
            self._stop()

        if self.state == 4:
            if len(self._local_planner._waypoint_buffer) + len(self._local_planner._waypoints_queue) <= 1:
                self._obstacle_actor = None
                self.state = 0
        
        if self.state == 5:
            if len(self._local_planner._waypoint_buffer) + len(self._local_planner._waypoints_queue) <= 1:
                self._tracking_actor = None
                self.state = 0

        #When to overtake
        if(self._obstacle_actor is not None and self.state is 0):
            current_speed = math.sqrt(self._actor.get_velocity().x**2 + self._actor.get_velocity().y**2)
            current_speed_other = math.sqrt(self._obstacle_actor.get_velocity().x**2 + self._obstacle_actor.get_velocity().y**2)
            speed_dif = current_speed - current_speed_other

            if speed_dif >= 5 and self._obstacle_distance <=100:
                self.state = 1
                self._tracking_actor = self._obstacle_actor


        #When to go back to original lane
        if(self._tracking_actor is not None and self.state is 0):
            product = self.calcVector()
            if product > self.distance_threshold:
                self.state = 2
        
            


        control = self._local_planner.run_step(debug=False)
        # Check if the actor reached the end of the plan
        if self._local_planner.done():
            self._reached_goal = True

        self._actor.apply_control(control)


        pass
