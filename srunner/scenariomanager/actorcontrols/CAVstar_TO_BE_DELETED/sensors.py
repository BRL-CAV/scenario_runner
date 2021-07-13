#!/usr/bin/env python

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
import carla

def attach_gnss(vehicle, gnss_queue, can_set_gnss_tick=1, gnss_tick_rate=0.1):
    world = CarlaDataProvider.get_world()

    gnss_bp = world.get_blueprint_library().find( 'sensor.other.gnss' )
    if can_set_gnss_tick:
        print( 'Setting GNSS tick to 0.1' )
        gnss_bp.set_attribute( 'sensor_tick', str(gnss_tick_rate) )
    gnss_trans = carla.Transform( carla.Location( x=0.0, y=0.0, z=0.0 ) )
    gnss = world.spawn_actor( gnss_bp, gnss_trans, attach_to=vehicle )
    gnss.listen( lambda data: gnss_queue.put( data ) )
