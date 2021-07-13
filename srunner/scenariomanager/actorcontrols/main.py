#!/usr/bin/python3
import sys
sys.path.append( './cavstar' )
import carla
import cavstar.integration as CAVstar

map_name = 'noLoopShort'
vehicle_blueprint_name = 'vehicle.seat.leon'

client = carla.Client( '127.0.0.1', 2000 )
client.set_timeout(3.0)

client.load_world( map_name )
world = client.get_world()
the_map = world.get_map()



print( 'Spawn a new vehicle, look for a blueprint.' )
vehicle_bp = world.get_blueprint_library().filter( vehicle_blueprint_name )[0]
print( 'Using map default spawn location.' )
map_spawn = the_map.get_spawn_points()[0]
print( 'Defined spawn location:', map_spawn )
print( 'Try and spawn a new vehicle.' )
vehicle = world.spawn_actor( vehicle_bp, map_spawn )



(viewer,) = CAVstar.setup_cavstar(client, world, the_map, vehicle)
sensors = CAVstar.SetupSensors(client, world, vehicle)
CAVstar.wait_for_start(world, vehicle)


while True:
    CAVstar.run_step(world, the_map, viewer, vehicle)