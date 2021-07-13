#!/usr/bin/python3


#####################################################################################################
#
#      Copyright (c) Fusion Processing Ltd 2018-2021
#      All rights reserved.
#
#####################################################################################################
#
#  General Options
#
####################################################################################################

# Want Trace ?
tron = True

# IP of riff servers to connect to
server_ip_addr = '91.227.124.104'

# We really want/need synchronous mode but it doesnt work properly
want_synchronous_mode = False

# Name of car / bus model
# vehicle_blueprint_name = 'vehicle.brl.bus'
vehicle_blueprint_name = 'vehicle.seat.leon'

# Set this to True if you just want the vehicle to auto drive around the map generating UTM coords
gather_gnss_coords_for_map = False
want_car_manual_control = False

# Set this if were using one of carlas built in maps which will then apply an offset
# to the lat/lng coords to position it away from 0.0/0.0 which is a daft place to have put it in
# the beginning
use_a_carla_built_in_map = False

gnss_tick_rate = 0.1
imu_tick_rate = 0.1

# Attach a Lane Invasion Detector sensor and display them
want_lane_invasions = False

# Want Lane Offsets & debugging.
# If debug then it draws arrows and prints extra text based on car to lane offset
want_lane_offsets = True
want_lane_offset_debug = True

# Save images from front of car to dir
# NOTE - at present were using async mode and it takes time to save the images which kills the
#        system. When we get sync mode working then it will have the time to save out images and
#        then this should work.
want_car_camera_front = False
want_car_camera_debug_filenames = True
camera_frame_every = 1.0
next_cam_frame = 1

# Nail the viewer to the front of the car per frame and use a screen capture method to grab the result
# This is way faster and runs realtime.
want_viewer_nailed_to_front_of_vehicle = False

# Radar
want_radar_sensor = True


####################################################################################################
#
#  Options per PC
#
####################################################################################################
import sys, os, platform

can_set_gnss_tick = 0

print( 'Setting up Carla python interface' )
carla_version = '0.9.11'
carla_map_path = ''
if os.name == 'posix':
    if platform.node() == 'punch':
        # TODO - this will most likely need to be relative to this interface
        # but for some reason it dont grok with ~/Carla/...
        #carla_dir = '/home/stevee/Carla/Carla_Export_Dists'
        carla_dir = '/home/stevee/Carla/Carla_BRL_BusModel/LinuxNoEditor';
        carla_map_path = '/Game/map_package/Maps/'
        print( 'Setting up for Punch using Carla version ' + carla_version )
        carla_egg = carla_dir + '/PythonAPI/carla/dist/carla-' + carla_version + '-py3.6-linux-x86_64.egg'
        sys.path.append( carla_egg )
        can_set_gnss_tick = 1
    elif platform.node() == 'zotac':
        carla_dir = '/home/tims/LinuxNoEditor';
        carla_map_path = '/Game/map_package/Maps/'
        print( 'Setting up for Zotac using Carla version ' + carla_version )
        carla_egg = carla_dir + '/PythonAPI/carla/dist/carla-' + carla_version + '-py3.6-linux-x86_64.egg'
        sys.path.append( carla_egg )
        can_set_gnss_tick = 1
    elif platform.node() == 'cav-sim':
        # carla_dir = os.getenv('CARLA_ROOT')
        carla_map_path = '/Game/map_package/Maps/'
        # print( 'Setting up for cav-sim using Carla version ' + carla_version )
        # carla_egg = os.getenv('PYTHONPATH')
        # sys.path.append( carla_egg )
        can_set_gnss_tick = 1
    else:
        print( 'Setting up for a different linux machine' )
        # TODO
elif os.name == 'nt':
    print( 'Setting up for Windows' )
    sys.path.append('D:/WindowsNoEditor/PythonAPI/carla/dist/carla-' + carla_version + '-py3.7-win-amd64.egg')

####################################################################################################
#
#  Imports
#
####################################################################################################
import math, time
import rifflib, ctypes
from datetime import datetime, timedelta
from random import *

try:
    import pygame
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_UP
    from pygame.locals import K_DOWN
    from pygame.locals import K_SPACE
    from pygame.locals import K_1
    from pygame.locals import K_2
    from pygame.locals import K_3
    from pygame.locals import K_4
    from pygame.locals import K_5
    from pygame.locals import K_6
    from pygame.locals import K_7
    from pygame.locals import K_8
    from pygame.locals import K_9
    from pygame.locals import K_0
except ImportError:
    raise RuntimeError( 'Cant import pygame' )

print( 'Importing Carla into python' )
import carla, utm, queue

####################################################################################################
#
#  We can now use carla objects. But first, define some interface structs
#
####################################################################################################

# Classes used for SimpleSim over RIFF either MainCritical or MainNonCritical

# Main Non Critical

# Sim Objects "SOBS"
class Riff_MNC_SOBS(ctypes.Structure):
    _fields_ = [ ('x'                 , ctypes.c_float   ),  # Vehicle frame, metres (positive forward)
                 ('y'                 , ctypes.c_float   ),  # Vehicle frame, metres (positive left)
                 ('dx'                , ctypes.c_float   ),  # Vehicle frame, Metres per second
                 ('dy'                , ctypes.c_float   ) ] # Vehicle frame, Metres per second


# Sim Lane Position "SLP "
class Riff_MNC_SLP(ctypes.Structure):
    _fields_ = [ ('timestamp'         , ctypes.c_double  ),  # Time when values were read from sensor, in seconds
                 ('lateral_offset'    , ctypes.c_float   ),  # Metres, positive when located left of route
                 ('lateral_confidence', ctypes.c_float   ),  # Fraction 0 to 1
                 ('heading_error'     , ctypes.c_float   ),  # Radians, positive left of chord between front and rear lane centres
                 ('heading_confidence', ctypes.c_float   ) ] # Fraction 0 to 1


# Sim Traffic Light "STL "
class Riff_MNC_STL(ctypes.Structure):
    _fields_ = [ ('red'               , ctypes.c_bool    ),
                 ('amber'             , ctypes.c_bool    ),
                 ('green'             , ctypes.c_bool    ),
                 ('padding'           , ctypes.c_bool    ) ]

# Sim GNSS "SGLH"
class Riff_MNC_SGLH(ctypes.Structure):
    _fields_ = [ ('utm_northing'      , ctypes.c_double  ),  # UTM metres
                 ('utm_easting'       , ctypes.c_double  ),  # UTM metres
                 ('heading'           , ctypes.c_float   ),  # Degrees clockwise from true north
                 ('heading_rate'      , ctypes.c_float   ),  # Degrees per second
                 ('timestamp'         , ctypes.c_float   ),  # Seconds since start of GPS reference week
                 ('padding'           , ctypes.c_int   ) ]   # Seconds since start of GPS reference week

# Sim Mems Data "SMSD"
class Riff_MNC_SMSD(ctypes.Structure):
    _fields_ = [ ('timestamp'         , ctypes.c_double  ),  # Time when values were read from sensor, in seconds
                 ('forward'           , ctypes.c_float   ),  # Meters per second per second, positive forward
                 ('left'              , ctypes.c_float   ),  # Meters per second per second, positive left, e.g. left turn
                 ('up'                , ctypes.c_float   ),  # Meters per second per second, positive up, e.g. gravity positive
                 ('yaw'               , ctypes.c_float   ),  # Radians per second, positive left turn
                 ('pitch'             , ctypes.c_float   ),  # Radians per second, positive nose down
                 ('roll'              , ctypes.c_float   ) ] # Radians per second, positive right side down

# Main Critical

# Sim Vehicle Control "SVC "
class Riff_MC_SVC(ctypes.Structure):
    _fields_ = [ ('throttle'          , ctypes.c_float   ),  # Fraction in the range 0.0 to 1.0
                 ('brake'             , ctypes.c_float   ),  # Fraction in the range 0.0 to 1.0
                 ('steering_torque'   , ctypes.c_float   ),  # Fraction in the range -1.0 to 1.0, Positive to the left
                 ('padding'           , ctypes.c_float   ) ]

# Sim Vehicle Feedback "SVF "
class Riff_MC_SVF(ctypes.Structure):
    _fields_ = [ ('timestamp'         , ctypes.c_double  ),  # Time when values updated
                 ('distance_left'     , ctypes.c_float   ),  # Metres
                 ('distance_right'    , ctypes.c_float   ),  # Metres
                 ('turning_angle'     , ctypes.c_float   ),  # Radians, Positive to the left
                 ('padding'           , ctypes.c_float   ) ]

# Data structures transfered to Main Non Critical (The sensor interface)
riff_MNC_SOBS = Riff_MNC_SOBS()
riff_MNC_SLP  = Riff_MNC_SLP()
riff_MNC_STL  = Riff_MNC_STL()
riff_MNC_SGLH = Riff_MNC_SGLH()
riff_MNC_SMSD = Riff_MNC_SMSD()

# Data structures transfered to and from Main Critical (The vehicle Interface)
riff_MC_SVC  = Riff_MC_SVC()
riff_MC_SVF  = Riff_MC_SVF()

gnss_riff_message = rifflib.Message()
mems_riff_message = rifflib.Message()
lane_riff_message = rifflib.Message()
radar_riff_message = rifflib.Message()
veh_control_riff_message  = rifflib.Message()

# Initialise the RIFF clients for the ADS to connect to
gnss_riff_client         = rifflib.RiffClient( 6163, server_ip_addr, 5, 'carla non critical gnss' )
mems_riff_client         = rifflib.RiffClient( 6164, server_ip_addr, 5, 'carla non critical mems' )
lane_riff_client         = rifflib.RiffClient( 6166, server_ip_addr, 5, 'carla non critical lane' )
radar_riff_client        = rifflib.RiffClient( 6166, server_ip_addr, 5, 'carla non critical radar' )
veh_control_riff_client  = rifflib.RiffClient( 6168, server_ip_addr, 5, 'carla critical veh ctl' )


####################################################################################################
#
####################################################################################################
start_time = time.time()
def GetMsFromStart():
    global start_time
    return int( (time.time() - start_time) * 1000.0 )


####################################################################################################
#
####################################################################################################
class Gnss( object ):
    lat = 0.0
    lng = 0.0
    utm_x = 0.0
    utm_y = 0.0

    def __init__( self ):
        self.lat = 0.0
        self.lng = 0.0
        self.utm_x = 0.0
        self.utm_y = 0.0

    def Set( self, lat_in, lng_in, utmx_in, utmy_in ):
        self.lat = lat_in
        self.lng = lng_in
        self.utm_x = utmx_in
        self.utm_y = utmy_in

    def UtmDist( self, utmx_in, utmy_in ):
        dist2 = (utmx_in - self.utm_x)*(utmx_in - self.utm_x) + (utmy_in - self.utm_y)*(utmy_in - self.utm_y);
        if dist2 > 0.0001:
            return math.sqrt( dist2 )
        else:
            return 0.0

    def PrintUtm( self ):
        print( f'GNSS: ({self.lat: 12.7f}, {self.lng: 12.7f}) -> ({self.utm_x: 12.3f},{self.utm_y: 12.3f})' )

    def PrintUtmWithTime( self ):
        now = GetMsFromStart() / 1000.0
        print( f'{now: 12.3f} : ({self.utm_x: 12.3f},{self.utm_y: 12.3f})' )

vehicle_gnss = Gnss()

gnss_queue = queue.Queue(1)
imu_queue  = queue.Queue(1)
lid_queue  = queue.Queue(1)
ccam_queue = queue.Queue(1)
radar_queue = queue.Queue(1)

next_cam_frame = 1;


####################################################################################################
#
# Since there is no reverse of map.transform_to_geolocation to go from lat long to
# map coords, we need to run a loop until were close enough
# Definitely not optimized ! But when used, this is only done once
#
####################################################################################################
def SetVehicleAtUtm( the_map, vehicle, utm_x, utm_y, yaw ):
    print( 'Setting verhicle at UTM:', utm_x, utm_y )
    (req_lat, req_lon) = utm.to_latlon( utm_x, utm_y, 32, 'U' )

    map_x = 0.0
    map_y = 0.0
    dist2 = 9999999999999.9

    # Run a fixed loop of 10K in 0.1 steps so we can move up to 1000.0 which is more than the map
    for loop in range( 0, 10000 ):
        geo = the_map.transform_to_geolocation( carla.Location( x=map_x, y=map_y ) )

        if geo.latitude < req_lat:
            map_y = map_y - 0.1
        else:
            map_y = map_y + 0.1

        if geo.longitude < req_lon:
            map_x = map_x + 0.1
        else:
            map_x = map_x - 0.1

        #print( 'Req_LL:', req_lat, req_lon, 'Cur_LL', geo.latitude, geo.longitude, 'Map pos', map_x, ',', map_y )

    trans = vehicle.get_transform()
    trans.location.x = map_x
    trans.location.y = map_y
    trans.location.z = 0.5
    trans.rotation.yaw = yaw
    vehicle.set_transform( trans )
    return



####################################################################################################
#
# Set up Carla Sim
#
####################################################################################################
try:
    print( 'Creating a Carla Sim Client' )
    client = carla.Client( '127.0.0.1', 2000 )
    # Print the version but it had better match that which we set earlier
    print( 'Carla client version: ', client.get_client_version() )
except Exception:
    print( 'Exception: Could not create Carla client.' )
    sys.exit( 1 )

try:
    print( 'The available maps for this Carla server are:' )
    print( client.get_available_maps() )

    # Pick a map from the list provided, if not one of the Town ones then you should have
    # built the tar.gz package distribution with the map embedded into it, which is...
    # involved to say the least and needs RoadRunner exports to UE4 imports and build etc

    req_map = ''
    if( use_a_carla_built_in_map ):
        #req_map = carla_map_path + 'Town01'
        #req_map = carla_map_path + 'Town01_Opt'
        #req_map = carla_map_path + 'Town02'
        #req_map = carla_map_path + 'Town02_Opt'
        req_map = carla_map_path + 'Town03'
        #req_map = carla_map_path + 'Town03_Opt'
        #req_map = carla_map_path + 'Town04'
        #req_map = carla_map_path + 'Town04_Opt'
        #req_map = carla_map_path + 'Town05'
        #req_map = carla_map_path + 'Town05_Opt'
    else:
        #req_map = carla_map_path + 'MWayShortFlat'
        #req_map = carla_map_path + 'northTerminal/northTerminal'
        # req_map = carla_map_path + 'northBound/northBound'
        #req_map = carla_map_path + 'southTerminal/southTerminal'
        #req_map = carla_map_path + 'southBound/southBound'
        req_map = 'noLoopShort'
        # req_map = 'northBound'

    if req_map != '':
        print( 'Requesting map: ', req_map )
        client.load_world( req_map )
except Exception:
    print( 'Exception: Could not set the requested Carla world map.' )
    sys.exit( 1 )

try:
    print( 'Setting world parameters' )
    world = client.get_world()
    world_settings = world.get_settings()

    if want_synchronous_mode:
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = 0.05
        world_settings.substepping = True
        world_settings.max_substep_delta_time = 0.01
        world_settings.max_substeps = 10
    else:
        world_settings.synchronous_mode = False

    world.apply_settings( world_settings )
except Exception:
    print( 'Exception: Setting world parameters.' )
    sys.exit( 1 )

try:
    print( 'Obtain the world map' )
    the_map = world.get_map()
except Exception:
    print( 'Exception: Getting the map.' )
    print( "You can get here if the XODR file was not included in the package." )
    print( "You may need to manually get it and add it into the expanded package." )
    sys.exit( 1 )

try:
    vehicles = world.get_actors().filter( vehicle_blueprint_name )
    if len( vehicles ) == 0:
        print( 'Spawn a new vehicle, look for a blueprint.' )
        vehicle_bp = world.get_blueprint_library().filter( vehicle_blueprint_name )[0]
        print( 'Using map default spawn location.' )
        map_spawn = the_map.get_spawn_points()[0]
        print( 'Defined spawn location:', map_spawn )
        print( 'Transform spawn location to map coordinates.' )
        map_geo = the_map.transform_to_geolocation( map_spawn.location )
        print( 'Vehicle should be spawned at', utm.from_latlon( map_geo.latitude, map_geo.longitude ) )
        print( 'Try and spawn a new vehicle.' )
        vehicle = world.spawn_actor( vehicle_bp, map_spawn )
    else:
        print( 'Using existing vehicle' )
        vehicle = vehicles[0]
        map_geo = the_map.transform_to_geolocation( vehicle.get_location() )
        print( 'Vehicle found at', utm.from_latlon( map_geo.latitude, map_geo.longitude ) )

        all_gnss = world.get_actors().filter('sensor.other.gnss')
        for x in all_gnss:
            x.destroy()
except Exception as ex:
    print( 'Exception from creating vehicle: ', ex )
    sys.exit( 1 )

try:
    print( 'Perform first tick' ) # so that the set vehicle locations can be obtained from them.
    world.tick();
    vehicles = world.get_actors().filter( vehicle_blueprint_name )
    print( 'Number of vehicles in sim:', len( vehicles ) )
    vehicle = vehicles[0]
    veh_trans = vehicle.get_transform()

    if req_map == carla_map_path + 'northBound/northBound':
        # For northBound map, position the car at start point

        # On the easterly Roundabout
        #veh_trans.location.x = 1138.0
        #veh_trans.location.y = 58.4
        #veh_trans.rotation.yaw = -20.0

        # On straight if testing lane offsets
        veh_trans.location.x = 926.0
        veh_trans.location.y = 143.0
        veh_trans.rotation.yaw = -175.0

        vehicle.set_transform( veh_trans )

    print( 'Vehicle is at position:', veh_trans )
    veh_hdg = (90 + 360 + veh_trans.rotation.yaw) % 360
    print( 'Note: This is heading:', round(veh_hdg,1) )

except Exception:
    print( 'Exception: Setting world first tick' )
    sys.exit( 1 )

try:
    print( 'Setting camera high above the vehicle and pointing downwards at it.' )
    viewer = world.get_spectator()
    view_trans = veh_trans
    if req_map == carla_map_path + 'northBound/northBound':
        view_trans.location.z = 200
        view_trans.location.x = veh_trans.location.x
        view_trans.location.y = veh_trans.location.y + 50
        view_trans.rotation.yaw = -90
        view_trans.rotation.pitch = -80
    else:
        view_trans.location.z = 50 + veh_trans.location.z
        view_trans.location.x = veh_trans.location.x
        view_trans.location.y = veh_trans.location.y 
        view_trans.rotation.pitch = -80
    viewer.set_transform( view_trans )
except Exception:
    print( 'Exception: Setting camera view transform to vehicle.' )
    sys.exit( 1 )

try:
    print( 'Setting client timeout' ) # back to something sensible or else you wont be able to quit it.
    client.set_timeout( 10.0 )
except Exception:
    print( 'Exception: Setting client timeout' )
    sys.exit( 1 )

try:
    print( 'Set up sensor and attach to vehicle' )
    try:
        print( 'Set up GNSS' )
        gnss_bp = world.get_blueprint_library().find( 'sensor.other.gnss' )
        if can_set_gnss_tick:
            print( 'Setting GNSS tick to 0.1' );
            gnss_bp.set_attribute( 'sensor_tick', str(gnss_tick_rate) )
        gnss_trans = carla.Transform( carla.Location( x=0.0, y=0.0, z=0.0 ) )
        gnss = world.spawn_actor( gnss_bp, gnss_trans, attach_to=vehicle )
        gnss.listen( lambda data: gnss_queue.put( data ) )
    except Exception as ex:
        print('Exception: GNSS:', ex )
        sys.exit( 1 )

    try:
        print( 'Set up IMU' )
        imu_bp = world.get_blueprint_library().find( 'sensor.other.imu' )
        imu_bp.set_attribute( 'sensor_tick', str(imu_tick_rate) )
        imu_trans = carla.Transform( carla.Location( x=0.0, y=0.0, z=0.0 ) )
        imu = world.spawn_actor( imu_bp, imu_trans, attach_to=vehicle )
        imu.listen( lambda data: imu_queue.put( data ) )
    except Exception as ex:
        print('Exception: IMU:', ex )
        sys.exit( 1 )

    if want_lane_invasions:
        try:
            print( 'Set up LaneInvasionDetector' )
            lid_bp = world.get_blueprint_library().find( 'sensor.other.lane_invasion' )
            lid_trans = carla.Transform( carla.Location( x=0.0, y=0.0, z=0.0 ) )
            lid = world.spawn_actor( lid_bp, lid_trans, attach_to=vehicle )
            lid.listen( lambda data: lid_queue.put( data ) )
        except Exception as ex:
            print('Exception: Lane Invasion Sensor:', ex )
            sys.exit( 1 )

    if want_car_camera_front:
        try:
            print( 'Set up Car Camera' )
            ccam_bp = world.get_blueprint_library().find( 'sensor.camera.rgb' )
            ccam_rate = "%f" %(camera_frame_every)
            ccam_bp.set_attribute( 'sensor_tick', ccam_rate )
            ccam_bp.set_attribute( 'image_size_x',  '320' )
            ccam_bp.set_attribute( 'image_size_y',  '240' )
            ccam_bp.set_attribute( 'fov',           '90' )
            ccam_bp.set_attribute( 'exposure_mode', 'manual' )
            ccam_bp.set_attribute( 'shutter_speed', '250.0' )
            # Set position. Put Z a couple m above car else you wont see much
            ccam_trans = carla.Transform( carla.Location( x=0.8, y=0.0, z=2.0 ) )
            ccam = world.spawn_actor( ccam_bp, ccam_trans, attach_to=vehicle )
            ccam.listen( lambda image: ccam_queue.put( image ) )
        except Exception as ex:
            print('Exception: Camera:', ex )
            sys.exit( 1 )

    if want_radar_sensor:
        try:
            print( 'Set up Radar Sensor' )
            radar_bp = world.get_blueprint_library().find( 'sensor.other.radar' )
            radar_bp.set_attribute( 'horizontal_fov',    '40.0'  )
            radar_bp.set_attribute( 'vertical_fov',      '12.0'  )
            radar_bp.set_attribute( 'range',             '150.0' )
            radar_bp.set_attribute( 'points_per_second', '1000' )
            radar_bp.set_attribute( 'sensor_tick',       '1.0'   )
            # Set position. Put Z a couple m above car else you wont see much
            radar_trans = carla.Transform( carla.Location( x=0.0, y=2.0, z=1.0 ) )
            radar = world.spawn_actor( radar_bp, radar_trans, attach_to=vehicle )
            radar.listen( lambda data: radar_queue.put( data ) )
        except Exception as ex:
            print('Exception: Radar:', ex )
            sys.exit( 1 )

except Exception:
    print( 'Exception setting up sensors' )
    sys.exit( 1 )


####################################################################################################
#
# Run Carla Sim in a loop
#
####################################################################################################

# Do you want to just collect up UTM coords of the map or drive it under sim or keyboard control
if gather_gnss_coords_for_map:
    print( 'Getting GNSS coords only' )
    if want_car_manual_control == False:
        vehicle.set_autopilot( True )
        while True:
            try:
                world.tick()

                if gnss_queue.qsize() > 0:
                    data = gnss_queue.get()
                    if data:
                        latitude = data.latitude
                        longitude = data.longitude

                        # Total hack to get the built in maps away from 0 lat which causes UTM issues
                        if( use_a_carla_built_in_map ):
                            latitude  = latitude  + 1.0
                            longitude = longitude + 1.0

                        utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon( latitude, longitude )
                        if vehicle_gnss.UtmDist( utm_x, utm_y ) > 1.0:
                            vehicle_gnss.Set( latitude, longitude, utm_x, utm_y )
                            vehicle_gnss.PrintUtm()

            except RuntimeError as ex:
                print('Exception: world tick:', ex )
    try:
        if want_car_manual_control:

            veh_throttle = 0.0
            veh_brake = 0.0
            veh_steer = 0.0
            veh_reverse = False

            # For northBound map
            veh_trans = vehicle.get_transform()
            veh_trans.location.x = 1100.0
            veh_trans.location.y = 75.0
            veh_trans.rotation.yaw = -45.0
            vehicle.set_transform( veh_trans )

            fixed_val = 0.0

            pygame.init()
            pygame.display.set_mode( (100,100) )
            now = GetMsFromStart() / 1000.0
            last_print = now
            while True:
                now = GetMsFromStart() / 1000.0
                try:
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            sys.exit()
                        if event.type == pygame.KEYDOWN:
                            key = pygame.key.get_pressed()

                            if event.key == K_LEFT:
                                veh_steer = veh_steer - 0.1;
                            if event.key == K_RIGHT:
                                veh_steer = veh_steer + 0.1;
                            if event.key == K_UP:
                                veh_throttle = veh_throttle + 0.1
                                veh_brake = 0.0
                            if event.key == K_DOWN:
                                veh_throttle = 0.0
                                veh_brake = veh_brake + 0.1
                            if event.key == K_SPACE:
                                veh_reverse = not veh_reverse
                            if event.key == K_1:
                                fixed_val = 0.1
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_2:
                                fixed_val = 0.2
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_3:
                                fixed_val = 0.3
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_4:
                                fixed_val = 0.4
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_5:
                                fixed_val = 0.5
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_6:
                                fixed_val = 0.6
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_7:
                                fixed_val = 0.7
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_8:
                                fixed_val = 0.8
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_9:
                                fixed_val = 0.9
                                print( "Fixed Val = ", fixed_val )
                            if event.key == K_0:
                                fixed_val = 1.0
                                print( "Fixed Val = ", fixed_val )

                    if fixed_val != 0.0:
                        cur_vel = vehicle.get_velocity()
                        if cur_vel.x > 20.0:
                            print( "Velocity:", cur_vel.x )
                            print( "Applying brakes at:", fixed_val )
                            veh_throttle = 0.0
                            veh_brake = fixed_val
                            fixed_val = 0.0

                except Exception as ex:
                    print( 'Pygame exception:', ex )

                try:
                    if veh_steer < -0.001:
                        veh_steer = veh_steer + 0.005
                    if veh_steer > 0.001:
                        veh_steer = veh_steer - 0.005

                    veh_throttle = veh_throttle - 0.001
                    veh_brake = veh_brake - 0.001

                    veh_steer    = min( 1.0, max( -1.0, veh_steer ) )
                    veh_throttle = min( 1.0, max(  0.0, veh_throttle ) )
                    veh_brake    = min( 1.0, max(  0.0, veh_brake ) )

                    vehicle.apply_control( carla.VehicleControl( throttle = veh_throttle, steer = veh_steer, brake = veh_brake, reverse = veh_reverse ) )
                    #print( f'Vehicle St/Th/Br/Rv: {veh_steer: 8.3f}, {veh_throttle: 8.3f}, {veh_brake: 8.3f}', veh_reverse )
                except Exception as ex:
                    print('Exception: vehicle control:', ex )

                if gnss_queue.qsize() > 0:
                    data = gnss_queue.get()
                    if data:
                        latitude = data.latitude
                        longitude = data.longitude

                        # Total hack to get the built in maps away from 0 lat which causes UTM issues
                        if( use_a_carla_built_in_map ):
                            latitude  = latitude  + 1.0
                            longitude = longitude + 1.0

                        utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon( latitude, longitude )
                        if vehicle_gnss.UtmDist( utm_x, utm_y ) > 1.0:
                            vehicle_gnss.Set( latitude, longitude, utm_x, utm_y )
                            vehicle_gnss.PrintUtmWithTime()
                            car_pos = vehicle.get_location()
                            print( round( car_pos.x,5 ), round( car_pos.y,5 ) )


                try:
                    world.wait_for_tick()
                except Exception as ex:
                    print('Exception: waiting for world tick:', ex )
    except KeyboardInterrupt:
        print( 'Keyboard interrupt, exiting...' )
else:
    print( "Run the vehicle under Fusion CAV Sim control\n" )
    steering_lock_angle_rad = 55 * 3.14159 / 180

    speed_error_integral = 0

    veh_utm_x = 0.0
    veh_utm_y = 0.0
    last_veh_utm_x = 0.0
    last_veh_utm_y = 0.0
    veh_spd = 0.0
    veh_hdg = 0.0
    veh_throttle = 0.0
    veh_brake = 0.0
    veh_steer = 0.0
    gnss_ms = 0
    req_throttle = 0.0
    req_brake = 0.0
    req_steer_torque = 0.0
    vldist = 0.0
    heading_error = 0.0
    vyawrad = 0.0
    lyawrad = 0.0
    distance_left = 0.0
    distance_right = 0.0
    veh_wheelbase = 4.0
    veh_width = 1.8
    wheel_utm_x = 0.0
    wheel_utm_y = 0.0

    # Work out how many seconds have elapsed since the start of the week since this offset must be
    # added to the timestamps of the messages (esp GNSS types).
    time_now = datetime.now()
    dow = time_now.isoweekday()
    delta = timedelta( days = -dow )
    sow = time_now + delta
    time_sow = datetime( sow.year, sow.month, sow.day, 0,0,0 )
    diff_sow = time_now - time_sow
    timestamp_offset = diff_sow.days * 86400 + diff_sow.seconds

    try:
        frame_counter = 0
        last_update_SVF_time = 0.0
        last_update_SGLH_time = 0.0
        last_update_SMSD_time = 0.0
        last_update_SLP_time = 0.0
        last_update_SLP_time_debug = 0.0
        last_update_gnss_time = 0.0

        # Before get going proper, send a message about where the car is.
        wait_for_start = 1
        while wait_for_start:
            if gnss_queue.qsize() > 0:
                data = gnss_queue.get()
                if data:
                    latitude  = data.latitude
                    longitude = data.longitude

                    # Total hack to get the built in maps away from 0 lat which causes UTM issues
                    if( use_a_carla_built_in_map ):
                        latitude  = latitude  + 1.0
                        longitude = longitude + 1.0

                    utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon( latitude, longitude )

                    last_veh_utm_x = utm_x
                    last_veh_utm_y = utm_y
                    veh_utm_x = utm_x
                    veh_utm_y = utm_y
                    veh_hdg = (90 + 360 + vehicle.get_transform().rotation.yaw) % 360

                    now = GetMsFromStart() / 1000.0

                    # Return position and heading to the controller
                    riff_MNC_SGLH.utm_northing = veh_utm_y
                    riff_MNC_SGLH.utm_easting = veh_utm_x
                    riff_MNC_SGLH.heading = veh_hdg
                    riff_MNC_SGLH.heading_rate = 0
                    riff_MNC_SGLH.timestamp = timestamp_offset + now

                    gnss_riff_message.tag = b'SGLH'
                    gnss_riff_message.length = ctypes.sizeof( riff_MNC_SGLH )
                    gnss_riff_message.data   = ctypes.byref(  riff_MNC_SGLH )
                    gnss_riff_client.write( gnss_riff_message );

                    wait_for_start = 0
                    print( "Got Start" )
            else:
                try:
                    world.wait_for_tick()
                except Exception as ex:
                    print('Exception: waiting for world tick:', ex )

        # Now on to loop proper
        while True:
            frame_counter = frame_counter + 1
            now = GetMsFromStart() / 1000.0

            gnss_riff_client.connect()
            mems_riff_client.connect()
            lane_riff_client.connect()
            veh_control_riff_client.connect()

            # Were not expecting to receive anything from these only write back to controller so keep them flushed
            gnss_riff_message = gnss_riff_client.receive()
            mems_riff_message = gnss_riff_client.receive()
            lane_riff_message = lane_riff_client.receive()

            if ccam_queue.qsize() > 0:
                if want_car_camera_debug_filenames:
                    if tron:
                        print( 'Pre Cam save' )
                image = ccam_queue.get()
                if image:
                    # Save raw_data as image using next_frame_num in preset dir
                    fn = "./cam_frames/Fr_%06d.jpg" % (next_cam_frame)
                    if want_car_camera_debug_filenames:
                        print( 'Saving frame <',fn,'>' )
                    image.save_to_disk( fn )
                    next_cam_frame = next_cam_frame + 1

            if want_viewer_nailed_to_front_of_vehicle:
                veh_trans = vehicle.get_transform()
                view_trans = veh_trans
                view_trans.location.z = veh_trans.location.z + 3.0
                viewer.set_transform( view_trans )

            new_vehicle_controller_motion = False
            while True: # We only want the latest one
                try: veh_control_riff_message = veh_control_riff_client.receive()
                except: pass # Continue if the connection is lost
                length = veh_control_riff_message.length.value
                data = veh_control_riff_message.data
                if not length: break
                if veh_control_riff_message.tag.value == b'SVC ':
                    if length == ctypes.sizeof( riff_MC_SVC ):
                        ctypes.memmove( ctypes.byref( riff_MC_SVC ), data, length )

                        req_throttle     = riff_MC_SVC.throttle
                        req_brake        = riff_MC_SVC.brake
                        req_steer_torque = riff_MC_SVC.steering_torque
                        #print( 'Read SVC : ReqThrot:', round( req_throttle, 4), '  ReqStT:', round( req_steer_torque, 6), '  ReqBr:', round( req_brake,2 ) )
                    else:
                        print( 'Unexpected vehicle controller motion length', length )

            if now - last_update_SVF_time > 0.1:
                dt = now - last_update_SVF_time;
                last_update_SVF_time = now

                inc_dist = veh_spd * dt;
                distance_left  = distance_left  + inc_dist
                distance_right = distance_right + inc_dist
                turning_angle = veh_steer * steering_lock_angle_rad
                if turning_angle < -0.02 or turning_angle > 0.02:
                    turn_radius = veh_wheelbase / math.tan( turning_angle )
                    extra_dist = (inc_dist / turn_radius) * (veh_width / 2)
                    distance_left  -= extra_dist
                    distance_right += extra_dist
                    #print( "Turn angle:", round(turning_angle,3), "  turn radius:", round(turn_radius,1), "  extra dist:", round(extra_dist,3) )

                # Calc absolute wheel positions to compare with gnss
                gnss_wheel_dx = math.fabs( veh_utm_x - wheel_utm_x )
                gnss_wheel_dy = math.fabs( veh_utm_y - wheel_utm_y )

                if gnss_wheel_dx > 1000.0 or gnss_wheel_dy > 1000.0:
                    wheel_utm_x = veh_utm_x
                    wheel_utm_y = veh_utm_y

                # Note heading is not same as math angle hence sin/cos swap
                i = inc_dist * math.sin( veh_hdg * 3.14159 / 180 )
                j = inc_dist * math.cos( veh_hdg * 3.14159 / 180 )
                #print( 'Hdg, Spd, Inc, Wheel ij:', round(veh_hdg,1), round(veh_spd,3), round(inc_dist,3), round(i,3), round(j,3) )

                wheel_utm_x += i
                wheel_utm_y += j
                gnss_wheel_dx = veh_utm_x - wheel_utm_x
                gnss_wheel_dy = veh_utm_y - wheel_utm_y
                #print( 'V/W', round(veh_utm_x,3), round(veh_utm_y,3), round(wheel_utm_x,3), round(wheel_utm_y,3) )
                #print( 'delta Veh:Wheel x,y:', round(gnss_wheel_dx,3), round(gnss_wheel_dy,3) )


		# SVC steer torque is +ve to left
		# SVF steer angle is +ve to left in radians
		# Carla steer is +ve right
		# So we will adopt +ve left and give carla -ve of it

		# Need to convert steer torque to steer angle.
		# Integrate torque with dt - a torque of 0.2 gives one radian per second on the van

                veh_throttle = req_throttle
                veh_brake    = req_brake
                veh_steer    += req_steer_torque * dt / (0.2 * steering_lock_angle_rad)

                veh_throttle = min( 1.0, max(  0.0, veh_throttle ) )
                veh_brake    = min( 1.0, max(  0.0, veh_brake ) )
                veh_steer    = min( 1.0, max( -1.0, veh_steer ) )

                #if tron:
                #    print( 'Throttle:', round( veh_throttle, 2), '   Steer:', round( veh_steer * steering_lock_angle_rad, 1), '  Brake:', round( veh_brake,2 ) )

                vehicle.apply_control( carla.VehicleControl( throttle = veh_throttle, steer = -1.0 * veh_steer, brake = veh_brake ) )

                # Must feedback state of the motion to the controller in order to set the heading and the timestamp for it
                riff_MC_SVF.timestamp = timestamp_offset + now
                riff_MC_SVF.distance_left = distance_left
                riff_MC_SVF.distance_right = distance_right
                riff_MC_SVF.turning_angle = veh_steer * steering_lock_angle_rad
                riff_MC_SVF.padding = 0.0

                veh_control_riff_message.tag = b'SVF '
                veh_control_riff_message.length = ctypes.sizeof( riff_MC_SVF )
                veh_control_riff_message.data   = ctypes.byref(  riff_MC_SVF )
                veh_control_riff_client.write( veh_control_riff_message );
                #print( "Sent distance left,right as", round(distance_left,1), round(distance_right,1) )
                #print( 'Send SVF : turn_angle (in rads):', round( riff_MC_SVF.turning_angle,3) )

            try:
                world.wait_for_tick()
            except Exception as ex:
                print('Exception: waiting for world tick:', ex )
            now = GetMsFromStart() / 1000.0

            # Receive information from Carla and send to ADS
            if gnss_queue.qsize() > 0:
                data = gnss_queue.get()
                if data:
                    if now - last_update_gnss_time > gnss_tick_rate:
                        latitude  = data.latitude
                        longitude = data.longitude

                        # Total hack to get the built in maps away from 0 lat which causes UTM issues
                        if( use_a_carla_built_in_map ):
                            latitude  = latitude  + 1.0
                            longitude = longitude + 1.0

                        # Work out vehicle position, speed and heading
                        utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon( latitude, longitude )
                        #print( "LatLng:", latitude, longitude )
                        #print( "UTM:", utm_x, utm_y )

                        last_veh_utm_x = veh_utm_x
                        last_veh_utm_y = veh_utm_y
                        veh_utm_x = utm_x
                        veh_utm_y = utm_y

                        veh_hdg = (90 + 360 + vehicle.get_transform().rotation.yaw) % 360

                        dx = veh_utm_x - last_veh_utm_x
                        dy = veh_utm_y - last_veh_utm_y
                        dist = dx*dx + dy*dy
                        if dist > 0.0:
                            dist = math.sqrt( dist )
                        dt = now - last_update_gnss_time
                        veh_spd = dist / dt

                        # How does this compare with the velocity as stated by Carla ?
                        vel = vehicle.get_velocity()
                        veh_spd2 = vel.x * vel.x + vel.y * vel.y
                        if veh_spd2 > 0.0:
                            veh_spd2 = math.sqrt( veh_spd2 )

                        #print( 'Spd, Dst, Time Spd2:', veh_spd, dist, dt, veh_spd2 )
                        last_update_gnss_time = now;

            if imu_queue.qsize() > 0:
                data = imu_queue.get()
                if data:
                    if now - last_update_SMSD_time > 0.1:
                        dt = now - last_update_SMSD_time;
                        last_update_SMSD_time = now;

                        riff_MNC_SMSD.timestamp = timestamp_offset + now
                        riff_MNC_SMSD.forward   = -data.accelerometer.x
                        riff_MNC_SMSD.left      = -data.accelerometer.y
                        riff_MNC_SMSD.up        = data.accelerometer.z
                        riff_MNC_SMSD.yaw       = -data.gyroscope.z
                        riff_MNC_SMSD.pitch     = -data.gyroscope.y
                        riff_MNC_SMSD.roll      = -data.gyroscope.x

                        mems_riff_message.tag = b'SMSD'
                        mems_riff_message.length = ctypes.sizeof( riff_MNC_SMSD )
                        mems_riff_message.data   = ctypes.byref(  riff_MNC_SMSD )
                        mems_riff_client.write( mems_riff_message );

                        # From cavstar.conf
                        wheelbase = 4.0
                        turn = veh_steer * steering_lock_angle_rad
                        yaw_steer = veh_spd * math.tan( turn ) / wheelbase
                        #print( 'GyroZ/Yaw/s, turn, Spd, TA/s:', round( -data.gyroscope.z,5 ), round( turn, 5), round( veh_spd,3 ), round( yaw_steer, 5 ) )
                        #print( 'Send SMSD: Acc, Gyro: ', round(data.accelerometer.x,3), round(data.accelerometer.y,3), round(data.accelerometer.z,3), round(data.gyroscope.x,3), round(data.gyroscope.y,3), round(data.gyroscope.z,3) )

            if now - last_update_SGLH_time > 0.1:
                heading_rate = (veh_hdg - riff_MNC_SGLH.heading) / (now - last_update_SGLH_time);
                last_update_SGLH_time = now

                # Return position and heading to the controller
                riff_MNC_SGLH.utm_northing = veh_utm_y
                riff_MNC_SGLH.utm_easting = veh_utm_x
                riff_MNC_SGLH.heading = veh_hdg
                riff_MNC_SGLH.heading_rate = heading_rate
                riff_MNC_SGLH.timestamp = timestamp_offset + now

                gnss_riff_message.tag = b'SGLH'
                gnss_riff_message.length = ctypes.sizeof( riff_MNC_SGLH )
                gnss_riff_message.data   = ctypes.byref(  riff_MNC_SGLH )
                gnss_riff_client.write( gnss_riff_message );
                #print( 'Send SGLH: x,y,hdg: ', round(veh_utm_x,1), round(veh_utm_y,1), round(veh_hdg,3) )

            # Lane invasion detector has new data ?
            if lid_queue.qsize() > 0:
                data = lid_queue.get()
                if data:
                    frame = data.frame
                    #print( 'Lane Invasion Detector: ', frame )

            # Radar has new data ?
            if radar_queue.qsize() > 0:
                data = radar_queue.get()
                if data:
                    # We need to build up a large array of values
                    # Whilst it might be possible to do it with an array of riff_MNC_SOBS's
                    # Its just a darn sight easier to use lists of c_float's
                    len_det = len(data);
                    det_array = []
                    det_array_count = 0;
                    for det in data:
                        alt = det.altitude        # In radians
                        azi = det.azimuth         # In radians
                        rng = det.depth           # In m
                        vel = det.velocity        # In m/s +ve away from sensor, -ve towards

                        # Weirdly, the SOBS XY coord system is looking down on the car from above where the car is facing
                        # to the right. So X is distance from front of sensor (or range) and Y is distance off of the
                        # axial line out from the car in the up-down plane. +Y is therefore to the left of the car when
                        # looking out from the car in the direction of the sensor.

                        # sobs_x is range in the plane
                        sobs_x  = rng * math.cos(alt) * math.cos(azi)
                        sobs_dx = vel * math.cos(alt) * math.cos(azi)

                        # sobs_y is lateral distance left-right only
                        sobs_y  = rng * math.sin(azi)
                        sobs_dy = vel * math.sin(azi)

                        det_array.append( sobs_x )
                        det_array.append( sobs_y )
                        det_array.append( sobs_dx )
                        det_array.append( sobs_dy )

                        det_array_count = det_array_count + 4

                        #if det_array_count == 4:
                        #    print( 'Alt, Azi, Rng, Vel:', round(alt,5), round(azi,5), round(rng,3), round(vel,1) )
                        #    print( 'x,y,dx,dy:', round(sobs_x,3), round(sobs_y,3), round(sobs_dx,1), round(sobs_dy,1) )

                    if det_array_count > 0:
                        # CTypes voodoo to convert python array into ctypes floats
                        ctarray = ctypes.c_float * det_array_count
                        ctarray2 = ctarray( *det_array )

                        # Pack that up into a SOBS message
                        radar_riff_message.tag = b'SOBS'
                        radar_riff_message.length = ctypes.sizeof( ctypes.c_float ) * det_array_count
                        radar_riff_message.data   = ctarray2
                        radar_riff_client.write( radar_riff_message );
                        #print( 'Sent Radar SOBS message with', det_array_count, 'entries of size', radar_riff_message.length )

            if want_lane_offsets and ((now - last_update_SLP_time) > 0.1):
                last_update_SLP_time = now

                debug_this_pass = False
                if want_lane_offset_debug and ((now - last_update_SLP_time_debug) > 0.5):
                    last_update_SLP_time_debug = now
                    debug_this_pass = True

                # Get vehicle position & bounding box and lane waypoint info
                try:
                    vtrans = vehicle.get_transform()
                    vbox = vehicle.bounding_box
                except:
                    print( 'Exception getting vehicle loc & bounding box' )
                    sys.exit( 1 )

                vbox.rotation = vtrans.rotation;
                vbox_ang = math.radians( (360 + vbox.rotation.yaw) % 360.0 )

                vbox_extent = vbox.extent
                arrow_len = vbox_extent.x;

                # Raise the box up from the base since carla isnt consistent with its location origins
                vbox.location = vtrans.location + carla.Location( z=vbox_extent.z );

                # Move the vbox vector start to the point where you want to pivot
                #   which is claimed to be the back wheels...
                #   but I dont believe it !
                #   You drive the bus to keep the front wheels in the lane.
                #   if your turning sharply however yuo want the middle of the bus not to breech the walls.
                #   Since we know the rate of turn, have it go from middle of the bus at high rate of turn
                #   to just ahead of the bus when going straight. I.e. add a vector on when going straight.
                turning = veh_steer * 3.0;      # in rads
                if turning < 0:
                    turning = -turning;         # want mag only
                if turning > 1.0:
                    turning = 1.0;              # clamp to 1, so about 1/3rd of a rad
                vbox_start = carla.Location( vbox.location );
                vbox_start.x += vbox_extent.x * 2.0 * (1.0 - turning) * math.cos( vbox_ang );
                vbox_start.y += vbox_extent.x * 2.0 * (1.0 - turning) * math.sin( vbox_ang );

                # Find end of vector as one arrow length from start
                vbox_end = carla.Location( vbox_start );
                vbox_end.x += arrow_len * math.cos( vbox_ang );
                vbox_end.y += arrow_len * math.sin( vbox_ang );

                # Find vehicle vector
                vi = vbox_end.x - vbox_start.x
                vj = vbox_end.y - vbox_start.y

                vdist2 = vi*vi + vj*vj
                vdist = 0.0
                if vdist2 > 0.0:
                    vdist = math.sqrt( vdist2 )
                    # Normalize this one
                    vi /= vdist
                    vj /= vdist
                else:
                    vdist = 0.0

                # Get closest projected lane way point to the vbox_start
                try:
                    ltype = carla.LaneType.Driving
                    veh_pivot = carla.Location( x = vbox_start.x, y = vbox_start.y, z = vbox_start.z )
                    wp = the_map.get_waypoint( veh_pivot, project_to_road=True, lane_type=ltype )
                except:
                    print( 'Exception getting waypoint for lane offsets' )
                    sys.exit( 1 )

                wp_ang = math.radians( (360 + wp.transform.rotation.yaw) % 360.0 )
                wp_start = wp.transform.location;
                # Set Z height to match vehicle
                wp_start.z = vbox_start.z
                wp_end = carla.Location( wp_start );
                wp_end.x += arrow_len * math.cos( wp_ang );
                wp_end.y += arrow_len * math.sin( wp_ang );

                li = wp_end.x - wp_start.x
                lj = wp_end.y - wp_start.y

                ldist2 = li*li + lj*lj
                ldist = 0.0
                if ldist2 > 0.0:
                    ldist = math.sqrt( ldist2 )
                    # Normalize this one too
                    li /= ldist
                    lj /= ldist
                else:
                    ldist = 0.0

                # Get vehicle to lane vector
                vli = wp_start.x - vbox_start.x
                vlj = wp_start.y - vbox_start.y

                vldist2 = vli*vli + vlj*vlj
                vldist = 0.0
                if vldist2 > 0.0:
                    vldist = math.sqrt( vldist2 )
                    # Dont normalize this one - were after the length
                else:
                    vldist = 0.0

                # vldist is magnitude so need to work out if right or left of centre. Needs to be +ve when left of lane
                # Carla is supposed to have projected the location to the lane so we dont have to do a dot prod
                # Normally we use right hand rule but I believe carla is working in a left handed axis set up
                #  so I think its opposite
                v_x_cl_k = vi*vlj - vj*vli;
                if v_x_cl_k < 0.0:
                    lane_debug_string = 'Veh is right of centre.'
                    vldist = -vldist;
                else:
                    lane_debug_string = 'Veh is left of centre.'

                ang_error = wp_ang - vbox_ang
                if ang_error > 3.14159:
                    ang_error = 3.14159 - ang_error
                if ang_error < -3.14159:
                    ang_error = ang_error + (3.14159 * 2.0)

                if debug_this_pass:
                    # Plot some arrows and boxes
                    wp_arrow_col = carla.Color( r=0, g=255, b=0 )
                    box_arrow_col = carla.Color( r=0, g=0, b=255 )
                    #world.debug.draw_box( vbox, vbox.rotation, life_time=30.0 );
                    world.debug.draw_arrow( wp_start, wp_end, arrow_size = arrow_len / 10.0, life_time = 30.0, color = wp_arrow_col );
                    world.debug.draw_arrow( vbox_start, vbox_end, arrow_size = arrow_len / 10.0, life_time = 30.0, color = box_arrow_col );

                    wp_deg   = round( wp_ang    * 180.0 / 3.14159, 1 )
                    v_deg    = round( vbox_ang  * 180.0 / 3.14159, 1 )
                    diff_deg = round( (wp_ang - vbox_ang) * 180.0 / 3.14159, 1 )
                    err_deg  = round( ang_error * 180.0 / 3.14159, 1 )
                    #print( "--------------------" )
                    #print( "VLDist:", round(vldist,3), lane_debug_string )
                    #print( "Angles: V:", v_deg, " W:", wp_deg, " Diff:", diff_deg, " Err:", err_deg )
                    #print( 'Send SLP: laneoff, hdgerr: ', round( vldist, 3 ), round( ang_error, 3 ) )
                    #print( 'VTrans: ', round( vtrans.location.x, 3 ), round( vtrans.location.y, 3 ), round( vtrans.rotation.yaw, 3 ) )

                # Return lane offset to the controller
                riff_MNC_SLP.timestamp = timestamp_offset + now

                riff_MNC_SLP.lateral_offset = vldist                  # From centre of lane
                riff_MNC_SLP.heading_error  = ang_error               # Rads error

                # Not so sure about the following...

                # If the angle error is huge (> 0.5 rads) then its probably a lane crossing and we should ignore it.
                mag_ang_err = ang_error;
                if mag_ang_err < 0.0:
                    mag_ang_err = -mag_ang_err;

                if mag_ang_err > 0.2:
                    riff_MNC_SLP.lateral_confidence = 0.0
                    riff_MNC_SLP.heading_confidence = 0.0
                else:
                    riff_MNC_SLP.lateral_confidence = 1.0
                    riff_MNC_SLP.heading_confidence = 1.0

                lane_riff_message.tag = b'SLP '
                lane_riff_message.length = ctypes.sizeof( riff_MNC_SLP )
                lane_riff_message.data   = ctypes.byref(  riff_MNC_SLP )
                lane_riff_client.write( lane_riff_message );

    except KeyboardInterrupt:
        print( 'Keyboard interrupt, exiting...' )

del client




