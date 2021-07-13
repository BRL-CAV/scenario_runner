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

# Set this to True if you just want the vehicle to auto drive around the map generating UTM coords
gather_gnss_coords_for_map = False
want_car_manual_control = False

# Set this if were using one of carlas built in maps which will then apply an offset
# to the lat/lng coords to position it away from 0.0/0.0 which is a daft place to have put it in
# the beginning
use_a_carla_built_in_map = True

gnss_tick_rate = 0.1
imu_tick_rate = 0.1

# Attach a Lane Invasion Detector sensor and display them
want_lane_invasions = False

# Want Lane Offsets & debugging
want_lane_offsets = False
want_lane_offset_debug = False

# Save images from front of car to dir
# NOTE - at present were using async mode and it takes time to save the images which kills the
#        system. When we get sync mode working then it will have the time to save out images and
#        then this should work.
want_car_camera_front = False
want_car_camera_debug_filenames = True
camera_frame_every = 0.25

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
if os.name == 'posix':
    if platform.node() == 'punch':
        # TODO - this will most likely need to be relative to this interface
        # but for some reason it dont grok with ~/Carla/...
        carla_dir = '/home/stevee/Carla/Carla_Export_Dists'
        print( 'Setting up for Punch using Carla version ' + carla_version )
        carla_egg = carla_dir + '/PythonAPI/carla/dist/carla-' + carla_version + '-py3.6-linux-x86_64.egg'
        sys.path.append( carla_egg )
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
                 ('heading_error'     , ctypes.c_float   ),  # Degrees, positive when heading left of route
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
class SetupCarla:
    def __init__(self):
        vehicle = None
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
                #req_map = '/Game/Carla/Maps/Town01'
                #req_map = '/Game/Carla/Maps/Town02'
                req_map = '/Game/Carla/Maps/Town03'
                #req_map = '/Game/Carla/Maps/Town04'
                #req_map = '/Game/Carla/Maps/Town05'
            else:
                req_map = '/Game/MWayShortFlat/Maps/MWayShortFlat/MWayShortFlat'

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
            # You can get here if the XODR file was not included in the package.
            # You may need to manually get it and add it into the expanded package.
            sys.exit( 1 )

        try:
            vehicles = world.get_actors().filter('vehicle.seat.leon')
            if len( vehicles ) == 0:
                print( 'Spawn a new vehicle, look for a blueprint.' )
                vehicle_bp = world.get_blueprint_library().filter('vehicle.seat.leon')[0]
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
        except Exception:
            print( 'Exception from creating vehicle' )
            sys.exit( 1 )

        try:
            print( 'Perform first tick' ) # so that the set vehicle locations can be get from them.
            world.tick();
            vehicles = world.get_actors().filter( 'vehicle.seat.leon' )
            print( 'Number of vehicles in sim:', len( vehicles ) )
            vehicle = vehicles[0]
            veh_trans = vehicle.get_transform()
            print( 'Vehicle is at position:', veh_trans )
        except Exception:
            print( 'Exception: Setting world first tick' )
            sys.exit( 1 )

        try:
            print( 'Setting camera high above the vehicle and pointing downwards at it.' )
            viewer = world.get_spectator()
            view_trans = veh_trans
            view_trans.location.z = 100
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
                self.gnss = gnss
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

                self.imu = imu
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

                    self.lid = lid
                except Exception as ex:
                    print('Exception: Lane Invasion Sensor:', ex )
                    sys.exit( 1 )

            if want_car_camera_front:
                try:
                    print( 'Set up Car Camera' )
                    ccam_bp = world.get_blueprint_library().find( 'sensor.camera.rgb' )
                    ccam_rate = "%f" %(camera_frame_every)
                    ccam_bp.set_attribute( 'sensor_tick', ccam_rate )
                    ccam_bp.set_attribute( 'image_size_x', '720' )
                    ccam_bp.set_attribute( 'image_size_y', '576' )
                    ccam_bp.set_attribute( 'fov', '90' )
                    # Set position. Put Z a couple m above car else you wont see much
                    ccam_trans = carla.Transform( carla.Location( x=0.8, y=0.0, z=2.0 ) )
                    ccam = world.spawn_actor( ccam_bp, ccam_trans, attach_to=vehicle )
                    ccam.listen( lambda data: ccam_queue.put( data ) )
                except Exception as ex:
                    print('Exception: Camera:', ex )
                    sys.exit( 1 )

            if want_radar_sensor:
                try:
                    print( 'Set up Radar Sensor' )
                    radar_bp = world.get_blueprint_library().find( 'sensor.other.radar' )
                    print( "Radar", radar_bp )
                    radar_bp.set_attribute( 'horizontal_fov',    '40.0'  )
                    radar_bp.set_attribute( 'vertical_fov',      '12.0'  )
                    radar_bp.set_attribute( 'range',             '150.0' )
                    radar_bp.set_attribute( 'points_per_second', '1000' )
                    radar_bp.set_attribute( 'sensor_tick',       '1.0'   )
                    # Set position. Put Z a couple m above car else you wont see much
                    radar_trans = carla.Transform( carla.Location( x=0.0, y=2.0, z=1.0 ) )
                    radar = world.spawn_actor( radar_bp, radar_trans, attach_to=vehicle )
                    radar.listen( lambda data: radar_queue.put( data ) )

                    self.radar = radar
                except Exception as ex:
                    print('Exception: Radar:', ex )
                    sys.exit( 1 )

        except Exception:
            print( 'Exception setting up sensors' )
            sys.exit( 1 )

        self.world = world
        self.the_map = the_map
        self.vehicle = vehicle


    def get(self):
        return (self.world, self.the_map, self.vehicle )
####################################################################################################
#
# Image saver in thread
# Attempt to put the processing of converting image and saving it to another core so that it wont
# hang the main thread. But it doesnt work well enough to make async mode work with it.
# Still probably better than not doing it this way tho.
#
####################################################################################################
def ImageSaveThread():
    while 1:
        if ccam_queue.qsize() > 0:
            if want_car_camera_debug_filenames:
                if tron:
                    print( 'Pre Cam save' )
            data = ccam_queue.get()
            if data:
                # Save raw_data as image using next_frame_num in preset dir
                fn = "./cam_frames/Fr_%06d.png" % (next_cam_frame)
                if want_car_camera_debug_filenames:
                    print( 'Saving frame <',fn,'>' )
                data.save_to_disk( fn )
                next_cam_frame = next_cam_frame + 1
                if want_car_camera_debug_filenames:
                    if tron:
                        print( 'Saved frame' )
            print( 'Post Cam save' )


####################################################################################################
#
# Run Carla Sim in a loop
#
####################################################################################################
if want_car_camera_front:
    # Kick of thread to save camera images to disk
    print( 'Starting Camera Image Save thread' )
    cam_saver_pid = os.fork()
    if cam_saver_pid == 0:
        ImageSaveThread()
        os.exit(0)

# Do you want to just collect up UTM coords of the map or drive it under sim or keyboard control
def RunSim(world, the_map, vehicle):
    global gnss_riff_message
    print( "Run the vehicle under Fusion CAV Sim control\n" )
    steering_lock_angle = 35
    map_centre_latitude = 49

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
    veh_wheelbase = 2.5
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
                        # print( 'Read SVC : ReqThrot:', round( req_throttle, 4), '  ReqStT:', round( req_steer_torque, 6), '  ReqBr:', round( req_brake,2 ) )
                    else:
                        print( 'Unexpected vehicle controller motion length', length )

            if now - last_update_SVF_time > 0.1:
                dt = now - last_update_SVF_time;
                last_update_SVF_time = now

                inc_dist = veh_spd * dt;
                distance_left  = distance_left  + inc_dist
                distance_right = distance_right + inc_dist
                turning_angle = veh_steer * steering_lock_angle * 3.14159 / 180.0
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
		# Integrate torque with dt - It should take 1 second to get from centre to full lock at maximum torque

                veh_throttle = req_throttle
                veh_brake    = req_brake
                veh_steer    += req_steer_torque * dt * 5

                veh_throttle = min( 1.0, max(  0.0, veh_throttle ) )
                veh_brake    = min( 1.0, max(  0.0, veh_brake ) )
                veh_steer    = min( 1.0, max( -1.0, veh_steer ) )

                #if tron:
                #    print( 'Throttle:', round( veh_throttle, 2), '   Steer:', round( veh_steer * steering_lock_angle, 1), '  Brake:', round( veh_brake,2 ) )

                vehicle.apply_control( carla.VehicleControl( throttle = veh_throttle, steer = -1.0 * veh_steer, brake = veh_brake ) )

                # Must feedback state of the motion to the controller in order to set the heading and the timestamp for it
                riff_MC_SVF.timestamp = timestamp_offset + now
                riff_MC_SVF.distance_left = distance_left
                riff_MC_SVF.distance_right = distance_right
                riff_MC_SVF.turning_angle = veh_steer * steering_lock_angle * 3.14159 / 180.0
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
                if want_lane_offset_debug:
                    print( "" )

                # Calculate lane offsets
                try:
                    lane_debug_string = ''
                    vtrans = vehicle.get_transform()
                    if want_lane_offset_debug:
                        print( "Veh Pos :", round(vtrans.location.x,1), round(vtrans.location.y,1) )
                    ltype = carla.LaneType.Driving
                    wp = the_map.get_waypoint( vtrans.location, project_to_road=True, lane_type=ltype )
                except:
                    print( 'Exception (1) getting waypoint for lane offsets' )

                try:
                    # Distance from centre of lane
                    lane_x = wp.transform.location.x
                    lane_y = wp.transform.location.y
                    cli = lane_x - vtrans.location.x
                    clj = lane_y - vtrans.location.y
                    if want_lane_offset_debug:
                        print( "Lane Pos:", round(lane_x,1), round(lane_y,1) )
                except:
                    print( 'Exception (2) getting transform for lane offsets' )

                try:
                    ldist2 = cli*cli + clj*clj
                    if ldist2 > 0.0:
                        ldist = math.sqrt( ldist2 )
                        cli /= ldist
                        clj /= ldist
                    else:
                        ldist = 0.0

                    if want_lane_offset_debug:
                        print( "Lane Dist:", round(ldist,1) )

                    # Veh vector
                    vhdgrad = ((90 + 360 + vtrans.rotation.yaw) % 360) * 3.14159 / 180.0
                    if want_lane_offset_debug:
                        print( "VehHdg:", round((vhdgrad * 180.0/3.14159),1) )
                    vi = math.sin( vhdgrad )
                    vj = math.cos( vhdgrad )

                    v_x_cl_k = vi*clj - vj*cli;
                    if v_x_cl_k > 0.0:
                        lane_debug_string = lane_debug_string + ' Veh is right of centre.'
                        ldist = ldist + wp.lane_width / 2.0
                        if want_lane_offset_debug:
                            print( "Adding half lane width of:", round( wp.lane_width / 2.0, 3 ) )
                    else:
                        lane_debug_string = lane_debug_string + ' Veh is left of centre.'
                        ldist = wp.lane_width / 2.0 - ldist
                        if want_lane_offset_debug:
                            print( "Difference from half lane:", round( wp.lane_width / 2.0, 3 ) )
                except:
                    print( 'Exception (3) getting rotations or math for lane offsets' )

                if want_lane_offset_debug:
                    print( "Lane Dist now:", round(ldist,1) )

                try:
                    # Lane vector
                    lhdgrad = ((90 + 360 + wp.transform.rotation.yaw) % 360) * 3.14159 / 180.0
                    if want_lane_offset_debug:
                        print( "LaneHdg:", round((lhdgrad * 180.0/3.14159),1) )
                    li = math.sin( lhdgrad )
                    lj = math.cos( lhdgrad )
                except:
                    print( 'Exception (4) getting transform or math for lane offsets' )

                try:
                    dot = vi * li + vj * lj
                    if want_lane_offset_debug:
                        print( "VdotL:", round(dot,6) )
                except:
                    print( 'Exception (5) getting heading error for lane offsets' )

                try:
                    if dot == 0.0:
                        lane_debug_string = lane_debug_string + ' Vehicle is tangent to lane.'
                    elif dot < 0.0:
                        lane_debug_string = lane_debug_string + ' Wrong way.'
                        vldist = ldist / (dot * -1.0)
                        lane_debug_string = lane_debug_string + ' Seen from veh, left side = ' + str( round(vldist,3) )
                    else:
                        vldist = ldist / dot
                        lane_debug_string = lane_debug_string + ' Seen from veh, left side = ' + str( round(vldist,3) )

                    if want_lane_offset_debug:
                        print( lane_debug_string )
                except:
                    print( 'Exception (6) lane offsets' )

                heading_error = vyawrad - lyawrad;
                if heading_error > 3.14159:
                    heading_error = (3.14159 * 2.0) - heading_error
                heading_error = heading_error * 180.0 / 3.14159
                if want_lane_offset_debug:
                    print( "HdgErr:", round(heading_error,1) )

                # Return lane offset to the controller
                riff_MNC_SLP.timestamp = timestamp_offset + now
                riff_MNC_SLP.lateral_offset = vldist
                # Not so sure about the following...
                riff_MNC_SLP.lateral_confidence = 1.0
                riff_MNC_SLP.heading_error = heading_error
                riff_MNC_SLP.heading_confidence = 1.0

                lane_riff_message.tag = b'SLP '
                lane_riff_message.length = ctypes.sizeof( riff_MNC_SLP )
                lane_riff_message.data   = ctypes.byref(  riff_MNC_SLP )
                lane_riff_client.write( lane_riff_message );
                if want_lane_offset_debug:
                    print( 'Send SLP: laneoff, hdgerr: ', round(vldist,3), round(heading_error,1) )

    except KeyboardInterrupt:
        print( 'Keyboard interrupt, exiting...' )


