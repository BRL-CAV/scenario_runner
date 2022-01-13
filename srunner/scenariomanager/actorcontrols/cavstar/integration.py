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
server_ip_addr = '91.227.124.10'
#server_ip_addr = '127.0.0.1'

# Were doing synchronous time
carla_time = 0
carla_time_delta = 1.0 / 30.0
cavstar_time = 0

# Name of car / bus model
vehicle_blueprint_name = 'vehicle.brl.bus'

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
want_lane_offset_debug = False

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
want_radar_sensor = False

# Want other static actors in the way
want_other_objects = True


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
    SCENARIO_RUNNER_ROOT = os.getenv('SCENARIO_RUNNER_ROOT')
    sys.path.append( SCENARIO_RUNNER_ROOT + '/srunner/scenariomanager/actorcontrols/cavstar' )

    if platform.node() == 'punch':
        # TODO - this will most likely need to be relative to this interface
        # but for some reason it dont grok with ~/Carla/...
        carla_dir = '/home/stevee/Carla/Carla_BRL_BusModel/LinuxNoEditor'
        carla_map_path = '/Game/map_package/Maps/'
        print( 'Setting up for Punch using Carla version ' + carla_version )
        carla_egg = carla_dir + '/PythonAPI/carla/dist/carla-' + carla_version + '-py3.6-linux-x86_64.egg'
        sys.path.append( carla_egg )
        can_set_gnss_tick = 1
    elif platform.node() == 'zotac':
        carla_dir = '/home/tims/LinuxNoEditor'
        carla_map_path = '/Game/map_package/Maps/'
        print( 'Setting up for Zotac using Carla version ' + carla_version )
        carla_egg = carla_dir + '/PythonAPI/carla/dist/carla-' + carla_version + '-py3.6-linux-x86_64.egg'
        sys.path.append( '/home/saquib/work/scenario_runner/srunner/scenariomanager/actorcontrols/cavstar' )
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
import math, time, re
import rifflib, ctypes
from datetime import datetime, timedelta
from random import *

#try:
#    import pygame
#    from pygame.locals import K_LEFT
#    from pygame.locals import K_RIGHT
#    from pygame.locals import K_UP
#    from pygame.locals import K_DOWN
#    from pygame.locals import K_SPACE
#    from pygame.locals import K_1
#    from pygame.locals import K_2
#    from pygame.locals import K_3
#    from pygame.locals import K_4
#    from pygame.locals import K_5
#    from pygame.locals import K_6
#    from pygame.locals import K_7
#    from pygame.locals import K_8
#    from pygame.locals import K_9
#    from pygame.locals import K_0
#except ImportError:
#    raise RuntimeError( 'Cant import pygame' )

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
                 ('padding'           , ctypes.c_int   ) ]

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

# Sim Time "SIMT"
class Riff_MC_SIMT(ctypes.Structure):
    _fields_ = [ ('time'              , ctypes.c_double  ) ]  # Sim Time

# Data structures transfered to Main Non Critical (The sensor interface)
riff_MNC_SOBS = Riff_MNC_SOBS()
riff_MNC_SLP  = Riff_MNC_SLP()
riff_MNC_STL  = Riff_MNC_STL()
riff_MNC_SGLH = Riff_MNC_SGLH()
riff_MNC_SMSD = Riff_MNC_SMSD()

# Data structures transfered to and from Main Critical (The vehicle Interface)
riff_MC_SVC  = Riff_MC_SVC()
riff_MC_SVF  = Riff_MC_SVF()
riff_MC_SIMT = Riff_MC_SIMT()

gnss_riff_message = rifflib.Message()
mems_riff_message = rifflib.Message()
lane_riff_message = rifflib.Message()
radar_riff_message = rifflib.Message()
other_objs_riff_message = rifflib.Message()
veh_control_riff_message  = rifflib.Message()
simtime_riff_message  = rifflib.Message()

# Initialise the RIFF clients for the ADS to connect to
gnss_riff_client         = rifflib.RiffClient( 6163, server_ip_addr, 5, 'carla non critical gnss' )
mems_riff_client         = rifflib.RiffClient( 6164, server_ip_addr, 5, 'carla non critical mems' )
lane_riff_client         = rifflib.RiffClient( 6166, server_ip_addr, 5, 'carla non critical lane' )
radar_riff_client        = rifflib.RiffClient( 6167, server_ip_addr, 5, 'carla non critical radar' )
veh_control_riff_client  = rifflib.RiffClient( 6168, server_ip_addr, 5, 'carla critical veh ctl' )
simtime_riff_client      = rifflib.RiffClient( 6170, server_ip_addr, 5, 'carla critical simtime' )


####################################################################################################
#
#  Get time from start of sim
#
####################################################################################################
start_time = time.time()

# Work out how many seconds have elapsed since the start of the week since this offset must be
# added to the timestamps of the messages (esp GNSS types). This is only done once at start of app
time_now = datetime.now()
dow = time_now.isoweekday()
delta = timedelta( days = -dow )
sow = time_now + delta
time_sow = datetime( sow.year, sow.month, sow.day, 0,0,0 )
diff_sow = time_now - time_sow
timestamp_offset = diff_sow.days * 86400 + diff_sow.seconds

def GetRealMsFromStart():
    global start_time
    return int( (time.time() - start_time) * 1000.0 )


####################################################################################################
#
#  GNSS
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
        dist2 = (utmx_in - self.utm_x)*(utmx_in - self.utm_x) + (utmy_in - self.utm_y)*(utmy_in - self.utm_y)
        if dist2 > 0.0001:
            return math.sqrt( dist2 )
        else:
            return 0.0

    def PrintUtm( self ):
        print( f'GNSS: ({self.lat: 12.7f}, {self.lng: 12.7f}) -> ({self.utm_x: 12.3f},{self.utm_y: 12.3f})' )

vehicle_gnss = Gnss()

gnss_queue = queue.Queue(1)
imu_queue  = queue.Queue(1)
lid_queue  = queue.Queue(1)
ccam_queue = queue.Queue(1)
radar_queue = queue.Queue(1)

next_cam_frame = 1


####################################################################################################
#
#  Since there is no reverse of map.transform_to_geolocation to go from lat long to
#  map coords, we need to run a loop until were close enough
#  Definitely not optimized ! But when used, this is only done once
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
#  Set up Cavstar
#
####################################################################################################
def SetupCavstar(client, world, the_map, vehicle):
    try:
        print( 'The available maps for this Carla server are:' )
        print( client.get_available_maps() )

    except Exception:
        print( 'Exception: Could not set the requested Carla world map.' )
        sys.exit( 1 )

    try:
        print( 'Setting world parameters' )
        world_settings = world.get_settings()
        world_settings.synchronous_mode = True
        world_settings.fixed_delta_seconds = carla_time_delta
        world_settings.substepping = True
        world_settings.max_substep_delta_time = 0.01
        world_settings.max_substeps = 16
        world.apply_settings( world_settings )
    except Exception:
        print( 'Exception: Setting world parameters.' )
        sys.exit( 1 )

    try:
        print( 'Perform first tick' ) # so that the set vehicle locations can be obtained from them.
        world.tick()
        veh_trans = vehicle.get_transform()
    except Exception:
        print( 'Exception: Setting world first tick' )
        sys.exit( 1 )

    try:
        print( 'Setting camera high above the vehicle and pointing downwards at it.' )
        viewer = world.get_spectator()
        view_trans = veh_trans
        view_trans.location.z = 50
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

    return (viewer, )


####################################################################################################
#
#  Set up Sensors
#
####################################################################################################
class SetupSensors:
    def __init__(self, client, world, vehicle):
        self.gnss = None
        self.imu = None
        self.lid = None
        self.ccam = None
        self.radar = None

        try:
            print( 'Set up sensor and attach to vehicle' )
            try:
                print( 'Set up GNSS' )
                gnss_bp = world.get_blueprint_library().find( 'sensor.other.gnss' )
                if can_set_gnss_tick:
                    print( 'Setting GNSS tick to 0.1' )
                    gnss_bp.set_attribute( 'sensor_tick', str(gnss_tick_rate) )
                gnss_trans = carla.Transform( carla.Location( x=0.0, y=0.0, z=0.0 ) )
                gnss = world.spawn_actor( gnss_bp, gnss_trans, attach_to=vehicle )
                gnss.listen( lambda data: gnss_queue.put( data ) )

                self.gnss = gnss
                # sensors.attach_gnss(world, vehicle, gnss_queue, can_set_gnss_tick, gnss_tick_rate)
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
                    ccam_bp.set_attribute( 'image_size_x',  '320' )
                    ccam_bp.set_attribute( 'image_size_y',  '240' )
                    ccam_bp.set_attribute( 'fov',           '90' )
                    ccam_bp.set_attribute( 'exposure_mode', 'manual' )
                    ccam_bp.set_attribute( 'shutter_speed', '250.0' )
                    # Set position. Put Z a couple m above car else you wont see much
                    ccam_trans = carla.Transform( carla.Location( x=0.8, y=0.0, z=2.0 ) )
                    ccam = world.spawn_actor( ccam_bp, ccam_trans, attach_to=vehicle )
                    ccam.listen( lambda image: ccam_queue.put( image ) )

                    self.ccam = ccam
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

                    self.radar = radar
                except Exception as ex:
                    print('Exception: Radar:', ex )
                    sys.exit( 1 )

        except Exception:
            print( 'Exception setting up sensors' )
            sys.exit( 1 )


####################################################################################################
#
#  Run Carla Sim in a loop
#
####################################################################################################
steering_lock_angle_rad = 55 * 3.14159 / 180
speed_error_integral = 0

veh_utm_x = 0.0
veh_utm_y = 0.0
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

last_sent_SIMT = 0.0



def RiffClients():
    gnss_riff_client.connect()
    mems_riff_client.connect()
    lane_riff_client.connect()
    radar_riff_client.connect()
    veh_control_riff_client.connect()
    simtime_riff_client.connect()

    # Were not expecting to receive anything from these only write back to controller so keep them flushed
    gnss_riff_message = gnss_riff_client.receive()
    mems_riff_message = mems_riff_client.receive()
    lane_riff_message = lane_riff_client.receive()
    radar_riff_message = radar_riff_client.receive()


def WaitForStart(world, vehicle):
    global cavstar_time
    global carla_time
    global carla_time_delta
    global steering_lock_angle_rad
    global speed_error_integral
    global veh_utm_x
    global veh_utm_y
    global veh_hdg
    global timestamp_offset

    global simtime_riff_client
    global simtime_riff_message
    global last_sent_SIMT

    try:
        world.tick()
    except Exception as ex:
        print('Exception: In WaitForStart, Tick:', ex )
        sys.exit( 1 )

    carla_time = 0.0
    cavstar_time = carla_time
    print( 'Initial Carla time:', carla_time )

    latitude  = 0.0
    longitude = 0.0
    if gnss_queue.qsize() > 0:
        data = gnss_queue.get()
        if data:
            latitude  = data.latitude
            longitude = data.longitude

    print( 'WaitForStart synchronous mode. Wait for simtime client to connect' )
    while True:
        RiffClients()
        if simtime_riff_client.connected():
            break

    print( 'WaitForStart synchronous mode. Simtime client connected. Send initial carla time to it.' )
    riff_MC_SIMT.time = carla_time
    simtime_riff_message.tag = b'SIMT'
    simtime_riff_message.length = ctypes.sizeof( riff_MC_SIMT )
    simtime_riff_message.data   = ctypes.byref(  riff_MC_SIMT )
    simtime_riff_client.write( simtime_riff_message )
    last_sent_SIMT = GetRealMsFromStart() / 1000.0

    # Before get going proper:
    #  Get cavstars current simtime
    #  Send a message about where the car is.
    print( 'Wait for starting conditions' )
    wait_for_start = 0
    while wait_for_start != 3:
        real_now = GetRealMsFromStart() / 1000.0
        RiffClients()

        if (wait_for_start & 2) == 0:
            if real_now - last_sent_SIMT > 1.0:
                riff_MC_SIMT.time = carla_time
                simtime_riff_message.tag = b'SIMT'
                simtime_riff_message.length = ctypes.sizeof( riff_MC_SIMT )
                simtime_riff_message.data   = ctypes.byref(  riff_MC_SIMT )
                simtime_riff_client.write( simtime_riff_message )
                print( 'Sending out simtime (Carla Time) as:', carla_time )
                last_sent_SIMT = real_now

            try: simtime_riff_message = simtime_riff_client.receive()
            except:
                print( 'No connection to simtime' )
                pass # Continue if the connection is lost

            length = simtime_riff_message.length.value
            data = simtime_riff_message.data
            if simtime_riff_message.tag.value == b'SIMT':
                if length == ctypes.sizeof( riff_MC_SIMT ):
                    ctypes.memmove( ctypes.byref( riff_MC_SIMT ), data, length )
                    cavstar_time = riff_MC_SIMT.time
                    carla_time = cavstar_time
                    print( 'Initial CavStar/Carla Time is:', cavstar_time )
                    wait_for_start |= 2
                else:
                    print( 'SIMT length bad' )
        else:
            wait_for_start |= 2


        if (wait_for_start & 1) == 0:
            utm_x, utm_y, utm_zone, utm_letter = utm.from_latlon( latitude, longitude )

            veh_utm_x = utm_x + 3 # Offset GNSS east to align carla world with real world
            veh_utm_y = utm_y
            veh_hdg = (90 + 360 + vehicle.get_transform().rotation.yaw) % 360

            # Return position and heading to the controller
            riff_MNC_SGLH.utm_northing = veh_utm_y
            riff_MNC_SGLH.utm_easting = veh_utm_x
            riff_MNC_SGLH.heading = veh_hdg
            riff_MNC_SGLH.heading_rate = 0
            riff_MNC_SGLH.timestamp = timestamp_offset + carla_time

            gnss_riff_message.tag = b'SGLH'
            gnss_riff_message.length = ctypes.sizeof( riff_MNC_SGLH )
            gnss_riff_message.data   = ctypes.byref(  riff_MNC_SGLH )
            gnss_riff_client.write( gnss_riff_message )

            wait_for_start |= 1
    print( 'Finished starting conditions' )



####################################################################################################
#
#  Run a time step
#
####################################################################################################
def RunStep(world, the_map, viewer, vehicle):
    global cavstar_time
    global carla_time
    global carla_time_delta
    global steering_lock_angle_rad
    global speed_error_integral
    global veh_utm_x
    global veh_utm_y
    global veh_spd
    global veh_hdg
    global veh_throttle
    global veh_brake
    global veh_steer
    global gnss_ms
    global req_throttle
    global req_brake
    global req_steer_torque
    global vldist
    global heading_error
    global vyawrad
    global lyawrad
    global distance_left
    global distance_right
    global veh_wheelbase
    global veh_width
    global wheel_utm_x
    global wheel_utm_y
    global timestamp_offset

    global gnss_riff_client
    global mems_riff_client
    global lane_riff_client
    global radar_riff_client
    global veh_control_riff_client
    global simtime_riff_client

    global gnss_riff_message
    global mems_riff_message
    global lane_riff_message
    global radar_riff_message
    global other_objs_riff_message
    global veh_control_riff_message
    global simtime_riff_message

    global last_sent_SIMT

    try:
        start_frame_time = GetRealMsFromStart() / 1000.0

        RiffClients()

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
                    #if req_throttle:
                    #    print("----", req_throttle)
                    #print( 'Read SVC : ReqThrot:', round( req_throttle, 4), '  ReqStT:', round( req_steer_torque, 6), '  ReqBr:', round( req_brake,2 ) )
                else:
                    print( 'Unexpected vehicle controller motion length', length )


        inc_dist = veh_spd * carla_time_delta
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

        # A torque of 1.0 would one radian per second on the van, but limited to 0.35
        road_wheel_angle_rate = min( 0.35, max( -0.35, req_steer_torque ) )

        # Need to convert angle rate to steer angle
        # Integrate angle rate with carla_time_delta
        veh_steer += road_wheel_angle_rate * carla_time_delta / steering_lock_angle_rad

        veh_throttle = min( 1.0, max(  0.0, req_throttle ) )
        veh_brake    = min( 1.0, max(  0.0, req_brake ) )
        veh_steer    = min( 1.0, max( -1.0, veh_steer ) )

        #if tron:
        #    print( 'Throttle:', round( veh_throttle, 2), '   Steer:', round( veh_steer * steering_lock_angle_rad, 1), '  Brake:', round( veh_brake,2 ) )

        # print(veh_throttle, -1.0 * veh_steer, veh_brake)
        vehicle.apply_control( carla.VehicleControl( throttle = veh_throttle, steer = -1.0 * veh_steer, brake = veh_brake ) )

        # Must feedback state of the motion to the controller in order to set the heading and the timestamp for it
        riff_MC_SVF.timestamp = timestamp_offset + carla_time
        riff_MC_SVF.distance_left = distance_left
        riff_MC_SVF.distance_right = distance_right
        riff_MC_SVF.turning_angle = veh_steer * steering_lock_angle_rad
        riff_MC_SVF.padding = 0.0

        veh_control_riff_message.tag = b'SVF '
        veh_control_riff_message.length = ctypes.sizeof( riff_MC_SVF )
        veh_control_riff_message.data   = ctypes.byref(  riff_MC_SVF )
        veh_control_riff_client.write( veh_control_riff_message )
        #print( "Sent distance left,right as", round(distance_left,1), round(distance_right,1) )
        #print( 'Send SVF : turn_angle (in rads):', round( riff_MC_SVF.turning_angle,3) )

        world_settings = world.get_settings()
        carla_time_delta = world_settings.fixed_delta_seconds

        carla_time += carla_time_delta
        #print( 'Carla time now:', carla_time )

        #print( 'Wait for simtime client to connect' )
        while True:
            if simtime_riff_client.connected():
                break
            else:
                RiffClients()

        #print( 'Send cavstar new carla_time.' )
        riff_MC_SIMT.time = carla_time
        simtime_riff_message.tag = b'SIMT'
        simtime_riff_message.length = ctypes.sizeof( riff_MC_SIMT )
        simtime_riff_message.data   = ctypes.byref(  riff_MC_SIMT )
        simtime_riff_client.write( simtime_riff_message )
        #print( 'Sending out Simtime as ', carla_time )

        if cavstar_time < carla_time:
            #print( 'RunStep: Wait whilst cavstar time is not yet carla_time:', cavstar_time, carla_time )
            sim_time = 0
            while sim_time < carla_time:
                simtime_riff_message = simtime_riff_client.receive()
                length = simtime_riff_message.length.value
                if simtime_riff_message.tag.value == b'SIMT' and length == ctypes.sizeof( riff_MC_SIMT ):
                    data = simtime_riff_message.data
                    ctypes.memmove( ctypes.byref( riff_MC_SIMT ), data, length )
                    sim_time = riff_MC_SIMT.time
                    #print( 'Got SimTime:', sim_time )
            cavstar_time = sim_time
            #print( 'CavStar and Carla times now:', cavstar_time, carla_time )

        if want_viewer_nailed_to_front_of_vehicle:
            veh_trans = vehicle.get_transform()
            view_trans = veh_trans
            veh_ang = math.radians( (360 + veh_trans.rotation.yaw) % 360.0 )
            vi = math.cos( veh_ang )
            vj = math.sin( veh_ang )
            view_trans.location.x = veh_trans.location.x + 2.0 * vi
            view_trans.location.y = veh_trans.location.y + 2.0 * vj
            view_trans.location.z = veh_trans.location.z + 3.0
            viewer.set_transform( view_trans )

        # Receive information from Carla and send to ADS
        if gnss_queue.qsize() > 0:
            data = gnss_queue.get()
            if data:
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

                veh_utm_x = utm_x + 3 # Offset GNSS east to align carla world with real world
                veh_utm_y = utm_y

                veh_hdg = (90 + 360 + vehicle.get_transform().rotation.yaw) % 360

                # Use Carla velocity for current speed
                vel = vehicle.get_velocity()
                veh_spd2 = vel.x * vel.x + vel.y * vel.y
                if veh_spd2 > 0.0:
                    veh_spd = math.sqrt( veh_spd2 )
                else:
                    veh_spd = 0.0

                #print( 'Speed:', veh_spd )

        if imu_queue.qsize() > 0:
            data = imu_queue.get()
            if data:

                #veh_hdg = math.degrees( data.compass )

                # Print sources of heading rate for comparison
                #print( carla_time, veh_utm_x, veh_utm_y, veh_spd, veh_hdg, veh_steer,
                #    math.degrees( data.compass ), -data.gyroscope.z )

                riff_MNC_SMSD.timestamp = timestamp_offset + carla_time
                riff_MNC_SMSD.forward   = -data.accelerometer.x
                riff_MNC_SMSD.left      = -data.accelerometer.y
                riff_MNC_SMSD.up        = data.accelerometer.z
                riff_MNC_SMSD.yaw       = -data.gyroscope.z
                riff_MNC_SMSD.pitch     = -data.gyroscope.y
                riff_MNC_SMSD.roll      = -data.gyroscope.x

                mems_riff_message.tag = b'SMSD'
                mems_riff_message.length = ctypes.sizeof( riff_MNC_SMSD )
                mems_riff_message.data   = ctypes.byref(  riff_MNC_SMSD )
                mems_riff_client.write( mems_riff_message )

                #turn = veh_steer * steering_lock_angle_rad
                #yaw_steer = veh_spd * math.tan( turn ) / veh_wheelbase
                #print( 'GyroZ/Yaw/s, turn, Spd, TA/s:', round( -data.gyroscope.z,5 ), round( turn, 5), round( veh_spd,3 ), round( yaw_steer, 5 ) )
                #print( 'Send SMSD: Acc, Gyro: ', round(data.accelerometer.x,3), round(data.accelerometer.y,3), round(data.accelerometer.z,3), round(data.gyroscope.x,3), round(data.gyroscope.y,3), round(data.gyroscope.z,3) )

        heading_rate = (veh_hdg - riff_MNC_SGLH.heading) / carla_time_delta

        # Return position and heading to the controller
        riff_MNC_SGLH.utm_northing = veh_utm_y
        riff_MNC_SGLH.utm_easting = veh_utm_x
        riff_MNC_SGLH.heading = veh_hdg
        riff_MNC_SGLH.heading_rate = heading_rate
        riff_MNC_SGLH.timestamp = timestamp_offset + carla_time

        gnss_riff_message.tag = b'SGLH'
        gnss_riff_message.length = ctypes.sizeof( riff_MNC_SGLH )
        gnss_riff_message.data   = ctypes.byref(  riff_MNC_SGLH )
        gnss_riff_client.write( gnss_riff_message )
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
                len_det = len(data)
                det_array = []
                det_array_count = 0
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
                    radar_riff_client.write( radar_riff_message )
                    #print( 'Sent Radar SOBS message with', det_array_count, 'entries of size', radar_riff_message.length )

        if want_lane_offsets:
            debug_this_pass = False
            if want_lane_offset_debug:
                debug_this_pass = True

            # Get vehicle position & bounding box and lane waypoint info
            try:
                vtrans = vehicle.get_transform()
                vbox = vehicle.bounding_box
            except:
                print( 'Exception getting vehicle loc & bounding box' )
                sys.exit( 1 )

            vbox.rotation = vtrans.rotation
            vbox_ang = math.radians( (360 + vbox.rotation.yaw) % 360.0 )

            vbox_extent = vbox.extent
            arrow_len = vbox_extent.x

            # Raise the box up from the base since carla isnt consistent with its location origins
            vbox.location = vtrans.location + carla.Location( z=vbox_extent.z )

            # Move the vbox vector start to the point where you want to pivot
            #   which is claimed to be the back wheels...
            #   but I dont believe it !
            #   You drive the bus to keep the front wheels in the lane.
            #   if your turning sharply however yuo want the middle of the bus not to breech the walls.
            #   Since we know the rate of turn, have it go from middle of the bus at high rate of turn
            #   to just ahead of the bus when going straight. I.e. add a vector on when going straight.
            turning = veh_steer * 3.0      # in rads
            if turning < 0:
                turning = -turning         # want mag only
            if turning > 1.0:
                turning = 1.0              # clamp to 1, so about 1/3rd of a rad
            vbox_start = carla.Location( vbox.location )
            vbox_start.x += vbox_extent.x * 2.0 * (1.0 - turning) * math.cos( vbox_ang )
            vbox_start.y += vbox_extent.x * 2.0 * (1.0 - turning) * math.sin( vbox_ang )

            # Find end of vector as one arrow length from start
            vbox_end = carla.Location( vbox_start )
            vbox_end.x += arrow_len * math.cos( vbox_ang )
            vbox_end.y += arrow_len * math.sin( vbox_ang )

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
            wp_start = wp.transform.location
            # Set Z height to match vehicle
            wp_start.z = vbox_start.z
            wp_end = carla.Location( wp_start )
            wp_end.x += arrow_len * math.cos( wp_ang )
            wp_end.y += arrow_len * math.sin( wp_ang )

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
            v_x_cl_k = vi*vlj - vj*vli
            if v_x_cl_k < 0.0:
                lane_debug_string = 'Veh is right of centre.'
                vldist = -vldist
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
                #world.debug.draw_box( vbox, vbox.rotation, life_time=30.0 )
                world.debug.draw_arrow( wp_start, wp_end, arrow_size = arrow_len / 10.0, life_time = 30.0, color = wp_arrow_col )
                world.debug.draw_arrow( vbox_start, vbox_end, arrow_size = arrow_len / 10.0, life_time = 30.0, color = box_arrow_col )

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
            riff_MNC_SLP.timestamp = timestamp_offset + carla_time

            riff_MNC_SLP.lateral_offset = vldist                  # From centre of lane
            riff_MNC_SLP.heading_error  = ang_error               # Rads error

            # Not so sure about the following...

            # If the angle error is huge (> 0.5 rads) then its probably a lane crossing and we should ignore it.
            mag_ang_err = ang_error
            if mag_ang_err < 0.0:
                mag_ang_err = -mag_ang_err

            if mag_ang_err > 0.2:
                riff_MNC_SLP.lateral_confidence = 0.0
                riff_MNC_SLP.heading_confidence = 0.0
            else:
                riff_MNC_SLP.lateral_confidence = 1.0
                riff_MNC_SLP.heading_confidence = 1.0

            lane_riff_message.tag = b'SLP '
            lane_riff_message.length = ctypes.sizeof( riff_MNC_SLP )
            lane_riff_message.data   = ctypes.byref(  riff_MNC_SLP )
            lane_riff_client.write( lane_riff_message )

        if want_other_objects:
            # Get stats about the vehicle
            veh_trans = vehicle.get_transform()
            veh_geo = the_map.transform_to_geolocation( veh_trans.location )
            veh_utm = utm.from_latlon( veh_geo.latitude, veh_geo.longitude )
            veh_ang = math.radians( (360 - veh_trans.rotation.yaw) % 360.0 )
            vi = math.cos( veh_ang )
            vj = math.sin( veh_ang )

            # These values are from the middle of the bus but sensors are located at front.
            # So move the detectors location half a bus length closer to the front
            half_bus_dist = 1.0
            vx = veh_utm[0] + half_bus_dist * vi
            vy = veh_utm[1] + half_bus_dist * vj

            veh_vel = vehicle.get_velocity()
            mag_veh_vel = veh_vel.x*veh_vel.x + veh_vel.y*veh_vel.y        # Not doing z
            if mag_veh_vel > 0.0:
                mag_veh_vel = math.sqrt( mag_veh_vel )

            # Get a list of all other actors / objects
            other_objects_array = []
            num_other_objects = 0
            actors  = world.get_actors()
            if len( actors ) > 0:
                for actor in actors:
                    if actor.type_id != vehicle_blueprint_name and (re.search("vehicle.*", actor.type_id) or re.search("walker.*", actor.type_id) ):
                        oa_trans = actor.get_transform()
                        oa_geo = the_map.transform_to_geolocation( oa_trans.location )
                        oa_utm = utm.from_latlon( oa_geo.latitude, oa_geo.longitude )
                        oax = oa_utm[0]
                        oay = oa_utm[1]
                        oa_ang = math.radians( (360 - oa_trans.rotation.yaw) % 360.0 )
                        oai = math.cos( oa_ang )
                        oaj = math.sin( oa_ang )

                        oa_vel = actor.get_velocity()
                        oa_mag_vel2 = oa_vel.x*oa_vel.x + oa_vel.y*oa_vel.y
                        oa_mag_vel = 0
                        if oa_mag_vel2 > 0.0:
                            oa_mag_vel = math.sqrt( oa_mag_vel2 )
                        oavi = oa_mag_vel * oai
                        oavj = oa_mag_vel * oaj

                        # Rel position
                        v_oa_i = oax - vx
                        v_oa_j = oay - vy

                        # Rel position in bus coordinate space
                        v_dot_voa = vi*v_oa_i + vj*v_oa_j
                        v_x_voa   = vi*v_oa_j - vj*v_oa_i

                        # Velocity in bus direction
                        v_dot_oav = vi * oavi + vj * oavj
                        v_x_oav   = vi * oavj - vj * oavi

                        if v_dot_voa > 0 and v_dot_voa < 300.0:
                            other_objects_array.append( v_dot_voa )
                            other_objects_array.append( v_x_voa )
                            other_objects_array.append( v_dot_oav - mag_veh_vel )
                            other_objects_array.append( v_x_oav )
                            num_other_objects = num_other_objects + 1
                            #print( '### SOBS obj:', round( v_dot_voa, 1 ), round( v_x_voa, 1 ), round( v_dot_oav - mag_veh_vel, 1 ), round( v_x_oav, 1 ) )

            ctarray = (ctypes.c_float * (num_other_objects * 4))( *other_objects_array )
            other_objs_riff_message.tag = b'SOBS'
            other_objs_riff_message.length = ctypes.sizeof( ctypes.c_float ) * (num_other_objects * 4)
            other_objs_riff_message.data   = ctarray
            radar_riff_client.write( other_objs_riff_message )

        end_frame_time = GetRealMsFromStart() / 1000.0
        dt = end_frame_time - start_frame_time
        if dt < (1.0/30.0):
            time.sleep( (1.0/30.0) - dt )

    except KeyboardInterrupt:
        print( 'Keyboard interrupt, exiting...' )





