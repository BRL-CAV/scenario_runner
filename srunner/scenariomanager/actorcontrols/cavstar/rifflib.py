#!/usr/bin/python3
#
#      Copyright (c) Fusion Processing Ltd 2018-2021
#      All rights reserved.
#
import ctypes, os

SCENARIO_RUNNER_ROOT = os.getenv('SCENARIO_RUNNER_ROOT')

if os.name == 'posix': rifflib = ctypes.CDLL( SCENARIO_RUNNER_ROOT + '/srunner/scenariomanager/actorcontrols/cavstar/librifflib.so' )
if os.name == 'nt': rifflib = ctypes.CDLL( 'x64/Release/rifflib.dll' )

# Define the generic RIFF message used to send and receive data
class Message:
    tag = ctypes.create_string_buffer(5)
    length = ctypes.c_int(0)
    data = ctypes.c_void_p(0)

# The RIFF server with automatic inititialise and close
class RiffServer:

    # Start the RIFF server using the provided port
    def __init__( self, port, timeout, name ):
        self.rsstate = ctypes.c_void_p(0)
        return_code = rifflib.RiffServer_Initialise(
            port, ctypes.c_double(timeout), name.encode('utf-8'),
            ctypes.byref(self.rsstate) )
        if return_code: raise RuntimeError( 'RiffServer failed to initialise' )

    # Stop the RIFF server
    def __del__( self ):
        if rifflib: rifflib.RiffServer_Disconnect( self.rsstate )

    # Returns True from a forked process with a connected client
    # Returns False from the server thread ready to listen again
    def listen( self ):
        return_code = rifflib.RiffServer_Listen( self.rsstate )
        if return_code: raise RuntimeError( 'RiffServer failed to listen' )

        if self.connected():
            if os.fork() == 0:
                return True
            else:
                # The original process prepares to listen again
                new_rsstate = ctypes.c_void_p(0)
                return_code = rifflib.RiffServer_Fork(
                    self.rsstate, ctypes.byref(new_rsstate) )
                if return_code: raise RuntimeError( 'RiffServer failed to fork' )
                self.rsstate = new_rsstate
        return False

    # Return true if connected to a server
    def connected( self ):
        connected = ctypes.c_int(0)
        return_code = rifflib.RiffServer_Get_Connected(
            self.rsstate, ctypes.byref(connected) )
        if return_code: raise RuntimeError( 'RiffServer failed get connected' )
        return connected.value != 0

    # Receive any incomming data
    def receive( self ):
        msg = Message()
        return_code = rifflib.RiffServer_Receive( self.rsstate,
            msg.tag, ctypes.byref(msg.data), ctypes.byref(msg.length) )
        if return_code: raise RuntimeError( 'RiffServer failed to receive' )
        return msg

    # Write data to be transmitted
    def write( self, message ):
        return_code = rifflib.RiffServer_Write( self.rsstate,
            message.tag, message.data, message.length )
        if return_code: raise RuntimeError( 'RiffServer failed to write' )

# The RIFF client with automatic inititialise and close
class RiffClient:

    # Start the RIFF client using the provided port and host
    def __init__( self, port, host, timeout, server_name ):
        self.rcstate = ctypes.c_void_p(0)
        return_code = rifflib.RiffClient_Initialise(
            port, host.encode('utf-8'),
            ctypes.c_double(timeout),
            server_name.encode('utf-8'),
            ctypes.byref(self.rcstate) )
        if return_code: raise RuntimeError( 'RiffClient failed to initialise' )

    # Stop the RIFF client
    def __del__( self ):
        if rifflib: rifflib.RiffClient_Disconnect( self.rcstate )

    # Connect to a RIFF server
    def connect( self ):
        return_code = rifflib.RiffClient_Connect( self.rcstate )
        if return_code: raise RuntimeError( 'RiffClient failed during connect' )

    # Return true if connected to a server
    def connected( self ):
        connected = ctypes.c_int(0)
        return_code = rifflib.RiffClient_Get_Connected(
            self.rcstate, ctypes.byref(connected) )
        if return_code: raise RuntimeError( 'RiffClient failed get connected' )
        return connected.value != 0

    # Receive any incomming data
    def receive( self ):
        msg = Message()
        return_code = rifflib.RiffClient_Receive( self.rcstate,
            msg.tag, ctypes.byref(msg.data), ctypes.byref(msg.length) )
        if return_code: raise RuntimeError( 'RiffClient failed to receive' )
        return msg

    # Write data to be transmitted
    def write( self, message ):
        return_code = rifflib.RiffClient_Write( self.rcstate,
            message.tag, message.data, message.length )
        if return_code: raise RuntimeError( 'RiffClient failed to write' )

