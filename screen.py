#!/usr/bin/env python



"""
Broadcast video feed to multiple screen
"""

from __future__ import print_function

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla


import os
import argparse
import logging
import time
import pygame
import sys
import numpy as np


class Display:
    def __init__(self, args):
        self.surface = None
        pygame.init()
        pygame.font.init()
        world = None


        print("Finding camera: ", args.camera)
        print("Number of displays found: ", pygame.display.get_num_displays())

        # try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        self.world = client.get_world()
        

        clock = pygame.time.Clock()

        # Get camera
        cameras = self.world.get_actors().filter('sensor.camera.rgb')
        print(cameras)
        self.camera  = None
        for c in cameras:
            if c.attributes['role_name'] == args.camera:
                self.camera = c 
                break
                
        if self.camera is None:
            print("Camera not found")
            sys.exit(0)
        
        self.display = pygame.display.set_mode(
            (int(self.camera.attributes['image_size_x']), int(self.camera.attributes['image_size_y'])),
            pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.FULLSCREEN, display=args.display)

        self.camera.listen(self.broadcast)
        # self.camera.listen(lambda image: self.broadcast(display, image))

            # self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        # finally:
        #     pygame.quit()
        #     pass

    def broadcast(self, image):

        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))


    def loop(self):
        while True:
            self.world.wait_for_tick()
            if self.surface is not None:
                self.display.blit(self.surface, (0, 0))

                pygame.display.flip()
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-c', '--camera',
        default='center',
        help='enable autopilot')
    argparser.add_argument(
        '-d', '--display',
        default=0,
        type=int,
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1440x940)')
    args = argparser.parse_args()

    args.rolename = 'hero'      # Needed for CARLA version
    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    disp = Display(args)

    try:
        pass
        disp.loop()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)


if __name__ == '__main__':

    main()
