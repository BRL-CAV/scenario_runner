#!/usr/bin/python3

import integration as CAVstar


setup = CAVstar.SetupCarla()
(world, the_map, vehicle ) = setup.get()

CAVstar.RunSim(world, the_map, vehicle )
