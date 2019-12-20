from __future__ import print_function

from multiprocess import Pool, Queue, Manager
import multiprocess as mp
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
from MyVehicle import MyVehicle
from Simulation import Simulation
from Const import Const
from geometric_helper_functions import *
from fitness import run_sim
import argparse
import os
import subprocess as sp
import neat
import visualize
import pickle
import random
import dill
import threading
import time

    
class SITLManager():

    def __init__(self, n_sims, start_sims, map, console):
        Const.N_SIMS = n_sims
        self.manager = mp.Manager()
        self.SITL_connection_strings = self.manager.Queue()
        self.open_ports = self.manager.Queue()
        # Startup N_SIMS sim_vehicle.py if needed
        if start_sims:
            for i in range(Const.N_SIMS): 
                cmd = []
                cmd.append('sim_vehicle.py')
                cmd.append('-vArduPlane')
                cmd.append('--speedup=' + str(Const.SPEED_UP)) # bug in ardupilot, doesnt actually do anything
                cmd.append('--no-rebuild')
                cmd.append('-I' + str(i))
                if map: cmd.append('--map')  
                if console: cmd.append('--console')
                # cmd.append('--no-mavproxy')

                sp.Popen(cmd, shell=False, stdout=sp.DEVNULL)
                # sp.Popen(cmd, shell=False)

                # Give the process some time to secure the needed ports
                time.sleep(1)
            print('Sims launched')

        # Create a queue of connection strings to connect to the SITL instances
        for i in range(Const.N_SIMS):
            #  Connect to SITL directly wthout mavproxy, can only do one instance
            # port = 5760 + i*10
            # self.SITL_connection_strings.put('tcp:127.0.0.1:' + str(port))
            # Connect to mavproxy, uses more rescources, max about 20 instances
            port = 14550 + i*10
            self.SITL_connection_strings.put('127.0.0.1:' + str(port))

        # Create a queue of ports for JBotEvolver tasks to connect to
        for i in range(Const.N_SIMS):
            self.open_ports.put(61000 + i)

        sims = []
        for i in range(Const.N_SIMS):
            p = mp.Process(target=self.initializer, args=(self.SITL_connection_strings, self.open_ports))
            p.start()
            sims.append(p)

        for p in sims:
            p.join()
    
    def start_sim(self, cmd):
        proc = sp.Popen(cmd, shell=False, stdout=sp.DEVNULL)

    def conn_sim(self, connection_string, sims, index):
        self.sims[index] = Simulation(connection_string, targets_amount=5, speedup=Const.SPEED_UP)

    def initializer(self, SITL_connection_strings, open_ports):
        from multiprocess import Pool, Queue, Manager
        import multiprocess as mp
        from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
        from MyVehicle import MyVehicle
        from Simulation import Simulation
        from Const import Const
        import global_sim
        import fitness
        
        print('initializing process {}'.format(mp.current_process().name))
        SITL_conn = SITL_connection_strings.get()
        port = open_ports.get()
        print('Connecting to sim {} with process {}'.format(SITL_conn, mp.current_process()))
        sim = Simulation(SITL_conn, port, targets_amount=5, speedup=Const.SPEED_UP, logging=Const.LOGGING)


if __name__ == '__main__':
    mp.set_start_method('spawn', True)

    # parse input
    parser = argparse.ArgumentParser(description='Train a NN with the NEAT algorithm.')
    parser.add_argument('--n_sims', type=int, default=1,
                    help='Amount of simulators to run in parallel')
    parser.add_argument('--start_sims', type=bool, default=1,
                    help='Set to False if sims are already running')
    parser.add_argument('--map', type=bool, nargs='?', const=True, default=0,
                    help='Load MAVProxy map')
    parser.add_argument('--console', type=bool, nargs='?', const=True, default=0,
                    help='Load MAVProxy console')
    args = parser.parse_args()

    SITLManager(args.n_sims, args.start_sims, args.map, args.console)

    


