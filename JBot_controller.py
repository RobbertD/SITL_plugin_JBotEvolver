from __future__ import print_function
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
from MyVehicle import MyVehicle
from Simulation import Simulation
import argparse
import os
import subprocess
import random
import socket
import threading
from queue import Queue
import multiprocessing as mp
import time

MAX_TIME_SECONDS = 20
UPDATE_FREQ = 1
MAX_TIMESTEPS = MAX_TIME_SECONDS * UPDATE_FREQ
SEEN_TARGET_BONUS = 100
MAX_GENERATIONS = 10
SPEED_UP = 10

N_SIMS = 1 # gets set by input arg

def eval_genomes(genomes, config):
    q = Queue()

    for i, (genome_id, genome) in enumerate(genomes):
        q.put((genome_id, genome))
        # print(genome_id)

    def threader():
        # while there are genomes in the queue a thread will run a simulation with it and return a fitness
        while True:
            (genome_id, genome) = q.get()
            if genome_id==3:
                print('genome id 3')
            
            print('Evaluating genome {} of {}'.format(genome_id, len(genomes)))
            net = neat.nn.FeedForwardNetwork.create(genome, config)
            genome.fitness = run_sim(net)
            q.task_done()

    # start N_SIMS threads
    for _ in range(N_SIMS):
        t = threading.Thread(target=threader, args=())
        t.daemon = True # thread closes when main thread closes
        t.start()

    # block untill all genomes are processed
    q.join()


def run_sim(net):
    # TODO check if speedups and threads arent set to high so they dont interfere with the timing
    # loop over the locks to find the simulator that is unlocked
    for i, sim_lock in enumerate(sim_locks):
        if sim_lock.acquire(blocking=False): #returns False if busy
            sims[i].reset()
            time.sleep(5/SPEED_UP) # give the simulation some time to set up 
            
            fitness = 0
            for t in range(MAX_TIMESTEPS):
                print(t)
                if sims[i].early_stop():
                    print('Stopping early')
                    break
                fitness += sims[i].get_fitness_data()
                sensor_readings = sims[i].vehicle1.get_sensor_readings()
                output = net.activate(sensor_readings)
                sims[i].vehicle1.control_plane(thrust=1, angle=output[0], duration=(1/(UPDATE_FREQ*SPEED_UP)))
            # if simulation has run, release and break. 
            sim_lock.release()
            break
    return fitness


if __name__ == '__main__':
    # parse input
    parser = argparse.ArgumentParser(description='Train a NN with the NEAT algorithm.')
    parser.add_argument('-n_sims', type=int, default=1,
                    help='Amount of simulators to run in parallel')
    parser.add_argument('-start_sims', type=bool, default=1,
                    help='Set to False if sims are already running')
    args = parser.parse_args()

    random.seed(a=1337)

    N_SIMS = args.n_sims

    # Startup N_SIMS sim_vehicle.py if needed
    if args.start_sims:
        mp.set_start_method('spawn')
        for i in range(N_SIMS): 
            cmd = []
            cmd.append('sim_vehicle.py')
            cmd.append('-vArduPlane')
            cmd.append('--speedup=' + str(SPEED_UP)) # bug in ardupilot, doesnt actually do anything
            cmd.append('-I' + str(i))
            # cmd.append('--console')

            cmd.append('--map')

            print(cmd)
            p = mp.Process(target=start_sim, args=(cmd,))
            p.daemon = True # thread closes when main thread closes
            p.start()
            # Give the process some time to secure the needed ports
            time.sleep(1)

        # let sim_vehicle instances setup before attempting to connect to them
        time.sleep(5)

    # connect to the simulators
    sims = []
    for i in range(N_SIMS):
        port = 14550 + i*10
        connection_string = '127.0.0.1:' + str(port)
        sims.append(Simulation(connection_string, targets_amount=5, speedup=SPEED_UP))

    for i in range(N_SIMS):
        port = 14550 + i*10
        connection_string = '127.0.0.1:' + str(port)
        sims.append(Simulation(connection_string, targets_amount=5, speedup=SPEED_UP))

    # Set locks for accessing the sims
    sim_locks =  [threading.Lock() for _ in sims]

    # create the socket
    # AF_INET == ipv4
    # SOCK_STREAM == TCP
    s = socket.socket(socket.AF_LOCAL, socket.SOCK_STREAM)
    s.bind((socket.gethostname(), 1234))
    s.listen(1)

    while True:
    # now our endpoint knows about the OTHER endpoint.
        clientsocket, address = s.accept()
        print(f"Connection from {address} has been established.")
