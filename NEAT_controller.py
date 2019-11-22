"""
2-input XOR example -- this is most likely the simplest possible example.
"""

from __future__ import print_function
import argparse
import os
import subprocess
import neat
import visualize
from MyVehicle import MyVehicle
from Simulation import Simulation
import random
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
import threading
from queue import Queue
import multiprocessing as mp
import time

MAX_TIME_SECONDS = 60
UPDATE_FREQ = 10 
MAX_TIMESTEPS = MAX_TIME_SECONDS * UPDATE_FREQ
SEEN_TARGET_BONUS = 100
MAX_GENERATIONS = 10
SPEED_UP = 4
N_SIMS = 1 # gets set by input arg

#  TODO debug: WARNING:autopilot:Mission upload timeout
# TODO debug:  RuntimeError: Expected 8 inputs, got 7, in neural net
# TODO logging?
# TODO seems to be evaluating genome 49 over and over?


def eval_genomes(genomes, config):
    q = Queue()

    for i, (genome_id, genome) in enumerate(genomes):
        q.put((genome_id, genome))

    def threader():
        # while there are genomes in the queue a thread will run a simulation with it and return a fitness
        while True:
            (genome_id, genome) = q.get()
            print('Evaluating genome {} of {}'.format(i, len(genomes)))
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

# def eval_genomes(genomes, config):
#     for genome_id, genome in genomes:
#         genome.fitness = 4.0
#         net = neat.nn.FeedForwardNetwork.create(genome, config)
#         for xi, xo in zip(xor_inputs, xor_outputs):
#             output = net.activate(xi)
#             genome.fitness -= (output[0] - xo[0]) ** 2


def run(config_file):
    # Load configuration.
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_file)

    # Create the population, which is the top-level object for a NEAT run.
    p = neat.Population(config)

    # Add a stdout reporter to show progress in the terminal.
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)
    p.add_reporter(neat.Checkpointer(5))

    # Run for up to MAX_GENERATIONS generations.
    winner = p.run(eval_genomes, MAX_GENERATIONS)

    # Display the winning genome.
    print('\nBest genome:\n{!s}'.format(winner))

    # # Show output of the most fit genome against training data.
    # print('\nOutput:')
    # winner_net = neat.nn.FeedForwardNetwork.create(winner, config)
    # for xi, xo in zip(xor_inputs, xor_outputs):
    #     output = winner_net.activate(xi)
    #     print("input {!r}, expected output {!r}, got {!r}".format(xi, xo, output))

    # node_names = {-1:'A', -2: 'B', 0:'A XOR B'}
    # visualize.draw_net(config, winner, True, node_names=node_names)
    # visualize.plot_stats(stats, ylog=False, view=True)
    # visualize.plot_species(stats, view=True)

    # p = neat.Checkpointer.restore_checkpoint('neat-checkpoint-4')
    # p.run(eval_genomes, 10)

def start_sim(cmd):
    proc = subprocess.run(cmd, shell=False, stdout=subprocess.DEVNULL)
    # proc = subprocess.run(cmd, shell=False)
    # set the speedup parameter 'manually'
    # proc.communicate(input='param set SIM_SPEEDUP 8')
    # # give it time to setup
    # time.sleep(15)
    



if __name__ == '__main__':
    # parse input
    parser = argparse.ArgumentParser(description='Train a NN with the NEAT algorithm.')
    parser.add_argument('-n_sims', type=int, 
                    help='Amount of simulators to run in parallel')
    parser.add_argument('-start_sims', type=bool,
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
        sims.append(Simulation(connection_string, targets_amount=5))

    # Set locks for accessing the sims
    sim_locks =  [threading.Lock() for _ in sims]

    # Determine path to configuration file. This path manipulation is
    # here so that the script will run successfully regardless of the
    # current working directory.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config.py')
    run(config_path)


