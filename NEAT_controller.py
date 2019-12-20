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
import global_sim
# TODO debug: WARNING:autopilot:Mission upload timeout, still occuring but doesn't seem to be a problem with current setup
# TODO debug:  RuntimeError: Expected 8 inputs, got 7, in neural net, hasn't happend again
# TODO logging?



class ParallelEvaluatorSims(neat.ParallelEvaluator):
    def __init__(self, num_workers, eval_function, timeout=None):
        self.num_workers = num_workers
        self.eval_function = eval_function
        self.timeout = timeout
        self.manager = mp.Manager()
        self.connection_strings = self.manager.Queue()
        for i in range(num_workers):
            #  Connect to SITL directly wthout mavproxy, can only do one instance
            # port = 5760 + i*10
            # self.connection_strings.put('tcp:127.0.0.1:' + str(port))
            # Connect to mavproxy, uses more rescources, max about 20 instances
            port = 14550 + i*10
            self.connection_strings.put('127.0.0.1:' + str(port))

        self.pool = mp.Pool(num_workers, initializer=self.initializer, initargs=(self.connection_strings,))

    def initializer(self, connection_strings):
        from multiprocess import Pool, Queue, Manager
        import multiprocess as mp
        from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal
        from MyVehicle import MyVehicle
        from Simulation import Simulation
        from Const import Const
        import global_sim
        import fitness
        print('initializing process {}'.format(mp.current_process().name))
        c = connection_strings.get()
        print('Connecting to sim {} with process {}'.format(c, mp.current_process()))
        global_sim.sim = Simulation(c, targets_amount=5, speedup=Const.SPEED_UP)

    def evaluate(self, genomes, config):
        jobs = []
        for ignored_genome_id, genome in genomes:
            g = (genome, config)
            jobs.append(self.pool.apply_async(self.eval_function, args=(g,)))

        # assign the fitness back to each genome
        for job, (ignored_genome_id, genome) in zip(jobs, genomes):
            genome.fitness = job.get(timeout=self.timeout)

class NeatEvolver():

    def __init__(self, n_sims, start_sims):
        random.seed(a=1337)
        Const.N_SIMS = n_sims

        # Startup N_SIMS sim_vehicle.py if needed
        if start_sims:
            for i in range(Const.N_SIMS): 
                cmd = []
                cmd.append('sim_vehicle.py')
                cmd.append('-vArduPlane')
                cmd.append('--speedup=' + str(Const.SPEED_UP)) # bug in ardupilot, doesnt actually do anything
                cmd.append('-I' + str(i))
                # cmd.append('--no-mavproxy')
                # cmd.append('--console')
                cmd.append('--map')

                sp.Popen(cmd, shell=False, stdout=sp.DEVNULL)
                # sp.Popen(cmd, shell=False)

                # Give the process some time to secure the needed ports
                time.sleep(1)
            print('Sims launched')
            # let sim_vehicle instances setup before attempting to connect to them
            time.sleep(2)

        # connect to the simulators
        # self.sims = [None] * Const.N_SIMS
        # threads = [None] * Const.N_SIMS
        # for i in range(Const.N_SIMS):
        #     port = 14550 + i*10
        #     connection_string = '127.0.0.1:' + str(port)
        #     t = threading.Thread(target=self.conn_sim, args=(connection_string,self.sims,i))
        #     t.start()
        #     threads[i] = t
        # for t in threads:
        #     t.join()

        # print("Connected to all sims")
        # Set locks for accessing the sims
        # manager = Manager()
        # self.sim_locks =  manager.list([manager.Lock() for _ in range(Const.N_SIMS)])
        # self.managed_sims = manager.list(self.sims)
        # Determine path to configuration file. This path manipulation is
        # here so that the script will run successfully regardless of the
        # current working directory.
        local_dir = os.path.dirname(__file__)
        config_path = os.path.join(local_dir, 'config.py')
        self.run(config_path)

    # def eval_genomes(self, genomes, config):
    #     manager = mp.Manager()
    #     q = manager.Queue()

    #     for i, (genome_id, genome) in enumerate(genomes):
    #         q.put((genome_id, genome))
    #         # print(genome_id)

    #     def threader(q):
    #         # while there are genomes in the queue a thread will run a simulation with it and return a fitness
    #         while True:
    #             (genome_id, genome) = q.get()
    #             print('Evaluating genome {} of {}'.format(genome_id, len(genomes)))
    #             net = neat.nn.FeedForwardNetwork.create(genome, config)
    #             genome.fitness = self.run_sim(net)
    #             print('set fitness')
    #             q.task_done()

    #     # start N_SIMS threads
    #     # procs = []
    #     for _ in range(Const.N_SIMS):
    #         p = mp.Process(target=threader, args=(q,))
    #         p.daemon = True # thread closes when main thread closes
    #         p.start()
    #     # Try with processpool
    #     pool = Pool(nodes=Const.N_SIMS)
    #     pool.apply_async(threader, genomes)

    #     q.join()
    #     # block untill all g        q.join()enomes are 
        # for p in procs:
            # p.join()
                
    # def calc_timestep_fitness(self, sim):
    #     # should be called every timestep
    #     timestep_fitness = 0
    #     shortest_dist = min([calc_distance_and_angle(t, sim.vehicle1.location.local_frame, sim.vehicle1.heading)[0] for t in sim.targets])

    #     # Add a bonus if target is seen
    #     # if sim.vehicle1.target_seen:
    #     #     timestep_fitness += 100 #SEEN_TARGET_BONUS
    #     #     sim.vehicle1.target_seen = False
        
    #     # Only add distance fitness if inside geofence
    #     if sim.vehicle1.geo_sensor.check_inside_fence():
    #         timestep_fitness = 1
    #         # timestep_fitness += (sim.shortest_start_dist - shortest_dist) / sim.shortest_start_dist
    #         # print('timestep fitness: {}'.format(timestep_fitness))aq
    #     else:
    #             timestep_fitness = -1
    #     return timestep_fitness

    # def run_sim(self, g):
    #     print('evaluating genome')
    #     (genome, config) = g
    #     # print('Evaluating genome {} of {}'.format(genome_id, len(genomes)))
    #     net = neat.nn.FeedForwardNetwork.create(genome, config)
    #     # TODO check if speedups and threads arent set to high so they dont interfere with the timing
    #     # loop over the locks to find the simulator that is unlocked
    #     for i, sim_lock in enumerate(self.sim_locks):
    #         if sim_lock.acquire(blocking=False): #returns False if busy
    #             print('Using simulator {} to evauate this genome'.format(i))
    #             sims[i].reset()
    #             time.sleep(5/Const.SPEED_UP) # give the simulation some time to set up 
                
    #             fitness = 0
    #             for t in range(Const.MAX_TIMESTEPS):
    #                 print(t)
    #                 print(Const.MAX_TIMESTEPS)
    #                 if sims[i].early_stop():
    #                     print('Stopping early')
    #                     break
    #                 fitness += self.calc_timestep_fitness(sims[i])
    #                 sensor_readings = sims[i].vehicle1.get_sensor_readings()
    #                 output = net.activate(sensor_readings)
    #                 print('sending output: {}'.format(output))
    #                 sims[i].vehicle1.control_plane(thrust=1, angle=output[0], duration=(1/(Const.UPDATE_FREQ*Const.SPEED_UP)))
    #                 print('sent output')
    #             # if simulation has run, release and break. 
    #             sim_lock.release()
    #             print('lock released')
    #             break
    #         else: pass
    #     return fitness

    def run(self, config_file):
        # Load configuration.
        config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                            neat.DefaultSpeciesSet, neat.DefaultStagnation,
                            config_file)

        # Create the population, which is the top-level object for a neat run.
        p = neat.Population(config)

        # Add a stdout reporter to show progress in the terminal.
        p.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        p.add_reporter(stats)
        p.add_reporter(neat.Checkpointer(5))

        # Run for up to MAX_GENERATIONS generations.
        p_evaluator = ParallelEvaluatorSims(Const.N_SIMS, run_sim)
        winner = p.run(p_evaluator.evaluate, Const.MAX_GENERATIONS)

        # Display the winning genome.
        print('\nBest genome:\n{!s}'.format(winner))

        # Show output of the most fit genome against training data.
        print('\nOutput:')
        winner_net = neat.nn.FeedForwardNetwork.create(winner, config)
        # for xi, xo in zip(xor_inputs, xor_outputs):
        #     output = winner_net.activate(xi)
        #     print("input {!r}, expected output {!r}, got {!r}".format(xi, xo, output))

        # node_names = {-1:'A', -2: 'B', 0:'A XOR B'}
        visualize.draw_net(config, winner, True)
        visualize.plot_stats(stats, ylog=False, view=True)
        visualize.plot_species(stats, view=True)

        # p = neat.Checkpointer.restore_checkpoint('neat-checkpoint-4')
        # p.run(eval_genomes, 10)

    def start_sim(self, cmd):
        proc = sp.Popen(cmd, shell=False, stdout=sp.DEVNULL)

    def conn_sim(self, connection_string, sims, index):
        self.sims[index] = Simulation(connection_string, targets_amount=5, speedup=Const.SPEED_UP)



if __name__ == '__main__':
    mp.set_start_method('spawn', True)

    # parse input
    parser = argparse.ArgumentParser(description='Train a NN with the NEAT algorithm.')
    parser.add_argument('-n_sims', type=int, default=1,
                    help='Amount of simulators to run in parallel')
    parser.add_argument('-start_sims', type=bool, default=1,
                    help='Set to False if sims are already running')
    args = parser.parse_args()

    NeatEvolver(args.n_sims, args.start_sims)

    


