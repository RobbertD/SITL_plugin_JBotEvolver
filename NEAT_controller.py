"""
2-input XOR example -- this is most likely the simplest possible example.
"""

from __future__ import print_function
import os
import neat
import visualize
from MyVehicle import MyVehicle
from Simulation import Simulation
import random
from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal

import time

MAX_TIME_SECONDS = 60
UPDATE_FREQ = 10 
MAX_TIMESTEPS = MAX_TIME_SECONDS * UPDATE_FREQ
SEEN_TARGET_BONUS = 100
MAX_GENERATIONS = 40
SPEED_UP = 4

def eval_genomes(genomes, config):
    for i, (genome_id, genome) in enumerate(genomes):
        # start a clean simulation without having to shutdown SITL
        print('Evaluating genome {} of {}'.format(i, len(genomes)))
        sm.run_sim()
        time.sleep(5/SPEED_UP) # give the simulation some time to set up 
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        genome.fitness = 0
        for t in range(MAX_TIMESTEPS):
            if sm.early_stop():
                print('Stopping early')
                break
            genome.fitness += sm.get_fitness_data()
            sensor_readings = sm.vehicle1.get_sensor_readings()
            output = net.activate(sensor_readings)
            sm.vehicle1.control_plane(thrust=1, angle=output[0], duration=(1/(UPDATE_FREQ*SPEED_UP)))

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


if __name__ == '__main__':
    random.seed(a=1337)
    sm = Simulation(targets_amount=5)

    # Determine path to configuration file. This path manipulation is
    # here so that the script will run successfully regardless of the
    # current working directory.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config.py')
    run(config_path)

    # sm.run_sim()
    # sm.vehicle1.get_sensor_readings()
    # sm.vehicle1.set_position(LocationGlobal(-35.360302546095355,149.16494236133667, 0), 100, 1)
    # time.sleep(10)
    # 
    # sm.vehicle1.control_plane(1, angle=0, duration=0.1)




