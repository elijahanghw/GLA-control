import numpy as np

from utils import *

class Genetic:
    def __init__(self, num_states, num_inputs, num_pop=100, offsprings=0.5):
        self.num_pop = num_pop
        self.num_states = num_states
        self.num_inputs = num_inputs
        self.num_offspring = int(offsprings*num_pop)

    def initialize(self):
        self.population = []
        self.fitness = []
        for _ in range(self.num_pop):
            individual = np.zeros(self.num_states + self.num_inputs)
            for j in range(self.num_states + self.num_inputs):
                individual[j] = log10uniform(-5, 5)
            self.population.append(individual)

    def roulette(self, fitness_sorted, offset=0.1):
        ## Roulette selection of parents ##
        # Fitness normalization
        fitness_roulette = []
        lowest_fitness = fitness_sorted[-1][1]
        fitness_sum = 0
        for i in range(self.num_pop):
            fitness_roulette.append(fitness_sorted[i][1] - lowest_fitness + offset)
            fitness_sum += fitness_sorted[i][1] - lowest_fitness + offset
            
        for i in range(self.num_pop):
            fitness_roulette[i] /= fitness_sum

        fitness_roulette = np.cumsum(fitness_roulette)      
        r1 = np.random.random()
        r2 = np.random.random()
        for ind, probability in enumerate(fitness_roulette):
            if probability > r1:
                parent1 = fitness_sorted[ind][0]
                break

        for ind, probability in enumerate(fitness_roulette):
            if probability > r2:
                parent2 = fitness_sorted[ind][0]
                break
        
        return parent1, parent2
    
    def crossover(self, parent1, parent2):
        ## Single point crossover ##
        point = np.random.randint(1, self.num_states+self.num_inputs-1)
        child1 = np.concatenate((parent1[0:point], parent2[point:]))
        child2 = np.concatenate((parent2[0:point], parent1[point:]))
        
        return child1, child2


    def mutate(self, child, mu=0.05, sigma=0.1):
        ## Mutate genes ##

        return child

    def next_gen(self):
        self.offsprings = []
        self.population = []
        self.fitness_sorted = sorted(self.fitness, key=lambda tup: tup[1], reverse=True)

        parent1, parent2 = self.roulette(self.fitness_sorted)

        for _ in range(self.num_offspring):
            child1, child2 = self.crossover(parent1, parent2)
            child1 = self.mutate(child1)
            child2 = self.mutate(child2)
            if len(self.offsprings) < self.num_offspring:
                self.offsprings.append(child1)
            if len(self.offsprings) < self.num_offspring:
                self.offsprings.append(child2)

        for i in range(self.num_pop-self.num_offspring):
            self.population.append(self.fitness_sorted[i][0])

        for offspring in self.offsprings:
            self.population.append(offspring)

        self.fitness = []



        