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
            species = np.zeros(self.num_states + self.num_inputs)
            for j in range(self.num_states + self.num_inputs):
                species[j] = log10uniform(-2, 3)
            self.population.append(species)

    def roulette(self, fitness):
        ## Roulette selection of parents ##
        parent1 = fitness[0][0]
        parent2 = fitness[1][0]
        
        return parent1, parent2
    
    def crossover(self, parent1, parent2):
        ## Single point crossover ##
        point = np.random.randint(0, self.num_states+self.num_inputs)
        child1 = parent1
        child2 = parent2
        
        return child1, child2


    def mutate(self, child, mu=0.05, sigma=0.1):
        ## Mutate genes ##

        return child

    def next_gen(self):
        self.offsprings = []
        self.fitness.sort(key=lambda tup: tup[1], reverse=True)
        parent1, parent2 = self.roulette(self.fitness)

        for _ in range(self.num_offspring):
            child1, child2 = self.crossover(parent1, parent2)
            child1 = self.mutate(child1)
            child2 = self.mutate(child2)
            if len(self.offsprings) < self.num_offspring:
                self.offsprings.append(child1)
            if len(self.offsprings) < self.num_offspring:
                self.offsprings.append(child2)
 
        self.population = self.fitness[0:self.num_pop-self.num_offspring]

        for offspring in self.offsprings:
            self.population.append(offspring)


        