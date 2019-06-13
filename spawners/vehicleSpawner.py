#!/usr/bin/python

import numpy as np

from entities.vehicles.vehicleSettings import *
from entities.vehicles.standardCar import *
from spawners.worldSpawner import *

class spawnCars(worldSpawner):
    def __init__(self, numberOfEntities):
        super(spawnCars, self).__init__(numberOfEntities)
    def spawnRandomlyStandardCars(self):
        standardCars = [None] * self.numberOfEntities
        for i in range(self.numberOfEntities):
            standardCars[i]  = standardCar(self.numberOfEntities, 'standardCar', i, 'random')
        return np.array(standardCars)
    def spawnStandardCarsInArea(self, position1, position2, velocity, target):
        standardCars = [None] * self.numberOfEntities
        for i in range(self.numberOfEntities):
            standardCars[i] = standardCar(self.numberOfEntities, 'standardCar', i, 'area', position1, position2, velocity, target)
        return np.array(standardCars)