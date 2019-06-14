#!/usr/bin/python
from entities.entities import *
import numpy as np
from entities.vehicles.vehicleSettings import *

from world.worldParameters import *
# class vehicleType:
class vehicle(entities):
    '''Defines the type of vehicle and packages the coordinates'''
    def __init__(self, numberOfEntities, vehType, id, initialConditions, position=np.zeros(3), target=np.zeros(3), velocity=np.zeros(3), target2=np.zeros(3)):
        super(vehicle, self).__init__(numberOfEntities)
        self.type              = vehType
        self.id                = id
        self.initialConditions = initialConditions
        if (initialConditions == 'random'):
            position      = (np.random.rand(3) * 2. - 1.)
            position[0]   = position[0] * worldLength
            position[1]   = position[1] * worldWidth
            position[2]   = 0.
            self.position = position
            velocity      = (np.random.rand(3) * 2. - 1.)
            velocity[0]   = velocity[0] * v0
            velocity[1]   = np.sqrt(v0**2 - velocity[0]**2)
            velocity[2]   = 0.
            self.velocity = velocity
            target        = (np.random.rand(3) * 2. - 1.)
            target[0]     = target[0] * worldLength
            target[1]     = target[1] * worldWidth
            target[2]     = 0.
            self.target   = target 
        if (initialConditions == 'area'):
            randomPositionInArea = np.random.rand(3)
            randomPositionInArea[0]   = randomPositionInArea[0] * (target[0] - position[0]) + position[0]
            randomPositionInArea[1]   = 1.0 * self.id / (numberOfEntities - 1) * (target[1] - position[1]) + position[1]
            randomPositionInArea[2]   = 0.
            self.position = randomPositionInArea
            randomTargetInArea = np.random.rand(3)
            randomTargetInArea[0]   = target2[0] + randomTargetInArea[0]*20
            randomTargetInArea[1]   = randomTargetInArea[1] * (target2[1] - position[1]) + position[1]
            randomTargetInArea[2]   = 0.
            self.target = randomTargetInArea
            self.velocity = np.array([0., 0., 0.])
            # print(self.id, self.position, self.target, self.velocity)
        if (initialConditions == 'defined'):
            self.position = position
            self.velocity = velocity
            self.target   = target
        self.color             = 'green'
        self.movementStatus    = 'driving'
        self.origin   = self.position
        
    def updatePosition(self, newPosition):
        self.position = newPosition
        
    def updateOrigin(self, newOrigin):
        self.origin = newOrigin
        
    def updateVelocity(self, newVelocity):
        self.velocity = newVelocity
