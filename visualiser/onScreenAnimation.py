#!/usr/bin/python

from spawners.pedestrianSpawner import *
from spawners.vehicleSpawner import *

from matplotlib import pyplot as plt
import matplotlib

from behaviours.propagateParameters import *
from behaviours.propagateDynamics import *

from world.worldParameters import worldLength, worldWidth

from mathlib.mathFunctions import *

nbStandardPedestrians = 0
nbStandardCars = 4

position1 = np.array([-worldLength, -worldWidth/2, 0.0]) 
position2 = np.array([-worldLength+worldLength/5, worldWidth/2, 0.0])
velocity  = np.zeros(3)
target = np.array([worldLength, worldWidth/4, 0.0])

pedestrians = spawnPedestrians(nbStandardPedestrians).spawnRandomlyStandardPedestrians()
cars        = spawnCars(nbStandardCars).spawnStandardCarsInArea(position1=position1, position2=position2, velocity=velocity, target=target)

figure  = plt.figure()
axes    = plt.axes(xlim=(-worldLength, worldLength), ylim=(-worldWidth, worldWidth)) 
for i in range(nbStandardPedestrians):
    plt.plot(pedestrians[i].position[0], pedestrians[i].position[1], 'bo')
    plt.plot(pedestrians[i].target[0], pedestrians[i].target[1], 'ro')
for i in range(nbStandardCars):
    car = matplotlib.patches.Ellipse((cars[i].position[0], cars[i].position[1]), cars[i].length, cars[i].width, angle=0., color=cars[i].color)
    axes.add_patch(car)
plt.ion()
plt.show()

newVelocity = np.zeros((nbStandardPedestrians,3)) 
newPosition = np.zeros((nbStandardPedestrians,3)) 
newVelocityCars = np.zeros((nbStandardCars,3)) 
newPositionCars = np.zeros((nbStandardCars,3)) 

walls = 0.
buildings = 2

for t in time:
    plt.cla()
    plt.xlim([-worldLength, worldLength])
    plt.ylim([-worldWidth, worldWidth])
    for currentPedestrian in range(nbStandardPedestrians):
        newVelocity[currentPedestrian], newPosition[currentPedestrian] = propagateInTime(dt, pedestrians[currentPedestrian], pedestrians, cars, walls, buildings)
    for currentPedestrian in range(nbStandardPedestrians):
        pedestrians[currentPedestrian].velocity = newVelocity[currentPedestrian] * pedestrianStopAtDestination(newPosition[currentPedestrian] - pedestrians[currentPedestrian].target)
        pedestrians[currentPedestrian].position = newPosition[currentPedestrian]
    for i in range(nbStandardPedestrians):         
        plt.plot(pedestrians[i].position[0], pedestrians[i].position[1], 'bo')  
        plt.plot(pedestrians[i].target[0], pedestrians[i].target[1], 'ro')     
    for currentCar in range(nbStandardCars):
        newVelocityCars[currentCar], newPositionCars[currentCar] = propagateInTime(dt, cars[currentCar], pedestrians, cars, walls, buildings)
    for currentCar in range(nbStandardCars):
        cars[currentCar].velocity = newVelocityCars[currentCar] * carStopAtDestination(newPositionCars[currentCar] - cars[currentCar].target)
        cars[currentCar].position = newPositionCars[currentCar]
        angle = acos(np.dot(cars[currentCar].velocity, np.array([1., 0., 0.]))/np.linalg.norm(cars[currentCar].velocity))
        if (cars[currentCar].velocity[1] < 0.):
            angle = -angle
        car = matplotlib.patches.Ellipse((cars[currentCar].position[0], cars[currentCar].position[1]), 
                                          cars[currentCar].length, cars[currentCar].width, angle=degrees(angle), color=cars[currentCar].color)
        axes.add_patch(car)
               
    plt.draw()
    plt.pause(0.001)