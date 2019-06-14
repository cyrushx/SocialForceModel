#!/usr/bin/python

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from world.worldParameters import *

from spawners.pedestrianSpawner import *
from spawners.vehicleSpawner import *

from behaviours.propagateDynamics import *

from datetime import datetime

nbStandardPedestrians = 0
nbStandardCars = 5

starting_region_lowerleft = np.array([-worldLength, -worldWidth/3, 0.0]) 
starting_region_upperright = np.array([-worldLength+worldLength/8, worldWidth/3, 0.0])
starting_velocity  = np.zeros(3)
target = np.array([worldLength, worldWidth/3, 0.0])

pedestrians = spawnPedestrians(nbStandardPedestrians).spawnRandomlyStandardPedestrians()
cars        = spawnCars(nbStandardCars).spawnStandardCarsInArea(position1=starting_region_lowerleft, 
    position2=starting_region_upperright, velocity=starting_velocity, target=target)

figure       = plt.figure()
axes         = plt.axes(xlim=(-worldLength, worldLength), ylim=(-worldWidth, worldWidth)) 
plt.axis('scaled')
# pedDots      = axes.plot([], [], 'bo')
 
carEllipses = [matplotlib.patches.Ellipse((cars[i].position[0], cars[i].position[1]), cars[i].length, cars[i].width, angle=0., color=cars[i].color) for i in range(nbStandardCars)]
for carEllipse in carEllipses:
    axes.add_patch(carEllipse)
 
newVelocity = np.zeros((nbStandardPedestrians,3)) 
newPosition = np.zeros((nbStandardPedestrians,3)) 
newVelocityCars = np.zeros((nbStandardCars,3)) 
newPositionCars = np.zeros((nbStandardCars,3)) 

walls = 0.
buildings = 0

def init():
    # pedDots.set_data([], [])
    for i in range(nbStandardCars):
        carEllipses[i].set_visible(False)
    return carEllipses
 
def animate(frames):
    # print(frames)
    if frames == 1:
        for i in range(nbStandardCars):
            carEllipses[i].set_visible(True) 
    t = simulation_frames[frames]
    for currentPedestrian in range(nbStandardPedestrians):
        newVelocity[currentPedestrian], newPosition[currentPedestrian] = propagateInTime(dt, pedestrians[currentPedestrian], pedestrians, cars, walls, buildings)
    for currentPedestrian in range(nbStandardPedestrians):
        pedestrians[currentPedestrian].velocity = newVelocity[currentPedestrian]
        pedestrians[currentPedestrian].position = newPosition[currentPedestrian]
    x = [pedestrians[i].position[0] for i in range(nbStandardPedestrians)]
    y = [pedestrians[i].position[1] for i in range(nbStandardPedestrians)]   
    # pedDots.set_data(x, y)
     
    for currentCar in range(nbStandardCars):
        newVelocityCars[currentCar], newPositionCars[currentCar] = propagateInTime(dt, cars[currentCar], pedestrians, cars, walls, buildings)
    for currentCar in range(nbStandardCars):
        cars[currentCar].velocity = newVelocityCars[currentCar]
        cars[currentCar].position = newPositionCars[currentCar]
        angle = acos(np.dot(cars[currentCar].velocity, np.array([1., 0., 0.]))/np.linalg.norm(cars[currentCar].velocity))
        if (cars[currentCar].velocity[1] < 0.):
            angle = -angle
        carEllipses[currentCar].angle  = degrees(angle)
        carEllipses[currentCar].center = (cars[currentCar].position[0], cars[currentCar].position[1])            
    return carEllipses

anim=animation.FuncAnimation(figure, animate, init_func=init, frames=len(simulation_frames), interval=20, blit=True)

file_name = '%d_vehicles_%s.mp4' % (nbStandardCars, datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
anim.save(file_name, bitrate=-1)
print('video recorded.')
