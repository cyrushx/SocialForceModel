#!/usr/bin/python

from spawners.pedestrianSpawner import *
from spawners.vehicleSpawner import *

from matplotlib import pyplot as plt
import matplotlib

from behaviours.propagateParameters import *
from behaviours.propagateDynamics import *

from world.worldParameters import worldLength, worldWidth

from mathlib.mathFunctions import *

from datetime import datetime
import hashlib
import os
import sys
import transforms3d
import tqdm
import copy

from snippet_writers import normalize_positions_list, create_normalizing_pose, normalize_track, save_adovehicle_track

nbStandardPedestrians = 0
nbStandardCars = 5

starting_region_lowerleft = np.array([-worldLength, -worldWidth/3, 0.0]) 
starting_region_upperright = np.array([-worldLength+worldLength/8, worldWidth/3, 0.0])
starting_velocity  = np.zeros(3)
target = np.array([worldLength, worldWidth/3, 0.0])

pedestrians = spawnPedestrians(nbStandardPedestrians).spawnRandomlyStandardPedestrians()
cars        = spawnCars(nbStandardCars).spawnStandardCarsInArea(position1=starting_region_lowerleft, 
    position2=starting_region_upperright, velocity=starting_velocity, target=target)

figure  = plt.figure()
axes    = plt.axes(xlim=(-worldLength, worldLength), ylim=(-worldWidth, worldWidth)) 
for i in range(nbStandardPedestrians):
    plt.plot(pedestrians[i].position[0], pedestrians[i].position[1], 'bo')
    plt.plot(pedestrians[i].target[0], pedestrians[i].target[1], 'ro')
for i in range(nbStandardCars):
    car = matplotlib.patches.Ellipse((cars[i].position[0], cars[i].position[1]), cars[i].length, cars[i].width, angle=0., color=cars[i].color)
    axes.add_patch(car)
plt.ion()
plt.axis('scaled')
plt.show()

newVelocity = np.zeros((nbStandardPedestrians,3)) 
newPosition = np.zeros((nbStandardPedestrians,3)) 
newVelocityCars = np.zeros((nbStandardCars,3)) 
newPositionCars = np.zeros((nbStandardCars,3)) 

walls = 0.
buildings = 0

current_time = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
hash_object = hashlib.sha1(current_time.encode('utf-8'))
pbHash = hash_object.hexdigest()
save_dir = 'SFM_data' + '/' + pbHash + '/'


def save_SFMvehicles(cars, state, save_dir, time_step, min_length=10, single_time_only=True):
    os.makedirs(save_dir,exist_ok=True)

    if state is None:
        state={'actors':{},'time':0,'file_counter':0,'step_counter':0}

    state['time'] = state['time'] + time_step
    state['step_counter'] = state['step_counter'] +1

    for i in range(nbStandardCars):
        currentCar = cars[i]
        act_id = currentCar.id
        position = currentCar.position
        velocity = currentCar.velocity

        angle = acos(np.dot(velocity, np.array([1., 0., 0.]))/np.linalg.norm(velocity))
        if (velocity[1] < 0.):
            angle = -angle

        R = transforms3d.euler.euler2mat(0.0, 0.0, angle).T

        pos_v = [position[0], position[1]]
        vel_v = []

        if (act_id not in state['actors']):
            state['actors'][act_id]={}
            state['actors'][act_id]['track'] = {'position_L':[pos_v],'timestamp':[state['time']],'heading_L':R[:2,:2].tolist()}
            state['actors'][act_id]['unique_id']=str(act_id)
            state['actors'][act_id]['nearby_agents']=[]
        else:
            state['actors'][act_id]['track']['position_L'].append(pos_v)
            state['actors'][act_id]['track']['timestamp'].append(state['time'])
            state['actors'][act_id]['track']['heading_L'].append(R[:2,:2].tolist())

        state['actors'][act_id]['track_time'] = state['time']

        # keep track length at min_length
        if len(state['actors'][act_id]['track']['timestamp']) > min_length:
            state['actors'][act_id]['track']['timestamp']=state['actors'][act_id]['track']['timestamp'][1:]
            state['actors'][act_id]['track']['position_L'] = state['actors'][act_id]['track']['position_L'][1:]
            state['actors'][act_id]['track']['heading_L'] = state['actors'][act_id]['track']['heading_L'][1:]

    for currentCar in tqdm.tqdm(cars):
        act_id = currentCar.id
        if len(state['actors'][act_id]['track']['timestamp']) == min_length and state['step_counter']>(min_length*2):
            # normalizing_pose = [R[:,2].tolist(),pos_v.tolist()]
            # TODO: remove copy once this is read-only
            track_data=copy.deepcopy(state['actors'][act_id])
            output_dict={'nearby_agents':[],'unique_id':track_data['unique_id']}
            # normalize track_data
            future_length=5
            normalizing_pose = create_normalizing_pose(track_data['track'], future_length,velocity_threshold=0.01)
            if (normalizing_pose is None):
                continue
            track=normalize_track(track_data['track'],normalizing_pose)
            for k in ['position_L','heading_L','timestamp']:
                output_dict[k]=track[k]
            R, t = normalizing_pose
            output_dict['normalizing_pose']= {'R':R.tolist(), 't':t.tolist()}

            file_counter = state['file_counter']
            file_prefix = 'SFM_agent'
            for currentCar2 in cars:
                act2_id = currentCar2.id

                # @todo: filter nearby agents based on distances
                if act2_id == act_id:
                    continue

                track_data2=copy.deepcopy(state['actors'][act2_id])
                track_data2 = normalize_track(track_data2['track'], normalizing_pose)
                # add unique id
                track_data2['unique_id'] = act2_id
                output_dict['nearby_agents'].append(track_data2)

            track_data['timestamp'] = state['time']

            if (single_time_only):
                for k in ['position_L', 'heading_L', 'timestamp']:
                    output_dict[k]=output_dict[k][-1:]
                for i in range(len(output_dict['nearby_agents'])):
                    for k2 in ['position_L', 'heading_L', 'timestamp']:
                        output_dict['nearby_agents'][i][k2] = output_dict['nearby_agents'][i][k2][-1:]

            save_adovehicle_track(file_prefix, save_dir, file_counter, output_dict, unique_id=output_dict['unique_id'])
            # print('.')
            state['file_counter'] =state['file_counter'] +1

    return state

save_state = None
for t in simulation_frames:
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
    
    # update car positions
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

    save_state = save_SFMvehicles(cars, save_state, save_dir, dt)    
    plt.draw()
    plt.pause(0.001)