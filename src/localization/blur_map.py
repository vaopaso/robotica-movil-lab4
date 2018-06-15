from PIL import Image,ImageFilter,ImageOps
import numpy as np
import math
import time
# import pandas as pd
import sys

# from scipy.stats import norm
from weighting import mapCoords_to_pixel
# from c_space import isPainted


def get_equation(alpha):
    '''Returns the equation of the corresponding ray given alpha [-90,90]
        in the form of a*y = m*x + n ([m,a,n]'''
    if abs(alpha) > 90:
        raise ValueError('Alpha must be between -90 and 90')
    if alpha == 90 or alpha == -90:
        return [12982438723,1,0]
    return [math.tan(alpha*math.pi/180),1,0]

def transform_eq(eq,pose_robot):
    '''Transforms given equation into absolute coordinates of the map given the position of the robot
        pose = [x,y,theta]
        eq => a*y = m*x + n => [m,a,n]'''
    
    slope = eq[0]/eq[1]
    abs_angle = math.atan(slope)
    # print(abs_angle)
    abs_angle += pose_robot['theta']

    new_slope = math.tan(abs_angle)
    n = 0
    if new_slope > 0:
        n -= pose_robot['x']*new_slope
        n += pose_robot['y']
    elif new_slope < 0:
        n -= pose_robot['x']*new_slope
        n += pose_robot['y']

    # TODO: revisar casos borde, en los que el angulo es negativo, o
    # cuando la tangente da infinito.
    return [new_slope,eq[1],n]

def get_coordinates(sensors,pose_robot,discretization,angles):
    '''pose_robot[2] between -pi and pi'''
    disc = discretization #discretizacin (en metros)
    matrix_x = np.zeros_like(sensors)
    matrix_y = np.zeros_like(sensors)

    # TODO: given the absolute position of the robot, return the coordinates of the different points 
    # of the sensor measurements.
    # angles = np.linspace(angle_init,angle_end,sensors.shape[1])

    # print("angles",angles)
    # print("sensors shape",sensors.shape)
    for i in range(len(angles)):
    # for i in range(sensors.shape[1]): 
        eq = transform_eq(get_equation(angles[i]),pose_robot)
        # print(angles[i],eq)
        angle = math.atan(eq[0]) #between -pi and pi
        dir1 = angle
        dir2 = (angle + math.pi) 
        
        if dir2 > math.pi:
            dir2 -= 2*math.pi
        # dir2 -= 2*math.pi
        # print(angles[i],dir1,dir2)
        mult_coord = 1
        dir_chosen = None

        # print(a,b)
        a = math.atan2(math.sin(pose_robot['theta']-dir1), math.cos(pose_robot['theta']-dir1))
        b = math.atan2(math.sin(pose_robot['theta']-dir2), math.cos(pose_robot['theta']-dir2))
        
        # print(a,b)
        # print(abs((pose_robot['theta']- abs(dir1))%(2*math.pi)), abs((pose_robot['theta']- abs(dir2))%(math.pi)))
        if abs(a) < abs(b):
        # if abs((pose_robot['theta']- abs(dir1))%(2*math.pi)) < abs((pose_robot['theta']- abs(dir2))%(math.pi)) or angles[i] == -90:
            dir_chosen = dir1
        else:
            dir_chosen = dir2

        
        if angles[i] == -90:
            dir_chosen = pose_robot['theta'] - math.pi/2
            if dir_chosen < -math.pi:
                dir_chosen += 2*math.pi
        if angles[i] == 90:
            dir_chosen = pose_robot['theta'] + math.pi/2
            if dir_chosen > math.pi:
                dir_chosen -= 2*math.pi
        
        # print("dir_chosen",dir_chosen)
        
        # Si esta en el cuadrante 3 o 4.
        if abs(dir_chosen) > math.pi/2:
            mult_coord = -1

        

        #TODO: se puede optimizar usando operaciones matriciales.
        for j in range(sensors.shape[0]):
            matrix_x[j,i] = pose_robot['x'] + (j+1) * disc * math.cos(dir_chosen)
            matrix_y[j,i] = pose_robot['y'] + (j+1) * disc * math.sin(dir_chosen)
        # if angles[i] == 0.0:
        #     print(matrix_x[:,i])
        #     print(matrix_y[:,i])
            
    return matrix_x,matrix_y



# b = np.random.uniform(low=0.0,high=20.0,size=181)
# start_time = time.time()
# sensors = callback_scan(b)
# print(sensors.shape)
# m_x,m_y = get_coordinates(sensors,[3,1,math.pi/2])
# print(pd.DataFrame(m_x))
# # print(a[:,0].shape)
# print("")
# print(pd.DataFrame(m_y))
# print("--- %s seconds ---" % (time.time() - start_time))
# # print(get_equation(-90))
