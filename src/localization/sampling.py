# from scipy.stats import norm
import numpy as np
import matplotlib.pyplot as plt

from weighting import mapCoords_to_pixel
from c_space import isPainted

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

#last_particles: output new_particles del resampling -> lista de (coord_x,coord_y,theta)
#last y new odom: {'x':.., 'y':.., 'theta':..}


def test_sigmas():
    fig, ax = plt.subplots(1, 1)

    last_odom = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    new_odom = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
    diff = {'x': new_odom['x']-last_odom['x'], 'y': new_odom['y']-last_odom['y'], 'theta': new_odom['theta']-last_odom['theta']} 
    last_particle = [0,0,0]

    rho, phi = cart2pol(diff['x'], diff['y'])
    # print('rho: {0}, phi: {1}'.format(rho,phi))
    ax.scatter(last_particle[0], last_particle[1], c='blue')

    sigma_theta = 0.1
    sigma_rho = 0.05
    sigma_phi = 0.05 #0.1
    size = 100

    sample_theta = np.random.normal(diff['theta'], sigma_theta, size) # giro sobre si mismo
    sample_rho = np.random.normal(rho, sigma_rho, size) # distancia lineal recorrida
    sample_phi = np.random.normal(phi, sigma_phi, size) # angulo abertura

    data = [[],[]]

    for i in range(size):
        
        sample_x, sample_y = pol2cart(sample_rho[i], sample_phi[i])
        new_particle = [last_particle[0]+sample_x, last_particle[1]+sample_y, last_particle[2]+sample_theta]
        data[0].append(new_particle[0])
        data[1].append(new_particle[1])

    ax.scatter(data[0], data[1], c='red', marker='.')
    plt.show()

# test_sigmas()