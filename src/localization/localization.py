import numpy as np
from PIL import Image,ImageFilter,ImageOps
from scipy.stats import norm
import math
import copy
from collections import deque
import cv2
import json
import time

from std_msgs.msg import String
import rospy

from c_space import isPainted,isPaintedOrUnexplored,paintNeighbours,read_pgm,print_matrix
from blur_map import get_equation,get_coordinates,transform_eq
from weighting import mapCoords_to_pixel,isValidPixel,score_of_ray
from sampling import cart2pol,pol2cart

_RED = [0, 0, 255]
_BLUE = [255, 0, 0]
_GREEN = [0, 255, 0]
_BLACK = [0, 0, 0]

class Localization:
    def __init__(self,path_map,path_yaml,num_particles):
        self.map_no_blur = read_pgm(path_map)
        self.map = self.blur_map(path_map)
        self.sensors = None
        self.resolution = 0.1
        self.origin = [0,0]
        self.ray_discretization = 0.1 #se discretiza espacio de 20 metros en 201 pasos discretos (10 cm)

        self.located = False
        self.pose_robot = None
        self.last_pose = None

        self.is_located_publisher = rospy.Publisher("/is_located", String, queue_size=5)
        self.location_publisher = rospy.Publisher("/location", String, queue_size=5)

        self.initial_num_particles = num_particles
        self.num_particles = num_particles

        self.init_angle = -26
        self.end_angle = 30

        self.angles = np.linspace(self.init_angle,self.end_angle,self.end_angle-self.init_angle+1)

        #cspace
        radius = 0.2/self.resolution
        self.cspace_matrix = copy.deepcopy(self.map_no_blur)
        self.c_space(self.map_no_blur,self.cspace_matrix, radius=radius)

        self.particles = None

        #Display
        self.img = cv2.imread(path_map)

        self.augmentation = 10
        self.display_resolution = self.resolution/self.augmentation
        self.img = cv2.resize(self.img, (self.img.shape[0]*self.augmentation, self.img.shape[0]*self.augmentation))
        self.particle_radius = int(round(0.05/self.display_resolution))

        # bajo este score, esta ubicado.
        self.threshold_located = 0.2
        self.min_score_weighting = math.pow(10,-40)

        #no empieza ubicando hasta que reciba particula inicial.
        self.start_locating = False
        self.initial_pose = None

        self.valid_measure = True

        self.avg_particle = None
        self.num_hardcoded = int(0.1*self.initial_num_particles)

    def initial_pose_callback(self,data):
        print("calback")
        if not self.start_locating:
            print("exito")
            self.initial_pose = json.loads(str(data.data))
            self.start_locating = True     

    
    def odom_callback(self, data):
        pos = data.pose.pose.position
        self.pose_robot = {'x':pos.x,'y':pos.y}

        if self.last_pose is None:
            self.last_pose = self.pose_robot

        angaux = data.pose.pose.orientation.w
        angaux = 2 * math.acos( angaux )
        if data.pose.pose.orientation.z < 0:
           angaux = 2*math.pi - angaux
        self.pose_robot['theta'] = angaux

    def timer_located(self,event):
        particles = copy.deepcopy(self.particles)
        score = self.score_distance(particles)
        print("score",score)
        is_located = False
        avg = None
        if score <= self.threshold_located:
            is_located = True
            data = np.zeros([len(particles), 3])
            for i, particle in enumerate(particles):
                data[i, 0] = particle['x']
                data[i, 1] = particle['y']
                data[i,2] = particle['theta']
            avg = np.average(data,axis=0)
            avg = {'x': avg[0], 'y': avg[1], 'theta': avg[2]}
            self.avg_particle = avg

        self.is_located_publisher.publish(String(json.dumps(is_located)))
        self.location_publisher.publish(String(json.dumps(avg)))
        

    def locate(self):
        '''Tick in charge of the location process'''
        self.last_pose = copy.deepcopy(self.pose_robot)
        if self.particles is None or len(self.particles) == 0 :
            #Resampling Inicial
            # input("first resampling")
            self.num_particles = self.initial_num_particles
            # self.angles = np.linspace(self.init_angle,self.end_angle,self.end_angle-self.init_angle+1)
            self.particles = self.first_generation_particles(self.initial_num_particles,self.cspace_matrix,self.origin,self.resolution)
            # self.particles[-1] = {'x':0.5,'y':1.3,'theta':0}
            if self.avg_particle is not None:
                self.particles[-self.num_hardcoded:] = self.sparse_particle(self.avg_particle,self.num_hardcoded)
                self.particles[-1] = self.avg_particle
            else:
                self.particles[-self.num_hardcoded:] = self.sparse_particle(self.initial_pose,self.num_hardcoded)
                self.particles[-1] = self.initial_pose
            print("hice first resampling")
        # input("hola espero")
        #weighting
        
        sensors = copy.deepcopy(self.sensors) #se hace freeze del valor de sensors en este minuto.
        sensors,angles = self.get_useful_rays(sensors,self.num_particles)
        valid_measure = copy.deepcopy(self.valid_measure)
        print("valid measure",valid_measure)

        # print("sensors")
        
        # print("sensors_shape",sensors.shape)
        # print("angle_shape",angles.shape)
        # print_matrix(self.sensors.round(2))
        # print("")

        t = time.time()
        # print("weighting")
        scores = []
        for particle in self.particles:
            # print("particle",particle)
            t0 = time.time()
            # print('angles:', self.angles)
            x_matrix,y_matrix = get_coordinates(sensors,particle,self.ray_discretization,angles)
            # print('[Weighting particle] get_coordinates time: {0}ms.'.format(round(1000*(time.time()-t0), 2)))
            t0 = time.time()
            score = self.weighting(sensors,x_matrix,y_matrix,self.map,self.resolution,self.origin)
            # print('[Weighting particle] weighting: {0}ms.'.format(round(1000*(time.time()-t0), 2)))
            # print("particle_score",score)
            scores.append(score)
        # print('Weighting time: {0}ms.'.format(round(1000*(time.time()-t), 2)))

        if np.sum(scores)>0 and len(scores)>0:# and max(scores) > self.min_score_weighting:
            print("max score",max(scores))
            if not valid_measure or max(scores) > self.min_score_weighting:
                scores = scores/np.sum(scores)
                #resampling
                # print("resampling")
                self.particles = self.resampling(self.num_particles,self.cspace_matrix,scores,self.particles)
                # self.last_pose = copy.deepcopy(self.pose_robot)
            else:
                print("mate particulas")
                self.particles = None
        else:
            print("mate particulas")
            self.particles = None
            # self.last_pose = copy.deepcopy(self.pose_robot)

        # print('locate() time: {0}ms.'.format(round(1000*(time.time()-t_all), 2)))

    def sampling_process(self):
        #sampling
        # print("sampling")
        if self.particles is not None:
            new_pose = copy.deepcopy(self.pose_robot)
            self.particles = self.sampling(self.particles,self.last_pose,new_pose,self.cspace_matrix)

    def c_space(self,matrix_original, matrix, radius):
        #radius: radio del robot (en pixeles de mapa)
        for i in range(len(matrix_original)):
            for j in range(len(matrix_original[i])):
                if isPainted(matrix_original, i, j):
                    paintNeighbours(matrix, i, j, radius, 0)

    def blur_map(self,map_path,radius=0.9):
        '''Returns blurred map according to radius'''
        # print("hola2")
        
        h = cv2.imread(map_path,cv2.IMREAD_GRAYSCALE)
        h[h==205] = 245
        h2 = np.divide(np.array([255.0- val for val in h.flatten()]).reshape((h.shape[0],h.shape[1])),255)

        blurred_prob_map = cv2.GaussianBlur(h2,(5,5),0,borderType=cv2.BORDER_CONSTANT)

        # ####################################
        # blur =  Image.open(map_path).filter(ImageFilter.GaussianBlur(radius=radius))
        # image = np.array(blur.getdata())
        # # print(image)

        # # func = 
        # # image = np.array([0 for val in image])
        # blurred_prob_map = np.divide(np.array([255.0-val for val in image]),255).reshape(blur.size[0], blur.size[1])
        # # blurred_prob_map = np.array([255.0-val for val in image]).reshape(blur.size[0], blur.size[1])
        # blurred_prob_map = np.add(blurred_prob_map,0.01)
        # # return blurred_prob_map.round(2)
        return blurred_prob_map

    def get_useful_rays(self,sensors,num_particles):
        ##################### Eficientizacion #######################################
        #self.angles tiene los angulos que queremos tomar
        #Depende del numero de particulas que hayan: 10 particulas para abajo: todos los rayos (57). 
        #                                            300 particulas, cuatro rayos
        #Interpolacion entre ambos.
        num_max_rays = int(round((self.end_angle - self.init_angle +1.0)*0.8))
        # print("maximo_rays",num_max_rays)
        # num_max_rays = 45
        num_min_rays = 3
        p1 = (20,num_max_rays)
        p2 = (300,num_min_rays)
        m = (p1[1]-p2[1])*1.0/(p1[0]-p2[0])
        num_rayos_func = lambda num_particulas: -m*p2[0]+p2[1]+m*num_particulas
        num_rayos = num_rayos_func(num_particles)
        
        if num_rayos < num_min_rays:
            num_rayos = num_min_rays
        elif num_rayos > num_max_rays:
            num_rayos = num_max_rays
        num_rayos = int(round(num_rayos))
        # print("num_particulas",num_particles)
        # print("num_rayos",num_rayos)

        choices_func = lambda m, n: [i*n//m + n//(2*m) for i in range(m)]
        choices = choices_func(num_rayos,sensors.shape[1]-1)
        sensors = sensors[:int(round(4.0/self.resolution)),choices]
        angles = np.linspace(self.init_angle,self.end_angle,self.end_angle-self.init_angle+1)[choices]
        return sensors,angles
        
        #############################################################################  
    
    def callback_scan(self,data):
        ranges_orig = np.array(data.ranges)
        # ranges = np.array(data)
        zero_angle_index = ranges_orig.shape[0] //2 
        ranges = ranges_orig[zero_angle_index+self.init_angle:zero_angle_index+self.end_angle+1]

        if len(ranges[ np.where( ranges > 19.0 )]) > len(ranges)*0.3:
            self.valid_measure = False
        else:
            self.valid_measure = True

        ranges = np.add(ranges,0.08)
        # print("ranges",ranges)
        disc = int(round(20/self.ray_discretization))+1
        var = 15
        rays = ranges.shape[0]
        sensors = np.zeros((disc,rays))
        for i in range(len(ranges)):
            value = ranges[i]
            value = int(round(value * 1.0/self.resolution))
            sensor_model = deque(norm.pdf(np.linspace(norm.ppf(0.001,scale=var),norm.ppf(0.999,scale=var),disc)))
            # print(sensor_model)
            # sensor_model = norm.pdf(np.linspace(norm.ppf(0.001,scale=var),norm.ppf(0.999,scale=var),disc))
            half = disc // 2
            if value > half:
                # shift de distribucion hacia la derecha.
                shift = value - half
                sensor_model.rotate(shift)
                sensor_model = list(sensor_model)
                sensor_model[:shift] = [0]*shift
                # sensor_model = np.pad(sensor_model, (shift,0), 'constant', constant_values=0)[:sensor_model.shape[0]]
            elif value < half:
                # shift de distribucion hacia la izquierda.
                shift = half - value
                sensor_model.rotate(-shift)
                sensor_model = list(sensor_model)
                sensor_model[-shift:] = [0]*shift
                # sensor_model = np.pad(sensor_model, (0,shift), 'constant', constant_values=0)[shift:]
            # print(sensor_model)
            # return sensor_model
            sensors[:,i] = np.array(sensor_model)
        sensors = np.add(sensors,1.0/disc)
        # print(sensors)
        # print(list(sensors[:,abs(self.init_angle)]))
        
            
        # print(ranges)
        # return sensors
        self.sensors = sensors
        # self.sensors = sensors[:int(round(4.0/self.resolution)), :]

        # print("shape_sensors",self.sensors.shape)
        # print("angles_shape",self.angles.shape)

    def first_generation_particles(self,num_particles,cspace_matrix,origin,resolution):
        '''creates the first random set of particles given c-space and map
            origin = [origin_x,origin_y]
            resolution is the amount of meters of each pixel.
        '''
        
        num_made = 0
        range_x = cspace_matrix.shape[0]*resolution
        range_y = cspace_matrix.shape[1]*resolution
    
        # robot_positions diccionario con los valores siendo listas de thetas en cada (row,column)
        particles = []
        # print_matrix(cspace_matrix)
        while num_made < num_particles:
            # TODO: random row, column and theta
            x = np.random.uniform(low=origin[0],high=origin[0]+range_x)
            y = np.random.uniform(low=origin[1],high=origin[1]+range_y)
            # TODO: check if that spot is available
            row,column = mapCoords_to_pixel(x,y,cspace_matrix.shape[0],cspace_matrix.shape[1],origin,resolution)
            # print(x,y,row,column)

            if not isPaintedOrUnexplored(cspace_matrix,row,column):
                # print("se crea")
                theta = np.random.uniform(0,math.pi*2)
                particles.append({'x':x,'y':y,'theta':theta})

                num_made += 1
            # if cspace_matrix[row,column] == 0:
            #     theta = np.random.uniform(0,math.pi*2)
            #     particles.append({'x':x,'y':y,'theta':theta})
            #     num_made += 1
        # particles.append({'x':0.3,'y':0.5,'theta':-math.pi/4})
        np.random.shuffle(particles)
        return particles

    def score_distance(self, particles):
        '''average euclidean distance between all points.'''
        data = np.zeros([len(particles), 2])
        for i, particle in enumerate(particles):
            data[i, 0] = particle['x']
            data[i, 1] = particle['y']

        tot = 0.0
        if data.shape[0] <= 1:
            return 0
        for i in xrange(data.shape[0]-1):
            tot += ((((data[i+1:]-data[i])**2).sum(1))**0.5).sum()
        avg = tot/((data.shape[0]-1)*(data.shape[0])/2.0)
        return avg

        


    def resampling(self,num_particles,cspace_matrix,prob_array,previous_particles):
        '''creates new particles given the score for each particle'''
        # el largo de prob_array da la cantidad de particulas anteriores. El indice corresponde con la particula.
        # previous particles es una lista de (row,column,theta) como el generado en first_generation_particles.
        new_particles = []

        # print(prob_array)

        ################################# Eficientizacion ##############################################
        #calcular score segun que tan separadas estan, y hacer una interpolacion para ver numero de particulas.
        score = self.score_distance(self.particles)
        # print('score distance:',score)
        #interpolacion
        max_particles = self.initial_num_particles
        min_particles = 20
        max_score = 3.5
        min_score = 0.1
        m = (max_particles-min_particles)/(max_score-min_score)
        particles_func = lambda score: max_particles+m*(score-max_score)

        self.num_particles = particles_func(score)
        if self.num_particles > max_particles:
            self.num_particles = max_particles
        elif self.num_particles < min_particles:
            self.num_particles = min_particles
        # print('nuevo nro particulas:',self.num_particles)
        ################################################################################################


        indexes = np.random.multinomial(self.num_particles, prob_array, size=1)[0]
        # print("shape indexes",len(indexes))
        for i in range(len(indexes)):
            for _ in range(indexes[i]):
                previous_particle = previous_particles[i]
                new_particles.append(previous_particle)
        # print("new_particles",new_particles)
        return new_particles

    def weighting(self,rays_matrix, x_matrix, y_matrix, mapa, resolution, bottom_left_origin):
        # rays_matrix: Matriz donde cada columna es un rayo y cada fila el valor de la medicion a una distancia especifica
        # mapa: mapa blurred
        if not ((rays_matrix.shape == x_matrix.shape) and (x_matrix.shape == y_matrix.shape)):
            raise ValueError("Error de dimensiones de matrices")

        # width_pixels = 27
        # height_pixels = 27
        # resolution = 0.1
        # bottom_left_origin = [-0.9, -1.3, 0.0]
        # print_matrix(mapa.round(2))
        width_pixels = mapa.shape[1]
        height_pixels = mapa.shape[0]
        cols = rays_matrix.shape[1]
        scores = np.empty([cols])
        for i in range(cols):
            scores[i] = score_of_ray(rays_matrix[:,i], x_matrix[:,i], y_matrix[:,i], mapa, self.map_no_blur, width_pixels, height_pixels , bottom_left_origin, resolution)
        
        # print(scores)
        product = 1
        aux = False
        # print(scores)
        for score in scores:
            if score > 0.005:
                aux = True
                product *= score
        if not aux:
            product = 0
        return product

    def rotate(self, origin, point, angle):
        """
        Rotate a point counterclockwise by a given angle around a given origin.
        The angle should be given in radians.
        """
        ox, oy = origin
        px, py = (point[0],point[1])

        qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
        qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
        return qx, qy
    
    def absolute_pos(self, robot_relative_pos, robot_absolute_pos):
        p_rot = self.rotate((0,0),robot_relative_pos, robot_absolute_pos[2])
        theta_abs = robot_relative_pos[2]+robot_absolute_pos[2]
        if theta_abs > 2*math.pi:
            theta_abs = 2*math.pi - theta_abs
        return (p_rot[0]+robot_absolute_pos[0],p_rot[1]+robot_absolute_pos[1],theta_abs)

    def absolute_diff(self,last_odom,new_odom,last_particle):
        # print("last_odom",last_odom)
        # print("new_odom",new_odom)
        # print("particle",last_particle)
        theta1 = last_odom['theta']
        theta2 = new_odom['theta']
        diff_theta = theta2-theta1
        if diff_theta < -math.pi:
            diff_theta = 2*math.pi + diff_theta
        elif diff_theta > math.pi:
            diff_theta -= 2*math.pi
        diff = [new_odom['x']-last_odom['x'],  new_odom['y']-last_odom['y'], diff_theta]
        # print("diff_original",diff)
        diff_relativa = list(self.rotate([0,0],diff[:2],-last_odom['theta']))+[diff[2]]
        # print("diff_relativa",diff_relativa)
        diff_absoluta = list(self.rotate([0,0],diff_relativa[:2],last_particle['theta'])) + [diff_relativa[2]]
        return diff_absoluta
    
    def sampling(self,last_particles, last_odom, new_odom, cspace_matrix):

        theta1 = last_odom['theta']
        theta2 = new_odom['theta']
        diff_theta = theta2-theta1
        if diff_theta < -math.pi:
            diff_theta = 2*math.pi + diff_theta
        elif diff_theta > math.pi:
            diff_theta -= 2*math.pi
        print("diff_theta",diff_theta)
        new_particles = []
        for i in range(len(last_particles)):
            diff = self.absolute_diff(last_odom,new_odom,last_particles[i])
            # print("diff",diff)
            diff = {'x':diff[0],'y':diff[1],'theta':diff[2]}
            # print('diff_absoluta',diff)
            rho, phi = cart2pol(diff['x'], diff['y'])
            
            # sigma_theta = 0.8
            # sigma_rho = 1.6
            # sigma_phi = sigma_rho/7
            sigma_theta = 0.15 #0.3
            sigma_rho = 0.05
            sigma_phi = 0.05
            
            # se "mueven" las particulas segun los modelos de probabilidad
            extra_theta = 1
            sample_theta = np.random.normal(diff['theta']*extra_theta, sigma_theta) # giro sobre si mismo
            sample_rho = np.random.normal(rho, sigma_rho) # distancia lineal recorrida
            sample_phi = np.random.normal(phi, sigma_phi) # angulo abertura

            
            sample_x, sample_y = pol2cart(sample_rho, sample_phi)
            x,y = last_particles[i]['x']+sample_x,last_particles[i]['y']+sample_y
            row,column = mapCoords_to_pixel(x,y,cspace_matrix.shape[0],cspace_matrix.shape[1],self.origin,self.resolution)
            
            theta = (last_particles[i]['theta']+sample_theta)
            if theta < -math.pi:
                theta += 2*math.pi
            elif theta > math.pi:
                theta -= 2*math.pi
            # print("New particle: x: {0}, y: {1}, theta: {2}".format(x,y,theta))
            new_particle = {'x':x, 'y':y,'theta': theta}
            if not isPaintedOrUnexplored(self.map_no_blur,row,column):
                new_particles.append(new_particle)
            else:
                pass
                # print("no se puede agregar",new_particle)

        return new_particles
        

    def sparse_particle(self,particle,number):
        particles = []
        for i in range(number):
            diff = {'x':0,'y':0,'theta':0}
            rho, phi = cart2pol(diff['x'], diff['y'])

            # sigma_theta = 0.8
            # sigma_rho = 1.6
            # sigma_phi = sigma_rho/7
            sigma_theta = 0.1 #0.3
            sigma_rho = 0.05
            sigma_phi = 0.05

            # se "mueven" las particulas segun los modelos de probabilidad
            extra_theta = 1
            sample_theta = np.random.normal(diff['theta']*extra_theta, sigma_theta) # giro sobre si mismo
            sample_rho = np.random.normal(rho, sigma_rho) # distancia lineal recorrida
            sample_phi = np.random.normal(phi, sigma_phi) # angulo abertura

            sample_x, sample_y = pol2cart(sample_rho, sample_phi)
            x,y = particle['x']+sample_x,particle['y']+sample_y
            #row,column = mapCoords_to_pixel(x,y,cspace_matrix.shape[0],cspace_matrix.shape[1],self.origin,self.resolution)

            theta = (particle['theta']+sample_theta)
            if theta < -math.pi:
                theta += 2*math.pi
            elif theta > math.pi:
                theta -= 2*math.pi
            
            particle = {'x':x, 'y':y,'theta': theta}
            particles.append(particle)
        return particles

    def plotParticles(self, event):
        image = copy.deepcopy(self.img)
        # rad = 0.14 #radio real robot
        rad = self.particle_radius*self.display_resolution

        if self.particles is not None:
        
            for particle in self.particles:
                i, j = mapCoords_to_pixel(particle['x'], particle['y'], image.shape[0], image.shape[1] , self.origin, self.display_resolution)
                # print(particle['x'],i,particle['y'],j)
                if isValidPixel(i, j, image.shape[0], image.shape[1]):
                    # draw robot circle
                    cv2.circle(image, (j,i), self.particle_radius, _RED, -1)
                    # draw direction
                    y_prima = rad*np.sin(particle['theta']) + particle['y']
                    x_prima = rad*np.cos(particle['theta']) + particle['x']
                    # print(x_prima,y_prima)
                    # x_prima,y_prima = mapCoords_to_pixel(x_prima,y_prima,image.shape[0],image.shape[1],bottom_left_origin,resolution)
                    # y_prima = int(round(particle_radius*np.sin(particle['theta'])))
                    # x_prima = int(round(particle_radius*np.cos(particle['theta'])))
                    x_prima,y_prima = mapCoords_to_pixel(x_prima,y_prima,image.shape[0],image.shape[1],self.origin,self.display_resolution)
                    cv2.line(image, (j,i), (y_prima,x_prima), _BLACK, 1)

        # Display window
        cv2.namedWindow('Localization', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Localization', 600,600)
        cv2.startWindowThread()
        cv2.imshow('Localization', image)
