import numpy as np
from c_space import isUnexplored

def mapCoords_to_pixel(x, y, width_pixels, height_pixels , bottom_left_origin, resolution):
    upper_left_origin = np.empty([2])
    upper_left_origin[0] = bottom_left_origin[0]
    upper_left_origin[1] = bottom_left_origin[1]+(height_pixels-1)*resolution
    # upper_left_origin[2] = bottom_left_origin[2]
    i = int((upper_left_origin[1]-y)/resolution)
    j = int((x-upper_left_origin[0])/resolution)
    # print(i,j)
    return i, j

def isValidPixel(i, j , width_pixels, height_pixels):
    if i >= height_pixels or i < 0:
        return False
    if j >= width_pixels or j < 0:
        return False
    return True

def score_of_ray(val, x, y, blured_map, original_map, width_pixels, height_pixels , bottom_left_origin, resolution):
    # val: vector de medicion de un solo rayo (con sensor model aplicado)
    # x: vector de coordenadas x de los valores dados en val
    # y: vector de coordenadas y de los valores dados en val
    # mapa: mapa blurred
    if not ((len(val) == len(x)) and (len(x) == len(y))):
        raise ValueError("Error de largos de input")
    size = len(val)
    vec = np.zeros([size])
    mp = np.zeros([size])
    for s in range(size):
        i, j = mapCoords_to_pixel(x[s], y[s], width_pixels, height_pixels , bottom_left_origin, resolution)
        
        if isValidPixel(i, j, width_pixels, height_pixels): #and not isUnexplored(original_map, i, j):
            # print("valido", i , j)
            vec[s] = blured_map[i, j] * val[s]
            mp[s] = blured_map[i, j]
        else:
            pass
            # print("no valido", i, j)
    score = sum(vec)
    # print("sensor model", val)
    # print("mp", mp)
    # print(score, vec)
    return score

