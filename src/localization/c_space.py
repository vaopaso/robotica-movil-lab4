import copy
import numpy as np
# import pandas as pd

# from blur_map import first_generation_particles,resampling

def read_pgm(map_path):
    """Return a raster of integers from a PGM as a list of lists."""
    pgmf = open(map_path, 'rb')
    a = pgmf.readline()
    assert a == b'P5\n'
    (width, height) = [int(i) for i in pgmf.readline().split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    raster = []
    for i in range(height):
        row = []
        for j in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(np.array(row))
    return np.array(raster)

def print_matrix(matrix):
    for i in range(len(matrix)):
        row = ""
        for j in range(len(matrix[i])):
            row += str(matrix[i][j]) + "\t"
        print(row)
        print()

def paint(matrix, i, j):
    matrix[i][j] = 0

def isPainted(matrix, i, j):
    try:
        if matrix[i][j] == 0:
            return True
    except:
        # print("ERROR EN IS PAINTED",i,j)
        return True
    return False

def isPaintedOrUnexplored(matrix, i, j):
    try:
        if matrix[i][j] < 255:
            return True
    except:
        # print("ERROR EN IS PAINTED OR UNEXPLORED",i,j)
        return True
    return False

def isUnexplored(matrix,i,j):
    if i < 0 or j < 0:
        return True
    try:
        if matrix[i][j] == 205:
            return True
    except:
        # print("ERROR IN IS UNEXPLORED")
        return True
    return False

def paintNeighbours(matrix, i, j, radius, acc):
    acc += 1
    if acc <= radius:
        if not isPainted(matrix, i+1, j):
            paint(matrix, i+1, j)
            paintNeighbours(matrix, i+1, j, radius, acc)
        if not isPainted(matrix, i, j+1):
            paint(matrix, i, j+1)
            paintNeighbours(matrix, i, j+1, radius, acc)
        if not isPainted(matrix, i-1, j):
            paint(matrix, i-1, j)
            paintNeighbours(matrix, i-1, j, radius, acc)
        if not isPainted(matrix, i, j-1):
            paint(matrix, i, j-1)
            paintNeighbours(matrix, i, j-1, radius, acc)

if __name__ == "__main__":
    pass
    # matrix = read_pgm(map_path)
    # matrix_original = copy.deepcopy(matrix)
    # pgmf.close()
    # c_space(matrix_original, matrix, radius=2)
    # # print_matrix(matrix)
    # # particles = first_generation_particles(100,np.array(matrix),[0,0],0.1)
    # # prob = [0.4,0.2,0.3,0.1,0]
    # # print(dic)
    # # print("")
    # print(particles)
    # new_dic,new_part = resampling(100,matrix,prob,particles)
    # print(new_dic)
    # print("")
    # print(new_part)
    