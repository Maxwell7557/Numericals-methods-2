import numpy as np
from numpy import linalg
import math
import time
from sys import exit
from sys import argv

def makeMatrix(type, size, alpha, beta) :
    first, last = [ [0 for i in range(size)] ], [ [0 for i in range(size)] ]
    center = [ [0 for i in range(size)] for z in range(size-2)]

    if type == '--default' :
        first[0][0], first[0][1] = -2, 1
        last[0][-1], last[0][-2] = -2, 1
    elif type == '-N' or type == '-nayman' :
        first[0][0], last[0][-1] = 1, 1
    elif type == '-D' or type == '-dirikhle' :
        first[0][0], first[0][1] = -1, 1
        last[0][-1], last[0][-2] = 1, -1
    elif type == '-M' or type == '-mixed' :
        h, alpha = (beta - alpha) / size, alpha
        first[0][0], first[0][1] = h-alpha, alpha
        last[0][-1], last[0][-2] = h+alpha, -alpha
    else :
        exit('ERROR: incorrect parameters')

    for i in range(size-2) :
        center[i][0+i], center[i][1+i], center[i][2+i] = 1, -2, 1

    first.extend(center)
    first.extend(last)

    return np.asarray(first)

def powerMethod(*args) :
    matrix, size, epsilon = args
    x0, curX = np.zeros((size, 1)), np.zeros((size, 1))
    x0[0][0] = 1
    curOwnValue, prevOwnValue = 0, 0
    eps = epsilon
    counter = 0

    while True :
        counter += 1
        curX = np.dot(matrix, x0)
        curOwnValue = ( np.dot(curX.reshape(1,size),x0) )/( np.dot(x0.reshape(1,size),x0) )
        # print(f"{counter}) New vector: {curX}")
        # print(f"New own value: {curOwnValue[0][0]}")
        if math.fabs(curOwnValue - prevOwnValue) < eps :
            print(f"Iteration {counter}")
            print(f"Max value: {curOwnValue[0][0]} ")
            return None
            # return curOwnValue[0][0]
        else :
            x0 = curX
            prevOwnValue = curOwnValue

def reverseMethod(*args) :
    matrix, size, epsilon = args
    x0, curX = np.zeros((size, 1)), np.zeros((size, 1))
    x0[0][0] = 1
    curOwnValue, prevOwnValue = 0, 0
    eps = epsilon
    reverseMatrix = linalg.inv(matrix)
    print(reverseMatrix)
    counter = 0

    while True :
        counter += 1
        curX = np.dot(reverseMatrix, x0)
        curOwnValue = ( np.dot(x0.reshape(1,size),x0) ) / ( np.dot(curX.reshape(1,size),x0) )
        # print(f"{counter}) New vector: {curX}")
        # print(f"New own value: {curOwnValue[0][0]}")
        if math.fabs(curOwnValue - prevOwnValue) < eps :
            print(f"Iteration {counter}")
            print(f"Min value: {curOwnValue[0][0]} ")
            return None
            # return curOwnValue[0][0]
        else :
            x0 = curX
            prevOwnValue = curOwnValue

if __name__ == "__main__" :
    start = time.time()
    scriptName, typeOfInitialConditions, n, alpha, beta, eps = argv

    matrix = makeMatrix(typeOfInitialConditions,int(n),int(alpha),int(beta))
    print( matrix )
    powerMethod(matrix, int(n), float(eps))
    reverseMethod(matrix, int(n), float(eps))

    end = time.time()
    print(f"Execution time: {end - start}(seconds)")
