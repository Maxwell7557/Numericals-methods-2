from sys import argv
import math
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

def show_chart(*coords) :
    flag = True
    x,y,z,w = [],[],[],[]
    if len(coords) == 3 :
        x, y, z = coords
        flag = True
    elif len(coords) == 4 :
        x, y, z, w = coords
        flag = False

    figure, axis = plt.subplots(figsize=(10, 10))

    axis.set_xlabel('$t$')
    axis.set_ylabel('$y$')
    axis.minorticks_on()
    axis.grid(which = "major",color='orange',linewidth = 1)
    axis.grid(which = "minor",color='orange',linestyle = ":")
    figure.tight_layout()

    if flag == True :
        axis.plot(x,y,'b',label='u1')
        axis.plot(x,z,'r',label='u2')
    elif flag == False :
        axis.plot(x,y,'b',label='u1')
        axis.plot(x,z,'r',label='u2')
        axis.plot(x,w,'g',label='u3')

    axis.legend()

    plt.show()

def initialConditionsForFirstTask(flag) :
    if flag == True :
        epsIAdditional = 10 ** -3
    elif flag == False :
        epsIAdditional = 10 ** -5
    T = 1
    tayMin = 0.001
    tayMax = 1
    tk = 0
    yk = [0,-0.412]
    prevYk, nextYk = yk, yk
    prevTayk, tayK = tayMin, tayMin
    w = 25
    a = 2.5 + w/40
    return epsIAdditional, T, tayMin, tayMax, tk, yk, prevYk, nextYk, prevTayk, tayK, w, a

def initialConditionsForSecondTask(flag) :
    if flag == True :
        epsIAdditional = 10 ** -3
    elif flag == False :
        epsIAdditional = 10 ** -5
    T = 1
    tayMin = 0.001
    tayMax = 1
    tk = 0
    yk = [1,1,1]
    prevYk, nextYk = yk, yk
    prevTayk, tayK = tayMin, tayMin
    a = 1
    k = 2
    return epsIAdditional,T, tayMin, tayMax, tk, yk, prevYk, nextYk, prevTayk, tayK, k, a

def firstTaskFunc(y,a,t) :
    x = []
    if t == 0 :
        x.append(-y[0]*y[1] + 1)
    elif t != 0 :
        x.append(-y[0]*y[1] + (math.sin(t)) / t)
    x.append(-y[1]**2 + (a*t) / (1+t**2))
    return x

def secondTaskFunc(y,t,a,k) :
    x = []
    x.append( ((k-a)/a)*y[1]*y[2] )
    x.append( ((k+a)/a)*y[0]*y[2] )
    x.append( ((a-k)/a)*y[0]*y[1] )
    return x

# def add(list,val) :
#     list.append(val)
#     return list

def explicitEuler(numberOfTask, flag) :
    epsIAdditional, T, tayMin, tayMax, tk, yk, prevYk, nextYk, prevTayk, tayK, k, a = initialConditionsForFirstTask(flag) if numberOfTask == 1 else initialConditionsForSecondTask(flag)
    yCoords = []
    tCoords = []

    while True :
        vect = firstTaskFunc(yk,a,tk) if numberOfTask == 1 else secondTaskFunc(yk,tk,a,k)
        tayIK = []
        tmpYk = []
        for val in vect :
            tayIK.append( epsIAdditional / math.fabs(val) + epsIAdditional / tayMax )
        tayK = min(tayIK)
        for i in range(len(vect)) :
            yk[i] = yk[i] + tayK*vect[i]
            tmpYk.append(yk[i])
        tk = tk + tayK
        tCoords.append(tk)
        yCoords.append(tmpYk)
        # print(f'tk: {tk}')
        # print(f'yk: {yk}')
        # print(yCoords)
        if tk >= T :
            tCoords.pop()
            yCoords.pop()
            # print(yCoords)
            print(f'tk: {tCoords[-1]}')
            print(f'yk: {yCoords[-1]}')
            return tCoords, yCoords
            # break

def nonExplicitEuler(numberOfTask,flag) :
    epsIAdditional, T, tayMin, tayMax, tk, yk, prevYk, nextYk, prevTayk, tayK, k, a = initialConditionsForFirstTask(flag) if numberOfTask == 1 else initialConditionsForSecondTask(flag)
    tCoords = []
    yCoords = []

    while True :
        nextTk = tk + tayK
        tmp = firstTaskFunc(nextYk,a,nextTk) if numberOfTask == 1 else secondTaskFunc(yk,tk,a,k)
        nextYk = []
        for i in range(len(yk)) :
            # nextYk[i] = yk[i] + tayK*tmp[i]
            nextYk.append(yk[i] + tayK*tmp[i])
        epsK = []
        for i in range(len(yk)) :
            epsK.append( -(tayK / (tayK + prevTayk)) * (nextYk[i] - yk[i] - (tayK/prevTayk)*(yk[i] - prevYk[i]) ) )

        condition = True
        for i in range(len(epsK)) :
            if math.fabs(epsK[i]) <= epsIAdditional :
                condition = False
                break

        if condition == True :
            tayK = tayK / 2
            nextTk = tk
            nextYk = yk
            continue

        nextTaykI = []
        for i in range(len(epsK)) :
            nextTaykI.append(math.sqrt(epsIAdditional / math.fabs(epsK[i])) * tayK)
            # if math.fabs(epsK[i]) > epsIAdditional :
            #     nextTaykI.append(tayK/2)
            # elif 4 < math.fabs(epsK[i]) and math.fabs(epsK[i]) <= epsIAdditional :
            #     nextTaykI.append(tayK)
            # if math.fabs(epsK[i]) <= epsIAdditional/4 :
            #     nextTaykI.append(tayK*2)

        # print(nextTaykI)
        nextTayk = min(nextTaykI)

        if nextTayk > tayMax :
            nextTayk = tayMax

        # print(f'tk+1: {nextTk}')
        # print(f'yk+1: {nextYk}')
        tCoords.append(nextTk)
        yCoords.append(nextYk)

        prevYk = yk
        yk = nextYk
        prevTayk = tayK
        tayK = nextTayk
        tk = nextTk

        if tk >= T :
            tCoords.pop()
            yCoords.pop()
            print(f'tk+1: {tCoords[-1]}')
            print(f'yk+1: {yCoords[-1]}')
            return tCoords,yCoords
            # break

def spline(x,y,curX) :
    tck = interpolate.splrep(x,y)
    return interpolate.splev(curX,tck)

if __name__ == "__main__" :
    scriptName, method, numberOfTask, eps = argv
    n = int(numberOfTask)
    if eps == "First" :
        choiceOfEps = True
    elif eps == "Second" :
        choiceOfEps = False

    tCoords, yCoords = [], []
    if method == 'explicit' :
        tCoords,yCoords = explicitEuler(n, choiceOfEps)
        # print(yCoords)
    elif method == 'nonexplicit' :
        tCoords,yCoords = nonExplicitEuler(n, choiceOfEps)

    # print(yCoords)
    # numOfElements = len(tCoords)+1
    x = np.linspace(0, 1, num=1000, endpoint=True)
    if n == 1 :
        firstY = [yCoords[i][0] for i in range(len(yCoords))]
        secondY = [yCoords[i][1] for i in range(len(yCoords))]
        y = [spline(tCoords,firstY,t) for t in x]
        z = [spline(tCoords,secondY,t) for t in x]
        show_chart(x,y,z)
    else :
        firstY = [yCoords[i][0] for i in range(len(yCoords))]
        secondY = [yCoords[i][1] for i in range(len(yCoords))]
        thirdY = [yCoords[i][2] for i in range(len(yCoords))]
        y = [spline(tCoords,firstY,t) for t in x]
        z = [spline(tCoords,secondY,t) for t in x]
        w = [spline(tCoords,thirdY,t) for t in x]
        show_chart(x,y,z,w)
