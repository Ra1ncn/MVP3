"""
Path Planning Using MRPP solver.
"""
import utils
import math
import sys
import os
import munkres
import os.path, subprocess
from subprocess import STDOUT,PIPE
from random import shuffle

def GetDist(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def generateGrid(wb, bound):
    newLine = []
    stepSize = (bound.height) / 5.0 * (2 / math.sqrt(3))
    xSize = 6
    ySize = 6
    newLine.append((bound.l + stepSize / 2, bound.u))    
    while True:
        if len(newLine) % 2 != len(utils.gridCopy) % 2:
            stepSpace = 1
        else:
            stepSpace = 2
        newLine.append((newLine[-1][0] + stepSize * stepSpace, newLine[0][1]))
        if len(newLine) > xSize:
            newLine.pop(-1)
            utils.gridCopy.append(newLine)
            nextLine = []
            if len(utils.gridCopy) % 2 == 1:
                nextLineMove = -1
            else:
                nextLineMove = 1
            nextLine.append((newLine[0][0] + nextLineMove * stepSize / 2, newLine[0][1] + math.sqrt(3) * stepSize / 2))
            if len(utils.gridCopy) == ySize:
                break
            newLine = nextLine

def assignStart(locs):
    startVertex = []
    # Get the start vertices 
    for x, y in locs:
        closestVertex = -1
        shortestDistace = 10000.0
        # Get the closest vertices
        for i in range(len(utils.gridCopy[0])):
            for j in range(len(utils.gridCopy)):
                thisVertexDist = GetDist(x, y, utils.gridCopy[i][j][0], utils.gridCopy[i][j][1])
                if thisVertexDist < shortestDistace:
                    # If it's already occupied by another car, don't use it
                    if i * 6 + j in startVertex:
                        continue
                    closestVertex = i * 6 + j
                    shortestDistace = thisVertexDist
        startVertex.append(closestVertex)
    return startVertex

def execute_java(startVertex, goalVertex):
    """ Run Path Planning Algorithm, Return path """
    cmd = ['java','projects.multipath.ILP.Main', '111', str(len(utils.gridCopy[0])), str(len(utils.gridCopy))]
    for i in range(9):
        cmd.append(str(startVertex[i]))
        cmd.append(str(goalVertex[i]))
    print(cmd)
    proc = subprocess.Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT)
    stdout, stderr = proc.communicate()
    return stdout

def extractPath(stdout):
    lines = stdout.splitlines()
    pointList = []
    for i in range(len(lines)):
        lines[i] = str(lines[i])
        if len(lines[i]) == 0:
            continue
        if lines[i][0] != 'A':
            continue
        pointList.append([])
        for j in range(len(lines[i])):
            if lines[i][j] == "(":
                pointList[-1].append(utils.gridCopy[int(lines[i][j + 3])][int(lines[i][j + 1])])
    return pointList

def linear_scaling(locs):
    centerx = 640
    centery = 360
    scale_factor = 4 / math.sqrt(3)
    m = munkres.Munkres()
    matrix = [[0 for i in range(9)] for j in range(9)]
    sgs = [(centerx, centery), (centerx + 100, centery), (centerx - 100, centery), (centerx, centery + 100), (centerx, centery - 100), (centerx + 100, centery - 100), (centerx + 100, centery + 100), (centerx - 100, centery - 100), (centerx - 100, centery + 100)]
    for i in range(9):
        for j in range(9):
            matrix[i][j] = GetDist(locs[i][0], locs[i][1], sgs[j][0], sgs[j][1])
    indexes = m.compute(matrix)
    paths0 = [list() for i in range(9)]
    for i in range(9):
        paths0[indexes[i][0]].append(sgs[indexes[i][1]])
        paths0[indexes[i][0]].append(((paths0[indexes[i][0]][0][0] - centerx) * scale_factor + centerx, (paths0[indexes[i][0]][0][1] - centery) * scale_factor + centery))
    new_locs = [0 for i in range(9)]
    for i in range(9):
        new_locs[i] = paths0[i][1]
    startVertex = assignStart(new_locs)
    order = [4, 7, 5, 2, 0, 1, 3, 6, 8]
    goalVertex = [sgs[order[i]] for i in range(9)]
    goalVertex = assignStart(goalVertex)
    stdout = execute_java(startVertex, goalVertex)
    print(stdout)
    paths = extractPath(stdout)    
    maxLen = 0    
    for i in range(9):
        if len(paths[i]) > maxLen:
            maxLen = len(paths[i])
    for i in range(maxLen):
        for j in range(9):
            if len(paths[j]) <= maxLen:
                paths0[j].append(paths[j][i])
            else:
                paths0[j].append(paths0[j][-1])
    for i in range(9):
        paths0[i].append(((sgs[order[i]][0] - centerx) * scale_factor + centerx, (sgs[order[i]][1] - centery) * scale_factor + centery))
        paths0[i].append(sgs[order[i]])
        paths0[i].append((paths0[i][-1][0], paths0[i][-1][1] - 70))
    return paths0

def GetPath(locs, goals, wb, bound):
    # paths = [list() for x in range(num)]
    if len(utils.gridCopy) == 0: 
        generateGrid(wb, bound)
    # startVertex = assignStart(locs)
    # stdout = execute_java(startVertex)
    # paths = extractPath(stdout)    
    return linear_scaling(locs)