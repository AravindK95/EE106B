#!/usr/bin/env python
import sys
import rospkg
import numpy as np

PROJECT_PATH = rospkg.RosPack().get_path('grasper_plan')
sys.path.append(PROJECT_PATH+'/src')

import obj_file
import transformations
import fc

GRASP_DATA_FILENAME = PROJECT_PATH+'/data/grasps.csv'

ranking = []
presort = []
vdistsort = []
hdistsort = []
cogdistsort = []

def combine_minmax(hsort,vsort,cogsort):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    rank = []
    for grasps in hsort:
        counter = 1
        grasps[0][0] = counter
        counter = counter + 1
    for grasps in vsort:
        counter = 1
        grasps[0][0] = counter
        counter = counter + 1
    for grasps in cogsort:
        counter = 1
        grasps[0][0] = counter
        counter = counter + 1

def combine_minmax(grasps):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    topfive = []

    for i in range(0,5):
        topfive[i] = max(max(grasps,key=lambda x:x[0][0]), min(grasps,key=lambda x:x[0][2]),key=lambda x: x[0][1])
        grasps.remove(topfive[i])

def grasp_eval(grasp, cog=0):
    #grasp --> list of tuples
    # - (rbt,cp1,cp2,dist)

    #generate grasp pairs
    #presort,vdistsort,hdistsort,cogdistsort
    # - [value,grasp1,grasp2]
    grasp = np.array(grasp)
    for i in range(grasp.shape[0]):
        for j in range(i+1, grasp.shape[0]):
            presort.append([[0,0,0],grasp[i],grasp[j]])

    #rank by horizontal distance
    for grasps in presort:
        hdistsort.append([grasps[1][3]+grasps[2][3],grasps[1],grasps[2]])
        presort[0][0] = grasps[1][3]+grasps[2][3]
  
    # #add distance apart vertical
    # for grasps in presort:
    #     zg1 = grasps[1][0][2,3]
    #     zg2 = grasps[2][0][2,3]
    #     vdistsort.append([abs(zg1-zg2),grasps[1],grasps[2]])
    #     presort[0][1] = abs(zg1-zg2)

    # #add dist from cog
    # for grasps in presort:
    #     zg1 = grasps[1][0][11]
    #     zg2 = grasps[2][0][11
    #     cogdistsort.append([abs(zg1-cog)+abs(zg2-cog),grasps[1],grasps[2]])
    #     presort[0][2] = abs(zg1-cog)+abs(zg2-cog)


    hdistsort.sort(key=lambda x: x[0])
    print hdistsort[0:4]
    # vdistsort.sort()
    # cogdistsort.sort()

    # sortedminmax = combine_minmax(presort)
    # sortedminmax = combine_vote(hdistsort,vdistsort,cogdistsort)


if __name__ == '__main__':
    in_f = open(GRASP_DATA_FILENAME, 'r')
    data = in_f.read()
    data = data.split('\n')
    data.pop(0)
    data.pop()

    grasps = list()
    for row in data:
        grasps.append(tuple([eval(e) for e in row.split(';')]))
    in_f.close()

    print grasp_eval(grasps)
