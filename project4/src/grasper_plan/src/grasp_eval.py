#!/usr/bin/env python
import sys
import rospkg
import numpy as np

PROJECT_PATH = rospkg.RosPack().get_path('grasper_plan')
sys.path.append(PROJECT_PATH+'/src')

import obj_file
import transformations
import fc

GRASP_DATA_FILENAME = PROJECT_PATH+'/data/grasps/pawn_grasps.csv'
SORTED_DATA_FILENAME1 = PROJECT_PATH+'/data/sorted/pawn_sortedminmax.csv'
SORTED_DATA_FILENAME2 = PROJECT_PATH+'/data/sorted/pawn_sortedvote.csv'

ranking = []
presort = []
vdistsort = []
hdistsort = []
cogdistsort = []

def combine_vote(hsort,vsort,cogsort,presort):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    rank = presort
    counter = 1

    for k in range(0,len(presort)):
        hsort[k][0] = counter
        vsort[k][0] = counter
        cogsort[k][0] = counter
        counter += 1

    # for graspsh in hsort:
    #     graspsh[0] = counter
    #     counter = counter + 1
    # counter = 1
    # for graspsv in vsort:
    #     counter = 1
    #     graspsv[0] = counter
    #     counter = counter + 1
    # counter = 1
    # for graspsc in cogsort:
    #     counter = 1
    #     graspsc[0] = counter
    #     counter = counter + 1

    for item in rank:
        for graspsh in hsort:
            if graspsh == item:
                item[0][0] = graspsh[0] 
        for graspsv in vsort:
            if graspsv == item:
                item[0][1] = graspsv[0] 
        for graspsc in cogsort:         
            if graspsc == item:
                item[0][2] = graspsc[0]

    for item in rank:
        item[0] = item[0][0] + item[0][1] + item[0][2]
    
    rank.sort(key=lambda x: x[0])
    return rank

def combine_minmax(grasps):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    topfive = []

    for i in range(0,5):
        # topfive.append(max(grasps, key=lambda x: x[0][1]))
        topfive.append(max(max(grasps,key=lambda x:x[0][1]), min(grasps,key=lambda x:x[0][2]),key=lambda x: x[0][0]))
        # topfive.append(max(max(grasps,key=lambda x:x[0][0]),key=lambda x: x[0][1]))
        grasps.remove(topfive[i])

    return topfive

def grasp_eval(grasp, cog=0.075):
    #grasp --> list of tuples
    # - (rbt,cp1,cp2,dist)

    #generate grasp pairs
    #presort,vdistsort,hdistsort,cogdistsort
    # - [value,grasp1,grasp2]
    grasp = np.array(grasp)
    for i in range(grasp.shape[0]):
        for j in range(i+1, grasp.shape[0]):
            zg1 = grasp[i][0][11]
            zg2 = grasp[j][0][11]
            if np.abs(zg1 - zg2) > 0.05 and np.abs(zg1 - zg2)<.25:
                presort.append([[0,0,0],grasp[i],grasp[j]])
    print presort

    #rank by horizontal distance
    for grasps in presort:
        hdistsort.append([grasps[1][3]+grasps[2][3],grasps[1],grasps[2]])
        grasps[0][0] = grasps[1][3]+grasps[2][3]
  
    #add distance apart vertical
    for grasps in presort:
        zg1 = grasp[1][0][11]
        zg2 = grasp[2][0][11]
        vdistsort.append([abs(zg1-zg2),grasps[1],grasps[2]])
        grasps[0][1] = abs(zg1-zg2)

    #add dist from cog
    for grasps in presort:
        zg1 = grasps[1][0][11]
        zg2 = grasps[2][0][11]
        cogdistsort.append([abs(zg1-cog)+abs(zg2-cog),grasps[1],grasps[2]])
        grasps[0][2] = abs(zg1-cog)+abs(zg2-cog)


    hdistsort.sort(key=lambda x: x[0])
    vdistsort.sort(key=lambda x: x[0])
    cogdistsort.sort(key=lambda x: x[0])

    sortedminmax = combine_minmax(presort)
    sortedvote = combine_vote(hdistsort,vdistsort,cogdistsort,presort)

    return sortedminmax, sortedvote


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

    print grasps
    sortedgrasps, sortedvotegrasps = grasp_eval(grasps)

    out_f = open(SORTED_DATA_FILENAME1, 'w')
    out_f.write('c1; c2\n')
    for g in sortedgrasps:
        c1, c2 = g[1][0], g[2][0]
        out_f.write(str(c1) + '; ')
        out_f.write(str(c2) + '; ')
        out_f.write('\n')

    out_v = open(SORTED_DATA_FILENAME2, 'w')
    out_v.write('c1; c2\n')
    for g in sortedvotegrasps:
        c1, c2 = g[1][0], g[2][0]
        out_v.write(str(c1) + '; ')
        out_v.write(str(c2) + '; ')
        out_v.write('\n')

    out_f.close()
    out_v.close()