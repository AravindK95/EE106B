#!/usr/bin/env python
import sys
import rospkg
import numpy as np

PROJECT_PATH = rospkg.RosPack().get_path('grasper_plan')
sys.path.append(PROJECT_PATH+'/src')

import obj_file
import transformations
import fc

GRASP_DATA_FILENAME = PROJECT_PATH+'/data/pawn_grasps.csv'
SORTED_DATA_FILENAME1 = PROJECT_PATH+'/data/pawn_sortedminmax.csv'
SORTED_DATA_FILENAME2 = PROJECT_PATH+'/data/pawn_sortedvote.csv'


ranking = []
presort = []
vdistsort = []
hdistsort = []
cogdistsort = []
apartsort = []

def topnum(grasps,maxmin,key,num):
    topnum = []
    clonegrasps = grasps
    if maxmin == 'x':
        for i in range(0,num):
            topnum.append(max(grasps,key=lambda x:x[0][key]))
            clonegrasps.remove(topnum[i])

    if maxmin == 'n':
        for i in range(0,num):
            topnum.append(min(grasps,key=lambda x:x[0][key]))
            clonegrasps.remove(topnum[i])

    return topnum

def combine_vote(hsort,vsort,cogsort,asort,presort):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    rank = presort
    counter = 1

    for k in range(0,len(presort)):
        hsort[k][0] = counter
        vsort[k][0] = counter
        cogsort[k][0] = counter
        asort[k][0] = counter

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
        item = item[1:2]
        for graspsh in hsort:
            graspsh = graspsh[1:2]
            if np.array_equal(graspsh,item):
                item[0][0] = graspsh[0] 
        for graspsv in vsort:
            graspsv = graspsv[1:2]
            if np.array_equal(graspsv,item):
                item[0][1] = graspsv[0] 
        for graspsc in cogsort:         
            graspsc = graspsc[1:2]
            if np.array_equal(graspsc,item):
                item[0][2] = graspsc[0]
        for graspsa in asort: 
            graspsa = graspsa[1:2]
            if np.array_equal(graspsa,item):
                item[0][3] = graspsa[0]


    for item in rank:
        item[0] = item[0][0] + 2*item[0][1] + 2*item[0][2] + 2*item[0][3]
    
    return rank.sort(key=lambda x: x[0])

def combine_minmax(grasps):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    topfive = []

    # change this later to be a function of the size of the input grasps
    topfive = topnum(topnum(topnum(topnum(grasps,'x',1,20),'n',3,15),'n',2,10),'x',0,5)
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
                presort.append([[0,0,0,0],grasp[i],grasp[j]])

    #rank by horizontal distance
    for grasps in presort:
        hdistsort.append([grasps[1][3]+grasps[2][3],grasps[1],grasps[2]])
        grasps[0][0] = grasps[1][3]+grasps[2][3]
  
    #add distance apart vertical
    for grasps in presort:
        zg1 = grasps[1][0][11]
        zg2 = grasps[2][0][11]
        vdistsort.append([abs(zg1-zg2),grasps[1],grasps[2]])
        grasps[0][1] = abs(zg1-zg2)

    #add dist from cog
    for grasps in presort:
        zg1 = grasps[1][0][11]
        zg2 = grasps[2][0][11]
        cogdistsort.append([abs(zg1-cog)+abs(zg2-cog),grasps[1],grasps[2]])
        grasps[0][2] = abs(zg1-cog)+abs(zg2-cog)

    #add same saide?
    for grasps in presort:
        zvect1 = grasps[1][0][8:10]
        zvect1n = zvect1/np.linalg.norm(zvect1)
        zvect2 = grasps[2][0][8:10]
        zvect2n = zvect2/np.linalg.norm(zvect2)
        apartsort.append([np.dot(zvect1n,zvect2n),grasps[1],grasps[2]])
        grasps[0][3] = np.dot(zvect1n,zvect2n)


    hdistsort.sort(key=lambda x: x[0])
    vdistsort.sort(key=lambda x: x[0])
    cogdistsort.sort(key=lambda x: x[0])
    apartsort.sort(key=lambda x: x[0])

    sortedminmax = combine_minmax(presort)
    # sortedvote = combine_vote(hdistsort,vdistsort,cogdistsort,apartsort,presort)

    # return sortedminmax, sortedvote
    return sortedminmax


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

    sortedgrasps = grasp_eval(grasps)
    # sortedgrasps, sortedvotegrasps = grasp_eval(grasps)


    out_f = open(SORTED_DATA_FILENAME1, 'w')
    out_f.write('c1; c2\n')
    for g in sortedgrasps:
        c1, c2 = g[1][0], g[2][0]
        out_f.write(str(c1) + '; ')
        out_f.write(str(c2) + '; ')
        out_f.write('\n')

    # out_v = open(SORTED_DATA_FILENAME2, 'w')
    # out_v.write('c1; c2\n')
    # for g in sortedvotegrasps:
    #     c1, c2 = g[1][0], g[2][0]
    #     out_v.write(str(c1) + '; ')
    #     out_v.write(str(c2) + '; ')
    #     out_v.write('\n')

    out_f.close()
    # out_v.close()