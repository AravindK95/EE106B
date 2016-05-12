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

def combine_vote(presort):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    #4th heuristic is how close to same side.

    rank = presort
    rank.sort(key=lambda x: x[0][0],reverse=True)
    for idx in range(0,len(rank)):
        rank[idx][0][0] = idx+1
    rank.sort(key=lambda x: x[0][1],reverse=True)
    for idx in range(0,len(rank)):
        rank[idx][0][1] = idx+1
    rank.sort(key=lambda x: x[0][2],reverse=False)
    for idx in range(0,len(rank)):
        rank[idx][0][2] = idx+1
    rank.sort(key=lambda x: x[0][3],reverse=False)
    for idx in range(0,len(rank)):
        rank[idx][0][3] = idx+1  

    for item in rank:
        item[0][4] = item[0][0] + 2*item[0][1] + 2*item[0][2] + 2*item[0][3]
    
    rank.sort(key=lambda x: x[0][4])

    return rank

def combine_minmax(grasps):
    #1st heursitic is combination of dist between c1 and c2 of each grasp
    #2nd heursitic is z distance between them
    #3rd heuristic is how close it is cog
    #4th heuristic is how close to same side.
    topfive = []

    # change this later to be a function of the size of the input grasps
    topfive = topnum(topnum(topnum(topnum(grasps,'x',1,20),'n',3,15),'n',2,10),'x',0,5)
    return topfive

def grasp_eval(grasp, cog=0.075):
    #grasp --> list of tuples
    # - (rbt,cp1,cp2,dist)

    #generate grasp pairs
    #presort,vdistsort,hdistsort,cogdistsort
    # - [[h1,h2,h3,h4,idx],grasp1,grasp2]
    grasp = np.array(grasp)
    for i in range(grasp.shape[0]):
        for j in range(i+1, grasp.shape[0]):
            zg1 = grasp[i][0][11]
            zg2 = grasp[j][0][11]
            if np.abs(zg1 - zg2) > 0.05 and np.abs(zg1 - zg2)<.25:
                presort.append([[0,0,0,0,0],grasp[i],grasp[j]])

    #rank by horizontal distance
    for grasps in presort:
        grasps[0][0] = grasps[1][3]+grasps[2][3]
  
    #add distance apart vertical
    for grasps in presort:
        zg1 = grasps[1][0][11]
        zg2 = grasps[2][0][11]
        grasps[0][1] = abs(zg1-zg2)

    #add dist from cog
    for grasps in presort:
        zg1 = grasps[1][0][11]
        zg2 = grasps[2][0][11]
        grasps[0][2] = abs(zg1-cog)+abs(zg2-cog)

    #add same saide?
    for grasps in presort:
        zvect1 = grasps[1][0][8:10]
        zvect1n = zvect1/np.linalg.norm(zvect1)
        zvect2 = grasps[2][0][8:10]
        zvect2n = zvect2/np.linalg.norm(zvect2)
        grasps[0][3] = np.dot(zvect1n,zvect2n)

    sortedminmax = combine_minmax(presort)
    sortedvote = combine_vote(presort)

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