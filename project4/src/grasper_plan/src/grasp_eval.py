import sys
import rospkg
import numpy as np
import obj_file
import transformations
import fc

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
	for grasps in hsort
		counter = 1
		grasps[0][0] = counter
		counter = counter + 1
	for grasps in vsort
		counter = 1
		grasps[0][0] = counter
		counter = counter + 1
	for grasps in cogsort
		counter = 1
		grasps[0][0] = counter
		counter = counter + 1

def combine_minmax(grasps):
	#1st heursitic is combination of dist between c1 and c2 of each grasp
	#2nd heursitic is z distance between them
	#3rd heuristic is how close it is cog
	topfive = []

	for i in range(0,5)
		topfive[i] = min(max(grasps,key=lambda(x):x[0][0]), max(grasps,key=lambda(x):x[0][1]),key=lambda(x): x[0][2])
		grasps.remove(topfive[i])

def grasp_eval(grasp, cog):
	#grasp --> list of tuples
	# - (rbt,cp1,cp2,dist)

	#generate grasp pairs
	#presort,vdistsort,hdistsort,cogdistsort
	# - [value,grasp1,grasp2]
	for i in range(len(grasp)):
        for j in range(i+1, len(grasp)):
            presort.append([[0,0,0],i,j])

    #rank by horizontal distance
   	for grasps in presort
   		hdistsort.append([grasps[1][3]+grasps[2][3],grasps[1],grasps[2]])
   		presort[0][0] = grasps[1][3]+grasps[2][3]
  
  	#add distance apart vertical
  	for grasps in presort
  		zg1 = grasps[1][0][2][3]
  		zg2 = grasps[2][0][2][3]
  		vdistsort.append([abs(zg1-zg2),grasps[1],grasps[2]])
  		presort[0][1] = abs(zg1-zg2)

  	#add dist from cog
  	for grasps in presort
  		zg1 = grasps[1][0][2][3]
  		zg2 = grasps[2][0][2][3]
  		cogdistsort.append([abs(zg1-cog)+abs(zg2-cog),grasps[1],grasps[2]])
  		presort[0][2] = abs(zg1-cog)+abs(zg2-cog)




  	hdistsort.sort()
  	vdistsort.sort()
  	cogdistsort.sort()

  	sortedminmax = combine_minmax(presort)
  	sortedminmax = combine_vote(hdistsort,vdistsort,cogdistsort)


