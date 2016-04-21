'''
Encapsulates mesh for grasping operations
Author: Jeff Mahler
'''
import copy
import logging
import numpy as np
import os
import sys

class Mesh3D(object):
    """
    A Mesh is a three-dimensional shape representation
    
    Params:
       vertices:  (list of 3-lists of float)
       triangles: (list of 3-lists of ints)
       normals:   (list of 3-lists of float)
       metadata:  (dictionary) data like category, etc
       pose:      (tfx pose)
       scale:     (float)
       component: (int)
    """
    def __init__(self, vertices, triangles, normals):
        self.vertices_ = vertices
        self.triangles_ = triangles
        self.normals_ = normals

    @property
    def vertices(self):
        return self.vertices_

    @property
    def triangles(self):
        return self.triangles_

    @property
    def normals(self):
        return self.normals_

    def compute_normals(self):
        """ Get vertex normals from triangles"""
        vertex_array = np.array(self.vertices_)
        tri_array = np.array(self.triangles_)
        self.normals_ = []
        for i in range(len(self.vertices_)):
            inds = np.where(tri_array == i)
            first_tri = tri_array[inds[0][0],:]
            t = vertex_array[first_tri, :]
            v0 = t[1,:] - t[0,:] 
            v1 = t[2,:] - t[0,:] 
            v0 = v0 / np.linalg.norm(v0)
            v1 = v1 / np.linalg.norm(v1)
            n = np.cross(v0, v1)
            n = n / np.linalg.norm(n)
            self.normals_.append(n.tolist())

    def tri_centers(self):
        """ Return a list of the triangle centers as 3D points """
        centers = []
        for tri in self.triangles_:
            v = np.array([self.vertices_[tri[0]], self.vertices_[tri[1]], self.vertices_[tri[2]]])
            center = np.mean(v, axis=0)
            centers.append(center.tolist())
        return centers

    def tri_normals(self):
        """ Return a list of the triangle normals """
        vertex_array = np.array(self.vertices_)
        normals = []
        for tri in self.triangles_:
            v0 = vertex_array[tri[0],:]
            v1 = vertex_array[tri[1],:]
            v2 = vertex_array[tri[2],:]
            n = np.cross(v1 - v0, v2 - v0)
            n = n / np.linalg.norm(n)
            normals.append(n.tolist())

    def center_vertices_avg(self):
        """ Re-center vertices at average vertex """
        vertex_array = np.array(self.vertices_)
        centroid = np.mean(vertex_array, axis = 0)
        vertex_array_cent = vertex_array - centroid
        self.vertices_ = vertex_array_cent.tolist()

    def center_vertices_bb(self):
        """ Re-center vertices at center of bounding box """
        vertex_array = np.array(self.vertices_)
        min_vertex = np.min(vertex_array, axis = 0)
        max_vertex = np.max(vertex_array, axis = 0)
        centroid = (max_vertex + min_vertex) / 2
        vertex_array_cent = vertex_array - centroid
        self.vertices_ = vertex_array_cent.tolist()

    def rescale(self, scale_factor):
        """
        Rescales the vertex coordinates by scale factor
        Params:
           scale_factor: (float) the scale factor
        Returns:
           Nothing. Modified the mesh in place (for now)
        """
        vertex_array = np.array(self.vertices_)
        scaled_vertices = scale_factor * vertex_array
        self.vertices_ = scaled_vertices.tolist()
