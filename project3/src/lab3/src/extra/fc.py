"""
Homework 3 Problem 4
Name: Aravind Kumaraguru
"""
from __future__ import division
import numpy as np

def force_closure(contacts, normals, num_facets, mu, gamma):
    """
    Calculates force by determining if the vector between both
    contact points are inside both friction cones.
    """
    v1 = contacts[:,1] - contacts[:,0]
    v2 = -v1
    n1 = normals[:,0]
    n2 = normals[:,1]

    theta1 = np.arccos(np.dot(v1, n1)/(np.linalg.norm(v1)*np.linalg.norm(n1)))
    theta2 = np.arccos(np.dot(v2, n2)/(np.linalg.norm(v2)*np.linalg.norm(n2)))

    if np.arctan(mu) > theta1 and np.arctan(mu) > theta2    :
       return 1
    return 0
