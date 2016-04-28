import numpy as np

def force_closure(contact,normals,num_facets,mu,gamma):
	c1 = contact[:,0]
	c2 = contact[:,1]
	f1 = normals[:,0]
	f2 = normals[:,0]
	v = (c2-c2).astype(float)
	theta1 = np.arccos((f1.dot(v))/(np.linalg.norm(f1)*np.linalg.norm(v)))
	theta2 = np.arccos((f2.dot(-v))/(np.linalg.norm(f2)*np.linalg.norm(-v)))

	if theta1<np.arctan(mu) and theta2<np.arctan(mu):
		return 1
	return 0
