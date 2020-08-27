import numpy as np
def to_skew(x):
	X = np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]])
	return X