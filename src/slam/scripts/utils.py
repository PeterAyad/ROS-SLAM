import numpy as np

def log_2_probability(lg):
    return 1 - (1/(1+np.exp(lg)))

def probability_2_log(pb):
    return np.log(pb/(1-pb))

def is_inside_mat (y, x , mat):
    return y>=0 and x>=0 and y < mat.shape[0] and x < mat.shape[1]