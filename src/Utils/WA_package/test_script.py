import numpy as np 
import sympy as sp
num_letters = 6
num_states  = 6

initial_array = sp.Matrix([[1, 0, 0, 0, 0, 0]])
final_array   = sp.Matrix.ones((6,1))
transition_dictionary = {}
keys = {}
symbols = {'pickup key', 'opened door', 
            'dropped key', 'closed door', 
            'pickup box', 'movement'}
for index in np.arange(6):
    transition_dictionary 
