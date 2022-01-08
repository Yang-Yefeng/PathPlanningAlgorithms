from Map.Continuous.obstacle import obstacle
from Map.Continuous.samplingmap import samplingmap
from Samplebased.Algorithms.RRT_Based.RRT import RRT
import cv2 as cv
import math
import numpy as np
import scipy.linalg as linalg
from scipy import integrate
import sympy as sp
import cvxopt as cp
import random


# t = sp.symbols('t')
# x = sp.symbols('x')
# if None:
#     print('sss')
# '''贝塞尔曲线'''

# a = [[2.0, 3.0, 4.0], [2.0, 3.0, 4.0], [2.0, 3.0, 4.0]]
#
# print(a[:][0])

# print(random.sample([3, 4, 5, 6, 7, 8], 1)[0])
# a = [[random.choice([0, 1]) for _ in range(10)] for _ in range(5)]
# b = str(a).replace(', ', '').replace('[', '').replace(']', '')
# print(b)
a = [[random.uniform(0,1) for _ in range(10)] for _ in range(10)]
print(np.around(a,2))