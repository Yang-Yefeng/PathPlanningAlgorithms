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

# t = sp.symbols('t')
# x = sp.symbols('x')
# if None:
#     print('sss')
# '''贝塞尔曲线'''

a = [[2.0, 3.0, 4.0], [2.0, 3.0, 4.0], [2.0, 3.0, 4.0]]

print(a[:][0])