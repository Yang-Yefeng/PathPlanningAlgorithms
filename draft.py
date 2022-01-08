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

# string = "['octagon', [3.899, 9.353, 0.787], [[ 3.575, 10.071], [3.163, 9.632], [3.181, 9.03 ], [3.62 , 8.618], [4.222, 8.636], [4.634, 9.075], [4.616, 9.677], [ 4.177, 10.089]]]"
# string = string.replace(' ', '').replace("'", '').replace('[', '').replace(']', '')
# print(string)
# print(string.split(','))
# a = []
# b =[1]
# a.append(b)
# print(a)
# b = [2]
# a.append(b)
# print(a)
print(10//4)