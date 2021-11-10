import time
import numpy as np
from Samplebased.Base import KDTree
import random
import time
import math

# pt = [random.uniform(0.0, 10.0) for _ in range(2)]
# # print(pt)
# tree = KDTree.create([pt])
# # kdtree.visualize(tree)
#
# t1 = time.time()
# for _ in range(10000):
#     pt = [random.uniform(0.0, 10.0) for _ in range(2)]
#     node, dis = tree.search_nn(pt)
#     tree.add(pt)
# t2 = time.time()
# print('10000次', t2 - t1)
# kdtree.visualize(tree)
# def tree_generate_with_start_end_point(start, end):
#     [dx, dy] = [end[i] - start[i] for i in range(2)]
#     scale = math.sqrt(50) / 5 / math.sqrt(dx ** 2 + dy ** 2)
#     x = start[0] + dx * scale
#     y = start[1] + dy * scale
#     return [x, y]
#
# start = [0, 0]
# end = [5, 5]
# dis = tree_generate_with_start_end_point(start, end)
# print(dis)
a = (0, 0)
b = a
print(a)
a = (1, 3)
print(b)