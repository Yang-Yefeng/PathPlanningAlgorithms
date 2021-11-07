import time

import numpy as np

class classA:
    def __init__(self, a: int, b: int, c: int):
        self.a = a
        self.b = b
        self.c = c

    def add(self):
        return self.a + self.b + self.c

class classB(classA):
    def __init__(self, a: int, b: int, c: int):
        super(classB, self).__init__(a, b, c)



# b = B()

if __name__ == '__main__':
    # a = classA(a=1, b=2, c=5)
    # print(a.add())
    #
    # b = classB(a=1, b=2, c=5)
    # print(b.add())
    # a = np.array([1,2,3])
    # a = [1,2,3]
    # b=a.copy()
    # print(a, b)
    # a[0] = 2
    # print(a, b)
    # def getffromopen(elem: np.ndarray)->float:
    #     return elem[3]
    # open_list = np.atleast_2d()
    # open_list.append([[1, 2], 1, 2, 9])
    # open_list.append([[2, 3], 1, 2, 6])
    # open_list.append([[3, 4], 1, 2, 7])
    # print(open_list)
    #
    # node = [i[0] for i in open_list].index([1, 2])
    # print(node)
    # print(open_list[0])

    # open_list2 = np.atleast_2d()
    # open_list2.append([[1, 2], 1, 2, 2])
    # open_list2.append([[1, 2], 1, 2, 3])
    # open_list2.append([[1, 2], 1, 2, 1])
    # open_list2.append([[1, 2], 1, 2, 0.85])
    # print(open_list2)
    #
    # open_list.append(open_list2.copy())
    # print(open_list)
    # open_list2.clear()
    # print(open_list)
    # open_list.sort(key=getffromopen)
    # node = open_list.pop(0)
    # print(node[0][0])

    # while len(open_list) > 0:
    #     open_list.pop(0)
    #     print(open_list)
    # print(np.inf)
    # a = [1, 2, 3]
    # b = np.array([1, 2, 3])
    # print(a==list(b))
    # print(tuple(b))
    # waitinglist = []
    # for i in [-1, 0, 1]:
    #     for j in [-1, 0, 1]:
    #         waitinglist.append([i, j])
    # print(waitinglist)
    # waitinglist.remove([0,0])
    # print(waitinglist)
    # b = [1, 2]
    # a = []
    # a.append([1,2])
    # a.append([1,2])
    # for i in a:
    #     print(i)
    # pass
    # x = np.array([5, 5, 5, 5, 5, 5, 5, 5, 5])
    # y = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])
    # c = np.array([x, y]).T
    # print(c)
    # print(d)
    # waitinglist = []
    # t1 = time.time()
    # for m in [-1, 0, 1]:
    #     for n in [-1, 0, 1]:
    #         waitinglist.append([m, n])
    # t2 = time.time()
    # print(t2 - t1)
    #
    # t3 = time.time()
    # waitinglist2 = [[i, j] for j in [-1, 0, 1] for i in [-1, 0, 1]]
    # t4 = time.time()
    # print(t4 - t3)
    a = [1, 1]
    b = [1, 2]
    t1 = time.time()
    c = [a[0] + b[0], a[1] + b[1]]
    t2 = time.time()
    print(t2 - t1)
    # print(c)
    t3 = time.time()
    d = [i + j for i, j in zip(a, b)]
    t4 = time.time()
    print(t4 - t3)
