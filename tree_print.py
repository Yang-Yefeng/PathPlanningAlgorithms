import os
import os.path


def showdir(path, depth, f):
    for item in os.listdir(path):
        if item == '.idea' or item == 'tree.txt' or item == '__pycache__' or item == '__init__.py':
            continue
        # print('item = ', item)
        if "." not in item:
            print("    " * depth + "+ " + item)
            f.write("    " * depth + "+ " + item + "\n")
        else:
            print("    " * depth + "" + item)
            f.write("    " * depth + "" + item + "\n")
        newitem = path + '/' + item
        if os.path.isdir(newitem):
            showdir(newitem, depth + 1, f)


def showmultidir():
    # current_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # print(current_path)
    # print(current_dir)
    # current_dir = "E:\Python\PyCode\RL_Algorithm_V2_0\RL_Algorithm"
    # f = open(current_dir + "\\" + "tree.txt", "w")
    f = open("tree.txt", "w")
    showdir(path=current_dir, depth=0, f=f)
    f.close()


if __name__ == '__main__':
    showmultidir()
