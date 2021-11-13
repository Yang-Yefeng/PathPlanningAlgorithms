import hashlib
from cv2 import VideoCapture
from moviepy.editor import *

dir_paths = './somefigures/video/mp4/'
save_path = './somefigures/video/gif/'

files = os.listdir(dir_paths)

for file in files:
    file_ext = str(os.path.splitext(file)[-1]).lower()
    filename = str(os.path.splitext(file)[0])
    print(filename)
    if file_ext != '.mp4':
        continue
    file_name = dir_paths + file
    clip = VideoFileClip(file_name)
    v_len = clip.duration
    zoom = 0 if v_len < 3 else 1
    cap = VideoCapture(file_name)
    # 获取视频信息
    if zoom > 0:
        # content = clip.subclip(0, v_len).resize((int(cap.get(3)/zoom), int(cap.get(4)/zoom)))  # 修改分辨率
        content = clip.subclip(0, v_len)
    else:
        content = clip.subclip(0, v_len)  # 不修改分辨率
    # 导出GIF
    md5 = hashlib.md5()
    md5.update(file_name.encode(encoding='utf-8'))
    gif_name = filename + '.gif'
    print(dir_paths)
    content.write_gif(filename=save_path + gif_name, fps=150)
    del(clip, cap, md5)
