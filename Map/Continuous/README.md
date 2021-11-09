# obstacle.py
This module is designed to define obstacles in a continuous map, including circle, ellipse, and convex polygon (triangle, rectangle, pentagon, hexagon).
One can use it by typing:
```
from obstacle import obstacle

obs = [['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
       ['rectangle', [3, 3.5],   [2.0, 5.0, 30.]],
       ['pentagon',  [7, 8.5],   [1.0, 180.0]],
       ['hexagon',   [8.0, 2],   [1.0, 30.0]],
       ['circle',    [6, 6],     [1.0]],
       ['ellipse',   [3, 8],     [2.6, 0.6, -20.0]]]
obs = obstacle(obs).get_obs()
```
Detailed formation of 'obs' can be found in the comment of obstacle.py

# samplingmap.py
This module defines continuous map for sampling based path planning algorithms.

# test_sampling_map.py
This is designed for testing samplingmap.py
Here is a demo by running test_sampling_map.py.

## a demo video
<video id="video" controls="" preload="none" poster="封面">
      <source id="mp4" src="mp4格式视频" type="video/mp4">
</videos>
