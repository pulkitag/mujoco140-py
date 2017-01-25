import mujoco_py
import numpy as np
#from scipy.misc import imsave
import time
import os
from os import path as osp

DATA_DIR    = osp.join(os.getenv('HOME'), 'code', 'gps', 'mjc_models', 'reacher.xml')
fullpath = './example.xml'
model = mujoco_py.MjModel(DATA_DIR)
width = 1000
height = 1000
viewer = mujoco_py.MjViewer(visible=True, init_width=width, init_height=height)
viewer.start()
viewer.set_model(model)

steps = 100
skip = 10

start = time.time()
for i in xrange(steps):
    viewer.render()
    viewer.loop_once()
    data, width, height = viewer.get_image()
    img = np.fromstring(data, dtype='uint8').reshape(height, width, 3)[::-1,:,:]
    #imsave('imgs/out_' + str(i) + '.png', img)
    #model.data.ctrl = np.random.randn(1,2)
    model.data.ctrl = np.array([1., 1.])
    for j in xrange(skip):
        model.step()
    time.sleep(0.1)
end = time.time()
print(end - start)

viewer.finish()
viewer = None
