import copy
import numpy as np
import os
from os import path as osp
import vis_utils as vu

from mujoco_agents.config import AGENT_MUJOCO
import mujoco_py as mjcpy

class SimpleAgentMuJoCo(object):
    """
    All communication between the algorithms and MuJoCo is done through
    this class.
    """
    def __init__(self, hyperparams):
        config = copy.deepcopy(AGENT_MUJOCO)
        config.update(hyperparams)
        self._hyperparams = config
        #Agent.__init__(self, config)
        self._setup_world(hyperparams['filename'])

    def __del__(self):
      self._small_viewer.finish()

    @property
    def model(self):
      return self._model

    def _setup_world(self, filename):
        """
        Helper method for handling setup of the MuJoCo world.
        Args:
            filename: Path to XML file containing the world information.
        """
        self._model = mjcpy.MjModel(filename)
        #self._joint_idx = list(range(self._model['nq']))
        #self._vel_idx = [i + self._model['nq'] for i in self._joint_idx]

        cam_pos = self._hyperparams['camera_pos']
        gofast = True
        self._small_viewer = mjcpy.MjViewer(visible=True,
                                            init_width=AGENT_MUJOCO['image_width'],
                                            init_height=AGENT_MUJOCO['image_height'],
                                            go_fast=gofast)
        self._small_viewer.start()
        self._small_viewer.set_model(self._model)
        self._small_viewer.render()
        #self._small_viewer.loop_once()


    def step(self, ctrl=None):
      if ctrl is not None:
        self._model.data.ctrl = copy.deepcopy(ctrl)
      self.model.step()

    def step_by_n(self, N, ctrl=None):
      for n in range(N):
        self.step(ctrl)

    def render(self):
      self._small_viewer.render()
      self._small_viewer.loop_once()

    def get_image(self):
      img_string, width, height = self._small_viewer.get_image()
      img = np.fromstring(img_string, dtype='uint8').reshape(
            (height, width, self._hyperparams['image_channels']))[::-1, :, :]
      return img

    def interactive(self):
      while True:
        isValid = False
        ip      = raw_input()
        if ip == 'q':
          break
        else:
          cmds = ip.split(',')
          if not len(cmds) == 2:
            continue
          else:
            ctrl = np.array([float(cmds[0]), float(cmds[1])])
        self.step_by_n(5, ctrl)
        self.render()


    def sample(self, policy, condition, verbose=True, save=True, noisy=True):
      pass


def simple_test():
  hyperparams = {}
  DATA_DIR    = osp.join(os.getenv('HOME'), 'code', 'gps', 'mjc_models') 
  hyperparams['filename'] = osp.join(DATA_DIR, 'reacher.xml')
  ag = SimpleAgentMuJoCo(hyperparams)
  return ag 
  ctrl = np.array(([1., 1.]))
  print (ag.model.data.xpos)
  ag.render()
  im1  = ag.get_image()
  ag.step_by_n(1000, ctrl)
  print (ag.model.data.xpos)
  ag.render()
  im2  = ag.get_image()
  vu.plot_pairs(im1, im2) 
  return ag
