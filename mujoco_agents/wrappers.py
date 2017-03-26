

class GymWrapper(object):
  def __init__(self, obj):
    assert type(obj) is SimpleAgentMujoco, 'type mismatch'
    self._my_obj = obj

  def step(self, action):
    self._my_obj.step()
    obs = 

