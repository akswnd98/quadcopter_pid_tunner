from randomization.randomization import (
  RandomizationNotifier,
  RandomizationObserver
)
from omni.isaac.core.prims import RigidPrimView
import numpy as np
from randomization.generator import RandomGenerator

class RandomScaleModel (RandomizationNotifier):
  def __init__ (self, quadcopters: RigidPrimView, observers: list[RandomizationObserver], env_num, random_scale_generator: RandomGenerator):
    super().__init__(observers)
    self.quadcopters = quadcopters
    self.scales = np.zeros((env_num, 3), dtype=np.float32)
    self.env_num = env_num
    self.random_scale_generator = random_scale_generator

  def notify (self):
    self.scales = self.random_scale_generator.generate()
    super().notify()

class RandomizeScale (RandomizationObserver):
  def __init__ (self, quadcopters: RigidPrimView):
    self.quadcopters = quadcopters

  def update (self, notifier: RandomScaleModel):
    self.quadcopters.set_local_scales(notifier.scales)
    # for i, quadcopter in zip(range(len(self.quadcopters.prims)), self.quadcopters.prims):
      # quadcopter.GetAttribute('xformOp:scale').Set(Gf.Vec3f(notifier.scales[i].tolist()))
      # XformCommonAPI.Get(omni.usd.get_context().get_stage(), quadcopter_path).SetScale(Gf.Vec3f(notifier.scales[i].tolist()), 0)
