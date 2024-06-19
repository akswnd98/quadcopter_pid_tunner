from randomization.randomization import (
  RandomizationNotifier,
  RandomizationObserver
)
from omni.isaac.core.prims import RigidPrimView
import numpy as np
from pxr import UsdPhysics
from pxr import Gf
from randomization.generator import RandomGenerator

class RandomInertiaModel (RandomizationNotifier):
  def __init__ (self, quadcopters: RigidPrimView, observers: list[RandomizationObserver], env_num, random_inertia_generator: RandomGenerator):
    super().__init__(observers)
    self.quadcopters = quadcopters
    self.inertias = np.zeros((env_num, 3), dtype=np.float32)
    self.env_num = env_num
    self.random_inertia_generator = random_inertia_generator

  def notify (self):
    self.inertias = self.random_inertia_generator.generate()
    super().notify()

class RandomizeInertia (RandomizationObserver):
  def __init__ (self, quadcopters: RigidPrimView):
    self.quadcopters = quadcopters

  def update (self, notifier: RandomInertiaModel):
    for quadcopter, inertia in zip(self.quadcopters.prims, notifier.inertias):
      quadcopter_mass_api = UsdPhysics.MassAPI.Apply(quadcopter)
      quadcopter_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia.tolist()))
