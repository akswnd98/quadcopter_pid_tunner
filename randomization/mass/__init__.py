from randomization.randomization import (
  RandomizationNotifier,
  RandomizationObserver
)
from omni.isaac.core.prims import RigidPrimView
from pxr import UsdPhysics
from randomization.generator import RandomGenerator

class RandomMassModel (RandomizationNotifier):
  def __init__ (self, quadcopters: RigidPrimView, observers: list[RandomizationObserver], env_num, random_mass_generator: RandomGenerator):
    super().__init__(observers)
    self.quadcopters = quadcopters
    self.env_num = env_num
    self.random_mass_generator = random_mass_generator

  def notify (self):
    self.masses = self.random_mass_generator.generate()
    super().notify()

class RandomizeMass (RandomizationObserver):
  def __init__ (self, quadcopters: RigidPrimView):
    self.quadcopters = quadcopters

  def update (self, notifier: RandomMassModel):
    for quadcopter, mass in zip(self.quadcopters.prims, notifier.masses):
      quadcopter_mass_api = UsdPhysics.MassAPI.Apply(quadcopter)
      quadcopter_mass_api.CreateMassAttr(mass)
