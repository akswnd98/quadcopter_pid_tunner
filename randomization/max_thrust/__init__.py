from randomization.randomization import (
  RandomizationNotifier,
  RandomizationObserver,
)
import numpy as np
from randomization.generator import RandomGenerator, BoundedRandomGenerator, BoundScaler
from randomization.mass import RandomMassModel
from env import PhysicsConstant

class RandomMaxThrustModel (RandomizationObserver):
  def __init__ (self, env_num, random_max_thrust_generator: RandomGenerator):
    super().__init__()
    self.max_thrust = np.zeros((env_num, ), dtype=np.float64)
    self.env_num = env_num
    self.random_max_thrust_generator = random_max_thrust_generator

  def observe (self, notifier: RandomizationNotifier):
    self.scales = self.random_max_thrust_generator.generate()

class RandomMaxThrustGenerator (BoundedRandomGenerator):
  def __init__ (self, bound_scaler: BoundScaler, mass_model: RandomMassModel):
    super().__init__(None, None, bound_scaler)
    self.mass_model = mass_model

  def generate (self):
    self.lower_bound = self.mass_model.masses * PhysicsConstant.g / 3.5
    self.upper_bound = self.mass_model.masses * PhysicsConstant.g / 1.5
    return super().generate()
