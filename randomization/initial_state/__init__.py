from randomization.randomization import (
  RandomizationNotifier,
  RandomizationObserver
)
import numpy as np
from randomization.generator import RandomGenerator
from omni.isaac.core.articulations import ArticulationView

class InitialStateRandomizer (RandomizationObserver):
  def __init__ (self, pid_tunners: ArticulationView, random_initial_state_generator: RandomGenerator):
    super().__init__()
    self.pid_tunners = pid_tunners
    self.random_initial_state_generator = random_initial_state_generator

  def update (self, notifier: RandomizationNotifier):
    random_initial_state = self.random_initial_state_generator.generate()
    self.pid_tunners.set_joints_default_state(
      np.concatenate([np.expand_dims(random_initial_state, axis=1)], axis=1)
    )
