import numpy as np
from randomization.generator import LogscaleBoundScaler
from randomization.max_thrust import RandomMaxThrustGenerator
from randomization.mass import RandomMassModel

def generate_random_max_thrust_generator (random_mass_model: RandomMassModel):
  return RandomMaxThrustGenerator(
    LogscaleBoundScaler(),
    random_mass_model
  )
