import numpy as np
from randomization.generator import BoundedRandomGenerator, LinearscaleBoundScaler

def generate_random_initial_state_generator (env_num: int):
  return BoundedRandomGenerator(
    np.ones((env_num, ), dtype=np.float64) * -np.pi / 6,
    np.ones((env_num, ), dtype=np.float64) * np.pi / 6,
    LinearscaleBoundScaler()
  )
