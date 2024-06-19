import numpy as np
from randomization.generator import BoundedRandomGenerator, LogscaleBoundScaler

'''def generate_random_mass_generator (env_num: int):
  return BoundedRandomGenerator(
    np.ones((env_num, ), dtype=np.float64) * 0.1,
    np.ones((env_num, ), dtype=np.float64) * 10,
    LogscaleBoundScaler()
  )'''

def generate_random_mass_generator (env_num: int):
  return BoundedRandomGenerator(
    np.ones((env_num, ), dtype=np.float64) * 1,
    np.ones((env_num, ), dtype=np.float64) * 1,
    LogscaleBoundScaler()
  )
