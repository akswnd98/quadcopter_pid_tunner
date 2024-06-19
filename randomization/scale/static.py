import numpy as np
from randomization.generator import Vec2GeneratorToXYSymmetricalVec3Generator, BoundedRandomGenerator, LogscaleBoundScaler

'''def generate_random_scale_generator (env_num: int):
  return Vec2GeneratorToXYSymmetricalVec3Generator(
    BoundedRandomGenerator(
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * 0.1,
        np.ones((env_num, 1), dtype=np.float64) * 0.01
      ], axis=1),
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * 1,
        np.ones((env_num, 1), dtype=np.float64) * 0.01
      ], axis=1),
      LogscaleBoundScaler()
    ),
  )'''

def generate_random_scale_generator (env_num: int):
  return Vec2GeneratorToXYSymmetricalVec3Generator(
    BoundedRandomGenerator(
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * 0.3,
        np.ones((env_num, 1), dtype=np.float64) * 0.01
      ], axis=1),
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * 0.3,
        np.ones((env_num, 1), dtype=np.float64) * 0.01
      ], axis=1),
      LogscaleBoundScaler()
    ),
  )
