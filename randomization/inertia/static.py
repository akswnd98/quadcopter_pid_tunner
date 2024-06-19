import numpy as np
from randomization.generator import Vec2GeneratorToXYSymmetricalVec3Generator, BoundedRandomGenerator, LogscaleBoundScaler

'''def generate_random_inertia_generator (env_num: int):
  return Vec2GeneratorToXYSymmetricalVec3Generator(
    BoundedRandomGenerator(
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * (0.1 * 0.2 ** 2 / 12),
        np.ones((env_num, 1), dtype=np.float64) * (10 * 1 ** 2 / 12)
      ], axis=1),
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * (0.1 * 0.2 ** 2 / 6),
        np.ones((env_num, 1), dtype=np.float64) * (10 * 1 ** 2 / 6)
      ], axis=1),
      LogscaleBoundScaler()
    ),
  )'''

def generate_random_inertia_generator (env_num: int):
  return Vec2GeneratorToXYSymmetricalVec3Generator(
    BoundedRandomGenerator(
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * (0.1 * 0.2 ** 2 / 12),
        np.ones((env_num, 1), dtype=np.float64) * (10 * 1 ** 2 / 12)
      ], axis=1),
      np.concatenate([
        np.ones((env_num, 1), dtype=np.float64) * (0.1 * 0.2 ** 2 / 12),
        np.ones((env_num, 1), dtype=np.float64) * (10 * 1 ** 2 / 12)
      ], axis=1),
      LogscaleBoundScaler()
    ),
  )