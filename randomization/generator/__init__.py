import numpy as np

class RandomGenerator:
  def generate (self):
    pass

class BoundScaler:
  def scale_bound (self, lower_bound, upper_bound):
    pass

  def reverse_scale (self, val):
    pass

class LinearscaleBoundScaler (BoundScaler):
  def scale_bound (self, lower_bound, upper_bound):
    return lower_bound, upper_bound
  
  def reverse_scale (self, val):
    return val

class LogscaleBoundScaler (BoundScaler):
  def scale_bound (self, lower_bound, upper_bound):
    return np.log(lower_bound), np.log(upper_bound)
  
  def reverse_scale (self, val):
    return np.exp(val)

class BoundedRandomGenerator (RandomGenerator):
  def __init__ (self, lower_bound, upper_bound, bound_scaler: BoundScaler):
    self.lower_bound = lower_bound
    self.upper_bound = upper_bound
    self.bound_scaler = bound_scaler
  
  def generate (self):
    scaled_lower_bound, scaled_upper_bound = self.bound_scaler.scale_bound(self.lower_bound, self.upper_bound)
    return self.bound_scaler.reverse_scale(np.random.uniform(scaled_lower_bound, scaled_upper_bound))

class Vec2GeneratorToXYSymmetricalVec3Generator (RandomGenerator):
  def __init__ (self, vec2_generator):
    super().__init__()
    self.vec2_generator = vec2_generator

  def generate (self) -> np.ndarray[np.ndarray[np.float64]]:
    vec2 = self.vec2_generator.generate()
    return np.concatenate([vec2[:, [0]], vec2], axis=1)
