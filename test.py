import numpy as np

if __name__ == '__main__':
  a = np.array([1, 2, 3], dtype=np.float64) < 2
  print(((a ^ True) * np.array([2, 3, 4], dtype=np.float64)))
