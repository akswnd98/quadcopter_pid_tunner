import numpy as np
from omni.isaac.core.prims import RigidPrimView
from physics_step_preparer import PhysicsStepPreparer
from physics_step_preparer.sensor.imu_sensor import ImuSensor

class SimulationOverTable (PhysicsStepPreparer):
  def __init__ (self, env_num: int, imu_sensor: ImuSensor):
    self.env_num = env_num
    self.table = np.zeros((self.env_num, ), dtype=np.bool)
    self.imu_sensor = imu_sensor

  def update (self):
    self.table = self.table | (np.abs(self.imu_sensor.cur_state[:, 3]) >= np.pi * 2 * 10)

  def initialize (self):
    self.table = np.zeros((self.env_num, ), dtype=np.bool)
