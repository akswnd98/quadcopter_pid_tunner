import numpy as np
from physics_step_preparer.sensor.imu_sensor import ImuSensor
from physics_step_preparer.sensor.euler_state import EulerState
from physics_step_preparer.simulation_over_table import SimulationOverTable
from omni.isaac.core.prims import RigidPrimView
from randomization.scale import RandomScaleModel
from physics_step_preparer import PhysicsStepPreparer

class QuadcopterControlSystem (PhysicsStepPreparer):
  def __init__ (
    self,
    P_phi: np.ndarray, D_phi: np.ndarray,
    P_theta: np.ndarray, D_theta: np.ndarray,
    P_w_x: np.ndarray, D_w_x: np.ndarray,
    P_w_y: np.ndarray, D_w_y: np.ndarray,
    P_w_z: np.ndarray, D_w_z: np.ndarray,
    imu_sensor: ImuSensor, euler_state: EulerState, simulation_over_table: SimulationOverTable,
    random_scale_model: RandomScaleModel,
    quadcopters: RigidPrimView,
    env_num: int
  ):
    self.P_phi, self.D_phi, self.P_theta, self.D_theta = P_phi, D_phi, P_theta, D_theta
    self.P_w_x, self.D_w_x, self.P_w_y, self.D_w_y, self.P_w_z, self.D_w_z = P_w_x, D_w_x, P_w_y, D_w_y, P_w_z, D_w_z
    self.imu_sensor = imu_sensor
    self.euler_state = euler_state
    self.simulation_over_table = simulation_over_table
    self.random_scale_model = random_scale_model
    self.quadcopters = quadcopters
    self.env_num = env_num
    self.cnt = 0

  def update (self):
    thrust = self.calculate_thrust()
    if self.cnt % 1000 == 0:
      pass
      # print(f'thrust: {thrust}')
    self.cnt += 1
    self.quadcopters.apply_forces_and_torques_at_pos(
      forces=np.concatenate([
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.expand_dims(np.logical_xor(self.simulation_over_table.table, True), axis=-1) * thrust[:, [0]]
      ], axis=-1),
      positions=np.concatenate([
        -self.random_scale_model.scales[:, [0]],
        self.random_scale_model.scales[:, [1]],
        np.zeros((self.env_num, 1), dtype=np.float64)
      ], axis=-1),
      is_global=False
    )
    self.quadcopters.apply_forces_and_torques_at_pos(
      forces=np.concatenate([
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.expand_dims(np.logical_xor(self.simulation_over_table.table, True), axis=-1) * thrust[:,[1]]
      ], axis=-1),
      positions=np.concatenate([
        self.random_scale_model.scales[:, [0]],
        self.random_scale_model.scales[:, [1]],
        np.zeros((self.env_num, 1), dtype=np.float64)
      ], axis=-1),
      is_global=False
    )
    self.quadcopters.apply_forces_and_torques_at_pos(
      forces=np.concatenate([
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.expand_dims(np.logical_xor(self.simulation_over_table.table, True), axis=-1) * thrust[:,[2]]
      ], axis=-1),
      positions=np.concatenate([
        self.random_scale_model.scales[:, [0]],
        -self.random_scale_model.scales[:, [1]],
        np.zeros((self.env_num, 1), dtype=np.float64)
      ], axis=-1),
      is_global=False
    )
    self.quadcopters.apply_forces_and_torques_at_pos(
      forces=np.concatenate([
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.zeros((self.env_num, 1), dtype=np.float64),
        np.expand_dims(np.logical_xor(self.simulation_over_table.table, True), axis=-1) * thrust[:,[3]]
      ], axis=-1),
      positions=np.concatenate([
        -self.random_scale_model.scales[:, [0]],
        -self.random_scale_model.scales[:, [1]],
        np.zeros((self.env_num, 1), dtype=np.float64)
      ], axis=-1),
      is_global=False
    )

  def calculate_phi_error (self, phi_ref: np.ndarray):
    return phi_ref - self.euler_state.phi

  def calculate_phi_dot_ref (self, phi_error: float):
    return phi_error * self.P_phi + self.euler_state.phi_dot * self.D_phi

  def calculate_theta_error (self, theta_ref: float):
    return theta_ref - self.euler_state.theta

  def calculate_theta_dot_ref (self, theta_error: float):
    return theta_error * self.P_theta + self.euler_state.theta_dot * self.D_theta

  def calculate_psi_dot_ref (self, psi_dot_ref):
    return psi_dot_ref

  def calculate_eta_dot_ref (self, phi_dot_ref, theta_dot_ref, psi_dot_ref):
    return np.concatenate([
      np.expand_dims(phi_dot_ref, axis=(-2, -1)),
      np.expand_dims(theta_dot_ref, axis=(-2, -1)),
      np.expand_dims(psi_dot_ref, axis=(-2, -1))
    ], axis=-2)

  def calculate_C (self):
    return np.array([np.array([
      [1, 0, -np.sin(theta)],
      [0, np.cos(phi), np.sin(phi) * np.cos(theta)],
      [0, -np.sin(phi), np.cos(phi) * np.cos(theta)]
    ]) for phi, theta in zip(self.euler_state.phi, self.euler_state.theta)])

  def calculate_w_ref (self, eta_dot_ref: np.ndarray, C: np.ndarray):
    return np.array([C_i @ eta_dot_ref_i for eta_dot_ref_i, C_i in zip(eta_dot_ref, C)])

  def calculate_w_error (self, w_ref: np.ndarray, w: np.ndarray):
    return w_ref - w

  def calculate_w_dot_ref (self, w_error: np.ndarray, w_dot: np.ndarray):
    return np.concatenate([
      w_error[:, 0: 1] * self.P_w_x + w_dot[:, 0: 1] * self.D_w_x,
      w_error[:, 1: 2] * self.P_w_y + w_dot[:, 1: 2] * self.D_w_y,
      w_error[:, 2: 3] * self.P_w_z + w_dot[:, 2: 3] * self.D_w_z
    ], axis=1)

  def calculate_delta_thrust (self, w_dot_ref: np.ndarray):
    return 0.5 * np.concatenate([
      np.expand_dims(w_dot_ref[:, 0, 0] + w_dot_ref[:, 1, 0], axis=-1),
      np.expand_dims(w_dot_ref[:, 0, 0] - w_dot_ref[:, 1, 0], axis=-1),
      np.expand_dims(-w_dot_ref[:, 0, 0] - w_dot_ref[:, 1, 0], axis=-1),
      np.expand_dims(-w_dot_ref[:, 0, 0] + w_dot_ref[:, 1, 0], axis=-1)
    ], axis=-1)
    # / (l * signal_force_ratio)
  
  def calculate_thrust (self):
    delta_thrust = self.calculate_delta_thrust(
      self.calculate_w_dot_ref(
        self.calculate_w_error(
          self.calculate_w_ref(
            self.calculate_eta_dot_ref(
              self.calculate_phi_dot_ref(
                self.calculate_phi_error(
                  np.zeros((self.env_num, ), dtype=np.float64)
                ),
              ),
              self.calculate_theta_dot_ref(
                self.calculate_theta_error(
                  np.zeros((self.env_num, ), dtype=np.float64)
                )
              ),
              self.calculate_psi_dot_ref(np.zeros((self.env_num, ), dtype=np.float64))
            ),
            self.calculate_C()
          ),
          np.expand_dims(self.imu_sensor.cur_state[:, 3: 6], axis=-1)
        ),
        np.expand_dims(self.imu_sensor.cur_state[:, 6: 9], axis=-1)
      )
    )
    return delta_thrust
