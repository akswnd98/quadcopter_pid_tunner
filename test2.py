import numpy as np
from omni.isaac.kit import SimulationApp
import os

CONFIG = {
  "headless": False,
}
simulation_app = SimulationApp(launch_config=CONFIG)


from omni.isaac.core import World

physics_dt = 1.0 / 1000.0
rendering_dt = 1.0 / 30.0
xn = 4
yn = 4
env_num = xn * yn

world = World(stage_units_in_meters=1.0)


import torch

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


from env import QuadcopterOriginPositionsGenerator, generate_pid_tunners, initialize_env

position_generator = QuadcopterOriginPositionsGenerator(xn, yn)
positions = position_generator.generate()

pid_tunner = generate_pid_tunners(positions)
initialize_env(world, pid_tunner, physics_dt, rendering_dt)

from omni.isaac.core.prims import RigidPrimView, RigidPrim
from omni.physics.tensors import create_simulation_view

# quadcopter = RigidPrim('/World/PidTunner_0/quadcopter', 'quadcopter')
# quadcopter.enable_rigid_body_physics()
quadcopters = RigidPrimView('/World/PidTunner_*/quadcopter')
quadcopters.enable_rigid_body_physics()

class PhysicsStepHandler:
  def __init__ (self):
    self.cnt = 0
  
  def get_physics_step_handler (self):
    return lambda step_size: self.handle_physics_step(step_size)

  def handle_physics_step (self, step_size):
    quadcopters.apply_forces_and_torques_at_pos(
      forces=[np.array([0, 0, 10], dtype=np.float64)],
      positions=[np.array([0, 1, 0], dtype=np.float64)],
      indices=[0],
      is_global=False
    )
    if self.cnt % 1000 == 0:
      pass
      # quadcopters.apply_forces(forces=[np.array([0, 0, 1], dtype=np.float64)], indices=[0], is_global=False)
    self.cnt += 1

physics_step_handler = PhysicsStepHandler()

world.add_physics_callback('main', physics_step_handler.get_physics_step_handler())

while True:
  physics_step_handler.cnt = 0

  world.reset()
  quadcopters.initialize()
  for _ in range(5000):
    world.step()
    physics_step_handler.cnt += 1
  world.stop()

simulation_app.close()
