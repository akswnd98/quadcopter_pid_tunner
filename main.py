import numpy as np
from omni.isaac.kit import SimulationApp
import os

CONFIG = {
  "headless": False,
}
simulation_app = SimulationApp(launch_config=CONFIG)


from omni.isaac.core import World

physics_dt = 1.0 / 1000.0
rendering_dt = 1.0 / 50.0
xn = 4
yn = 4
env_num = xn * yn
assets_root_path = 'omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/'
quadcopter_usd_path = os.path.join(assets_root_path, 'Isaac/Robots/Quadcopter/quadcopter.usd')

world = World(stage_units_in_meters=1.0)


import torch

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


from env import QuadcopterOriginPositionsGenerator, generate_pid_tunners, initialize_env

position_generator = QuadcopterOriginPositionsGenerator(xn, yn)
positions = position_generator.generate()

pid_tunners = generate_pid_tunners(positions)
initialize_env(world, pid_tunners, physics_dt, rendering_dt)

from omni.isaac.core.prims import RigidPrimView
from randomization.randomization import RandomizationNotifier, RandomizationNotifierToObserverAdapter
from randomization.scale import RandomScaleModel, RandomizeScale
from randomization.mass import RandomMassModel, RandomizeMass
from randomization.inertia import RandomInertiaModel, RandomizeInertia
from randomization.max_thrust import RandomMaxThrustModel
from randomization.initial_state import InitialStateRandomizer

from physics_step_preparer import ContainerPhysicsStepPreparer
from physics_step_preparer.sensor.imu_sensor import ImuSensor
from physics_step_preparer.sensor.euler_state import EulerState, ZeroInitialization
from physics_step_preparer.simulation_over_table import SimulationOverTable
from physics_step_preparer.control_system import QuadcopterControlSystem
from randomization.inertia.static import generate_random_inertia_generator
from randomization.mass.static import generate_random_mass_generator
from randomization.scale.static import generate_random_scale_generator
from randomization.max_thrust.static import generate_random_max_thrust_generator
from randomization.initial_state.static import generate_random_initial_state_generator

quadcopters = RigidPrimView('/World/PidTunner_*/quadcopter')
quadcopters.enable_rigid_body_physics()

random_scale_model = RandomScaleModel(
  quadcopters,
  [RandomizeScale(quadcopters)],
  env_num,
  generate_random_scale_generator(env_num)
)
random_mass_model = RandomMassModel(
  quadcopters,
  [RandomizeMass(quadcopters)],
  env_num,
  generate_random_mass_generator(env_num)
)
random_inertia_model = RandomInertiaModel(
  quadcopters,
  [RandomizeInertia(quadcopters)],
  env_num,
  generate_random_inertia_generator(env_num)
)
random_max_thrust_model = RandomMaxThrustModel(
  env_num,
  generate_random_max_thrust_generator(random_mass_model)
)
initial_state_randomizer = InitialStateRandomizer(pid_tunners, generate_random_initial_state_generator(env_num))

randomizer = RandomizationNotifier([
  RandomizationNotifierToObserverAdapter(random_scale_model),
  RandomizationNotifierToObserverAdapter(random_mass_model),
  RandomizationNotifierToObserverAdapter(random_inertia_model),
  random_max_thrust_model,
  initial_state_randomizer
])

imu_sensor = ImuSensor(env_num)
euler_state = EulerState(imu_sensor, ZeroInitialization(env_num))

sensor_container = ContainerPhysicsStepPreparer([
  imu_sensor,
  euler_state
])

simulation_over_table = SimulationOverTable(env_num, imu_sensor)

control_system = QuadcopterControlSystem(
  P_phi=1e+1, D_phi=0e+0,
  P_theta=0e+0, D_theta=0e+0,
  P_w_x=1e+0, D_w_x=0e+0,
  P_w_y=0e+0, D_w_y=0e+0,
  P_w_z=0e+0, D_w_z=0e+0,
  imu_sensor=imu_sensor, euler_state=euler_state, simulation_over_table=simulation_over_table,
  random_scale_model=random_scale_model, quadcopters=quadcopters, env_num=env_num
)

physics_step_preparer = ContainerPhysicsStepPreparer([
  sensor_container,
  simulation_over_table,
  control_system
])

class PhysicsStepHandler:
  def __init__ (self):
    self.cnt = 0
  
  def get_physics_step_handler (self):
    return lambda step_size: self.handle_physics_step(step_size)

  def handle_physics_step (self, step_size):
    physics_step_preparer.update()
    if self.cnt % 1000 == 0:
      pass
      # print(f'euler_state.phi[0] euler_state.phi_dot[0]: {euler_state.phi[0]}, {euler_state.phi_dot[0]}')
      # print(f'orientation: {imu_sensor.cur_state[:, 9: ]}')
    self.cnt += 1

physics_step_handler = PhysicsStepHandler()

world.add_physics_callback('main', physics_step_handler.get_physics_step_handler())

while True:
  print('hello')
  physics_step_handler.cnt = 0
  randomizer.notify()
  simulation_over_table.initialize()

  world.reset()
  quadcopters.initialize()
  for _ in range(250):
    world.step()
  print(physics_step_handler.cnt)
  world.stop()

simulation_app.close()
