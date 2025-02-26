from ..sensor_interface import DepthInterface, ImuInterface
from ..motor_controller import MotorController
import numpy as np
from scipy.ndimage import rotate
from .simulation_animator import SimulationAnimator
from scipy.spatial.transform import Rotation as R

import quaternion

# kinda sucks, 2d
class Simulation:

    def __init__(self, motor_locations, motor_directions, inertia, bounds, deadzone):
        self.location = np.array([0., 0.])
        self.velocity = np.array([0., 0.])
        self.acceleration = np.array([0., 0.])
        self.rotation = 0 # degrees
        # start by facing positive x axis
        self.rotational_velocity = 0
        self.rotational_acceleration = 0

        self.motor_locations = motor_locations # relative to center
        self.motor_directions = motor_directions
        self.motor_speeds = np.array([np.array([0., 0., 0.]) for _ in motor_directions])
        # print(self.motor_speeds)
        self.moment_of_inertia = inertia

        self.timestep = 0.01 # in seconds
        self.prevtime = 0
        self.animator = SimulationAnimator(fps=1/self.timestep)

        self.drag = 0

        self.bounds = bounds
        self.deadzone = deadzone
    
    def simulate(self, time):
        for timepoint in np.arange(self.prevtime, time, self.timestep):
            self.location += self.velocity * self.timestep
            # print(self.acceleration)
            # print(self.timestep)
            self.velocity += self.acceleration * self.timestep
            self.rotation += self.rotational_velocity * self.timestep
            self.rotational_velocity += self.rotational_acceleration * self.timestep
            rotation = R.from_euler('z', np.deg2rad(self.rotation))
            
            rotated_speeds = []
            rotated_locations = []
            torques = []
            for i in range(len(self.motor_speeds)):
                s = np.delete(rotation.apply(np.array([self.motor_speeds[i][0], self.motor_speeds[i][1], 0.0])), 2)
                l = np.delete(rotation.apply(np.array([self.motor_locations[i][0], self.motor_locations[i][1], 0.0])), 2)
                rotated_speeds.append(s)
                rotated_locations.append(l)
                torques.append(np.cross(l, s))
            self.acceleration = sum(rotated_speeds)
            torque = sum(torques)

            self.rotational_acceleration = torque / self.moment_of_inertia
            
            self.animator.append(
                self.location,
                self.rotation,
                self.velocity,
                [loc + self.location for loc in rotated_locations],
                rotated_speeds
            )
            self.rotational_velocity *= (1 - self.drag)
            self.velocity *= (1 - self.drag)

    def render(self):
        self.animator.render()

    def update_motor_speeds(self, speeds):
        self.motor_speeds = [d * s for d, s in zip(self.motor_directions, speeds)]

    class FakeDepthSensor(DepthInterface):
        def __init__(self, deviation):
            self.deviation = deviation

        def get_depth(self):
            return 0.
        
        def initialize(self):
            pass

        def overview(self) -> str:
            return f"Simulated Depth Sensor -- Standard deviation: {self.deviation}. Currently hovercraft-only, no depth."
        
    class FakeIMU(ImuInterface):

        def __init__(self, max_dev_accel, acceleration_function, rotation_function):
            self.max_dev_accel = max_dev_accel
            self.acceleration_function = acceleration_function
            self.rotation_function = rotation_function

        def get_accelerations(self):
            acceleration = self.acceleration_function() + (np.random.rand(2) - 0.5) * 2 * self.max_dev_accel
            return np.append(acceleration, 0) # bc it expects a 3d acceleration
        
        def get_rotation(self):
            return self.rotation_function()

        def initialize(self):
            pass

        def overview(self) -> str:
            return f"Simulated IMU -- Standard deviation: {self.max_dev_accel}"
        

    def depth(self, dev):
        return self.FakeDepthSensor(dev)
    
    def imu(self, dev):
        return self.FakeIMU(dev, lambda: self.acceleration, lambda: quaternion.from_euler_angles([self.rotation, 0, 0]))
    
    def set(self, index, speed):
            
            speed = max(min(speed, self.bounds[index][1]), self.bounds[index][0])
            if(self.deadzone[index][0] > speed > self.deadzone[index][1]):
                speed = 0
            self.motor_speeds[index] = self.motor_directions[index] * speed
            # print(self.motor_speeds)
    
    def set_motor(self, index):
        return lambda speed: self.set(index, speed)
        

        

# a = Simulation(
#         np.array([
#             np.array([-1., -1.]),
#             np.array([-1., 1.]),
#             np.array([1., 1.]),
#             np.array([1., -1.])
#         ]),
#         np.array([
#                 np.array([1., -1.]),
#                 np.array([1., 1.]),
#                 np.array([1., -1.]),
#                 np.array([1., 1.])
#         ]),
#         1/6
# )
# # a.update_motor_speeds(np.array([1, 1, 1, 1]))
# # a.simulate(2)
# # a.update_motor_speeds(np.array([1, -1, -1, 1]))
# # a.simulate(8)
# # a.update_motor_speeds(np.array([0, 0, 0, 0]))
# # a.simulate(8)
# # # a.update_motor_speeds(np.array([0, 0, 0, 0]))
# # # a.simulate(6)
# # a.render()


# # # def emergency_kill():
#     # pass

# # def set_motor(id, speed):
# #     set_pin(id, speed)
