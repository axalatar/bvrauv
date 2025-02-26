import numpy as np
from library.auv import AUV
from library.sensor_interface import SensorInterface
from library.motor_controller import MotorController, Motor
from library.inertia import InertiaBuilder, Cuboid
from library.tasks.simulate import Simulate
from library.tasks.accelerate_vector import AccelerateVector
from library.mission import Path

from library.simulation.simulation import Simulation
motor_locations = [
    np.array([-1., -1., 0.]),
    np.array([-1., 1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.])
    ]

motor_directions = [
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.])
    ]



# print([f"{i}, {loc}, {direction}" for i, (loc, direction) in enumerate(zip(motor_locations, motor_directions))])


bounds = [[-1, 1]] * 4
deadzone = [[-0.1, 0.1]] * 4
sim = Simulation(motor_locations, motor_directions, 1/6, bounds, deadzone)

sim_anchovy = AUV(
    motor_controller = MotorController(
        inertia = InertiaBuilder(
            Cuboid(
                mass=1,
                width=1,
                height=1,
                depth=0.1,
                center=np.array([0,0,0])
            )).moment_of_inertia(),
            motors = [
                Motor(
                    direction,
                    loc,
                    sim.set_motor(i),
                    lambda: 0,
                    Motor.Range(bounds[i][0], bounds[i][1]),
                    Motor.Range(-deadzone[i][0], deadzone[i][1])
                    )
                for i, (loc, direction) in enumerate(zip(motor_locations, motor_directions))
                ]
        ),
        sensors = SensorInterface(imu=sim.imu(0.05), depth=sim.depth(0.01)),
        pin_kill = lambda: None
    )

sim_anchovy.register_subtask(Simulate(sim))

# print()

mission = Path(
    AccelerateVector(np.array([1., 0., 0., 0., 0., 0.]), 1),
    AccelerateVector(np.array([0., 1., 0., 0., 0., 0.]), 2),
    AccelerateVector(np.array([-0.5, -0.5, 0., 0., 0., 0.]), 3),
    AccelerateVector(np.array([0., 0., 0., 0., 0., 1.]), 5)
)

sim_anchovy.travel_path(mission)

sim.render()