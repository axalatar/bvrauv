from auv import AUV
from sensor_interface import DepthInterface, ImuInterface, SensorInterface
from motor_controller import MotorController, Motor
import numpy as np

from inertia import InertiaBuilder, Cuboid, HollowCylinder

anchovy = AUV(
    motor_controller=MotorController(
        
        inertia=InertiaBuilder(
            Cuboid(
                mass=3, 
                size=np.array([12, 2.5, 12]), 
                center=np.array([0,0,0]),
                normal=np.array([0,1,0])
                )
        ).moment_of_inertia(),

        motors=[
            Motor(np.array([1, 0, 1]), np.array([5, 0, 5]), lambda x: set_motor(0, x)),
            Motor(np.array([1, 0, -1]), np.array([-5, 0, 5], lambda x: set_motor(1, x))),
            Motor(np.array([1, 0, 1]), np.array([-5, 0, -5], lambda x: set_motor(2, x))),
            Motor(np.array([1, 0, -1]), np.array([5, 0, -5], lambda x: set_motor(3, x))),

            Motor(np.array([0, 1, 0]), np.array([4, 0, 4], lambda x: set_motor(4, x))),
            Motor(np.array([0, 1, 0]), np.array([4, 0, -4], lambda x: set_motor(5, x))),
            Motor(np.array([0, 1, 0]), np.array([-4, 0, -4], lambda x: set_motor(6, x))),
            Motor(np.array([0, 1, 0]), np.array([-4, 0, 4], lambda x: set_motor(7, x))),
        ]
    ),

    sensors=SensorInterface(
        imu=IMU(),
        depth=DepthSensor()
    ),

    logging=False,      # whether to log to a file
    console=False,      # whether to log to the console

    pin_kill=emergency_kill   # directly access pins to disable all motors, for emergency use
)


mission = Path(
    Calibrate(),                                        # calibrate all sensors
    BeginHoldDepth(depth=10),                           # begin to hold depth, initial depth of 10
        
    TravelVectorTimed(np.array([1, 0, 0]), seconds=10), # travel on the x axis for 10 seconds

    HoldPosition(wait=3),                               # once it's arrived, realign until still for 3 secs

    Spin(spins=3),                                      # spin three times

    TravelVectorTimed(np.array([2, 0, 0]), seconds=5),  # travel on the x axis twice as fast for 5 secs

    TravelVectorTimed(np.array([0, 4, 0]), seconds=5)   # travel straight up for 5 secs at max speed
)                                                       # end of path, motors killed

anchovy.travel_path(mission)