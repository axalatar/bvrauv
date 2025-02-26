from ..mission import Subtask
from ..sensor_interface import SensorInterface
from ..simulation.simulation import Simulation

import numpy as np
import time

class HeadingPID(Subtask):

    def __init__(self, ):
        super().__init__()
        self.simulation = simulation
        self.prevtime = -1.


    @property
    def name(self) -> str:
        return "Simulate subtask"

    def update(self, sensors: SensorInterface, wanted_speed: np.ndarray) -> np.ndarray:
        new_time = time.time()
        if(self.prevtime != -1):
            self.simulation.simulate(new_time - self.prevtime)
        self.prevtime = new_time
        return np.array([0., 0., 0., 0., 0., 0.])
        