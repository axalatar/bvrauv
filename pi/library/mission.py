from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np

from pi.library.sensor_interface import SensorInterface


class Task(ABC):

    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def update(self, sensors: SensorInterface) -> None:
        pass

class Subtask(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def update(self, sensors: SensorInterface, wanted_speed: np.ndarray) -> None:
        pass


class Path:
    def __init__(self, *args: Task):
        self.path: Tuple[Task, ...] = args
