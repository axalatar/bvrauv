import time
from collections import deque

from pi.motor_control.pid.PID import PID
from pin_control import PinControl
# A high-level wrapper class for a port control sub with pre-made methods


class UserSub:
    def __init__(self, pin_control: PinControl, Fp: float, Fi: float, Fd: float, max_stop_speed: float, stopping_accuracy: int = 50):
        """
        A high-level wrapper class for a PinControl sub. `Fp`, `Fi`, and `Fd` are PIDs for the forwards PID,
        `max_stop_speed` is the maximum average speed which the sub counts as stopped, and
        `stopping_accuracy` is how many samples the sub will take when checking if its average speed is
        below the threshold
        """
        self.pin_control: PinControl = pin_control
        self.forward_speed: float = 0
        self.Fp: float = Fp
        self.Fi: float = Fi
        self.Fd: float = Fd
        self.max_stop_speed: float = max_stop_speed
        self.stopping_accuracy: int = stopping_accuracy
        self.update_sensors()

    def update_sensors(self) -> None:
        """Update all sensor data"""

        self.pin_control.update_data()
        self.forward_speed = self.pin_control.imu.get_forward_speed()

    def update_motors(self) -> None:
        """Send a packet to update all motors"""

        self.pin_control.update_motors()

    def depth(self) -> float:
        """Current depth"""
        return self.pin_control.sub.depth

    def heading(self) -> float:
        """Current heading"""
        return self.pin_control.sub.heading

    def speed(self) -> float:
        """Current speed"""
        return self.forward_speed

    def set_wanted_speed(self, speed: float) -> None:
        """Sets the current wanted forward speed to `speed`"""
        self.pin_control.sub.set_wanted_speed(speed)

    def set_wanted_heading(self, heading: float) -> None:
        """Sets the current wanted heading to `heading` degrees"""
        self.pin_control.sub.set_heading(heading)

    def set_wanted_depth(self, depth: float) -> None:
        """Sets the current wanted depth to `depth`"""
        self.pin_control.sub.set_wanted_depth(depth)

    def stop_motion(self, timeout: float = 10, interval: float = 0.01) -> bool:
        """
        Stop forward motion using a pid defined using `Kp`, `Ki`, and `Kd`; blocks until average speed over
        `average_length` intervals is less than or equal to max_speed. If it is not stopped after
        `timeout` seconds, it will stop and return False, otherwise True.
        Updates motors every `interval` seconds, default 0.01 (= 10 ms)
        """
        speed_pid = PID(self.Fp, self.Fi, self.Fd, 0)

        start = time.time()
        previous = deque()

        while timeout > time.time() - start:
            if len(previous) >= self.stopping_accuracy:
                average = sum(previous) / self.stopping_accuracy
                if self.max_stop_speed >= average:
                    return True
            previous.appendleft(self.speed())
            previous.pop()

            self.update_sensors()
            self.set_wanted_speed(speed_pid.signal(self.speed()))
            self.update_motors()

            time.sleep(interval)

        return False

    def move_time(self, speed: float, seconds: float) -> bool:
        """
        Move at `speed` for `seconds` seconds before stopping. Returns whether the sub successfully
        stopped after moving.
        """
        self.set_wanted_speed(speed)
        time.sleep(seconds)
        return self.stop_motion()