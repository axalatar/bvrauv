from pi.motor_control.pid.PID import PID
from pi.motor_control.pid.differential_PID import DifferentialPID
from typing import Tuple, List
# An abstract model of a sub which contains all the needed PIDs and other low-level details
# to control the sub.


class AUV:
    def __init__(self, wanted_depth: float, wanted_angle: float, motor_min: float, Dp: float, Di: float, Dd: float, Rp: float, Ri: float, Rd: float):
        """
        Dp, Di, and Dd are Kp Ki and Kd for the depth PID, and Rp Ri and Rd are the
        equivalents for the heading PID
        """

        self.wanted_speed: float = 0

        self.depth: float = 0
        # self.rotation = np.array([0, 0, 0])
        # roll (left/right into water) / pitch (forward/backward) / yaw (left/right turning)
        # check out the picture on https://en.wikipedia.org/wiki/Ship_motions
        # in degrees!

        # not using that currently; right now, the rotation will just be turn in theta degrees
        self.heading: float = 0
        self.wanted_heading = wanted_angle
        self.heading_diff = 0
        # TODO better rotation; rotate to vector?

        self.depth_pid: PID = PID(Dp, Di, Dd, wanted_depth)
        self.heading_pid: DifferentialPID = DifferentialPID(PID(Rp, Ri, Rd, 0), motor_min)
    
    def set_depth(self, depth: float) -> None:
        """Sets the current depth of the sub, should be set by IMU."""
        self.depth = depth

    def set_heading(self, heading: float) -> None:
        """Sets the heading of the sub in degrees, should be set by IMU"""
        self.heading = heading

    def set_heading_diff(self, heading_diff: float) -> None:
        """Sets the difference of the sub's heading from where it is to where it should be, corrected"""
        self.heading_diff = heading_diff

    def set_wanted_depth(self, depth: float) -> None:
        """Sets the WANTED depth of the sub, will reset depth PID"""
        self.depth_pid = PID(self.depth_pid.Kp, self.depth_pid.Ki, self.depth_pid.Kd, depth)

    def set_wanted_heading(self, heading: float) -> None:
        """Sets the WANTED heading of the sub, will reset heading PID"""
        self.wanted_heading = heading
        rot = self.heading_pid.pid
        self.heading_pid = DifferentialPID(PID(rot.Kp, rot.Ki, rot.Kd, 0), rot.min)

    def set_wanted_speed(self, speed: float) -> None:
        """Sets the WANTED forward speed of the sub"""
        self.wanted_speed = speed

    # TODO this should return a list with motor labels rather than numbers, eg. ["VFL", 0.5]
    # TODO fix typing output (set length of tuple output)
    def get_motors(self) -> Tuple[float, ...]:
        """
        Gets the calculated speed each of the motors should travel at, in the form \n
        (VFL, VFR, VBL, VBR, HFL, HFR, HBL, HBR) \n
        (eg. HBR = horizontal-back-right, VFL = vertical-front-left)
        """

        vertical_motors: List[float] = [self.depth_pid.signal(self.depth)] * 4
        horizontal_motors: List[float] = [self.wanted_speed] * 4
        
        heading_motors = self.heading_pid.signal(self.heading_diff)  # get heading pid speed (x, y)

        for i in range(4):
            horizontal_motors[i] += heading_motors[i % 2]
            # add to diagonals

        return tuple(vertical_motors + horizontal_motors)
