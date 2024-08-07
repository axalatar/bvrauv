from typing import Optional, Tuple, Dict
from pi.motor_control.direction import direction

# apply to motors, as some are reversed. applied at .toString()
motor_reverse_fix = {
    "VFL": -1,
    "VFR": -1,
    "VBL": -1,
    "VBR": 1,
    "HFL": 1,
    "HFR": 1,
    "HBL": 1,
    "HBR": 1
}

encoded_direction_fix = {direction(key): value for key, value in motor_reverse_fix.items()}


# A class to choose which motors to enable/disable when sending a message
class MotorPacket:
    def __init__(self, commands: Optional[Dict[int, float]] = None):
        """
        `commands` is a list of tuples, each of which contains
        (magnitude (to be sent to arduino), motor (in direction format))
        If unset, the packet will be empty
        """
        self.commands: Dict[int, float] = {}

        if commands is not None:
            self.setCommands(commands)

    def getCommands(self) -> Dict[int, float]:
        return self.commands

    def setCommands(self, commands: Dict[int, float]) -> None:
        """
        Set the full command list. `commands` is in the same format as in __init__
        """
        self.commands = commands.copy()

    def addCommand(self, command: Tuple[int, float]) -> None:
        """
        Add a command. `command` is a tuple in the same format as in __init__
        """
        self.commands[command[0]] = command[1]

    def toString(self) -> str:
        string_builder = []

        for motor, magnitude in self.commands.items():
            string_builder.append(','.join([str(magnitude * encoded_direction_fix[motor]), str(motor)]))

        return ':'.join(string_builder) + ":"

    def __add__(self, other):
        sum_commands = self.commands.copy()
        for key, magnitude in other.commands():
            if sum_commands[key] is not None:
                sum_commands[key] += magnitude
            sum_commands[key] = magnitude

        return MotorPacket(sum_commands)


def all_motors_packet(speeds: Tuple[float, ...]) -> MotorPacket:
    """
    Sends commands to all motors, in the format\n
    (VFL, VFR, VBL, VBR, HFL, HFR, HBL, HBR)\n
    Where each value should be a motor speed. Serial is a MotorControl object.
    """
    commands = {i: speeds[i] for i in range(len(speeds))}
    return MotorPacket(commands)
