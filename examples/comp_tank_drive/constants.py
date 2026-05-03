from wpimath.units import meters


class RobotConst:
    """
    Represents universal robot constants (not bounded to a particular subsystem).
    """
    ROBOT_WIDTH: meters = 0.6604
    ROBOT_LENGTH: meters = 0.7112
    BUMPER_HEIGHT: meters = 0.1778


class JoystickButton:
    """
    Represents universally used button constants & bindings.
    """
    INTAKE_ARM_UP = 1
    INTAKE_ARM_DOWN = 2

    INTAKE_GRABBING = 3
    INTAKE_RELEASING = 4

    SHOOTER_SPINNING = 5

    GAME_SIM_SPAWN = 6  # Spawns a line of balls in the center of the simulated field.
    GAME_SIM_CLEAR = 7  # Clears all fuels from the simulated field.
    GAME_SIM_SHOOT = 8  # Shoots a ball directly from the robot in the simulated field.



class JoystickAxis:
    """
    Represents universally used axis (joystick) constants & bindings.
    """
    DRIVE_LINEAR = 0
    DRIVE_ANGULAR = 1
