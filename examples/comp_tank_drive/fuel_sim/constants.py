import math

from wpimath.geometry import Translation3d


PERIOD = 0.02  # sec
GRAVITY = Translation3d(0, 0, -9.81)  # m/s^2
# Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
AIR_DENSITY = 1.2041  # kg/m^3
FIELD_COR = math.sqrt(22 / 51.5)  # coefficient of restitution with the field
FUEL_COR = 0.5  # coefficient of restitution with another fuel
NET_COR = 0.2  # coefficient of restitution with the net
ROBOT_COR = 0.1  # coefficient of restitution with a robot
FUEL_RADIUS = 0.075
FIELD_LENGTH = 16.51
FIELD_WIDTH = 8.04
TRENCH_WIDTH = 1.265
TRENCH_BLOCK_WIDTH = 0.305
TRENCH_HEIGHT = 0.565
TRENCH_BAR_HEIGHT = 0.102
TRENCH_BAR_WIDTH = 0.152
FRICTION = 0.1  # proportion of horizontal vel to lose per sec while on ground
FUEL_MASS = 0.448 * 0.45392  # kgs
FUEL_CROSS_AREA = math.pi * FUEL_RADIUS * FUEL_RADIUS
# Drag coefficient of smooth sphere: https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
DRAG_COF = 0.47  # dimensionless
DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA


# Grid-based collision detection parameters
CELL_SIZE = 0.25  # meters
GRID_COLS = math.ceil(FIELD_LENGTH / CELL_SIZE)  # 67
GRID_ROWS = math.ceil(FIELD_WIDTH / CELL_SIZE)  # 33

# Hub constants
HUB_ENTRY_HEIGHT = 1.83
HUB_ENTRY_RADIUS = 0.56
HUB_SIDE = 1.2
HUB_NET_HEIGHT_MAX = 3.057
HUB_NET_HEIGHT_MIN = 1.5
HUB_NET_OFFSET = HUB_SIDE / 2 + 0.261
HUB_NET_WIDTH = 1.484

# Pre-computed squared constants for performance (avoid repeated multiplications in hot path)
FUEL_RADIUS_SQUARED = FUEL_RADIUS * FUEL_RADIUS
HUB_ENTRY_RADIUS_SQUARED = HUB_ENTRY_RADIUS * HUB_ENTRY_RADIUS

FIELD_XZ_LINE_STARTS = [
    Translation3d(0, 0, 0),
    Translation3d(3.96, 1.57, 0),
    Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
    Translation3d(4.61, 1.57, 0.165),
    Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
    Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
    Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
    Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
    Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
]

FIELD_XZ_LINE_ENDS = [
    Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
    Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
    Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
    Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
    Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
    Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
    Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    Translation3d(
        4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    Translation3d(FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
]
