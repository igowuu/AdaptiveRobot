import math
from typing import Callable, Optional, List
import random

from wpimath.geometry import (
    Pose2d, 
    Pose3d, 
    Rotation2d, 
    Rotation3d, 
    Transform3d, 
    Translation2d,
    Translation3d
)
from wpimath.kinematics import ChassisSpeeds

from wpilib import Timer

from ntcore import NetworkTableInstance

import fuel_sim.constants as const


class FuelSim:
    class Fuel:
        """
        Representation of a single fuel object in the simulation.
        """
        def __init__(self, pos: Translation3d, vel: Optional[Translation3d] = None):
            self.pos = pos
            self.vel = vel if vel is not None else Translation3d()

        def update(
            self, 
            simulate_air_resistance: bool, 
            subticks: int, 
            blue_hub: 'FuelSim.Hub', 
            red_hub: 'FuelSim.Hub', 
            relevant_lines: Optional[List[int]] = None, 
            relevant_trenches: Optional[List[int]] = None
        ) -> None:
            """
            Updates the fuel position and velocity each cycle.
            """
            self.pos = self.pos + self.vel * (const.PERIOD / subticks)
            
            if self.pos.Z() > const.FUEL_RADIUS:
                Fg = const.GRAVITY * const.FUEL_MASS
                Fd = Translation3d()

                if simulate_air_resistance:
                    speed = self.vel.norm()
                    if speed > 1e-6:
                        Fd = self.vel * (-const.DRAG_FORCE_FACTOR * speed)

                accel = (Fg + Fd) / (const.FUEL_MASS)
                self.vel = self.vel + (accel * (const.PERIOD / subticks))
            
            if abs(self.vel.Z()) < 0.05 and self.pos.Z() <= const.FUEL_RADIUS + 0.03:
                self.vel = Translation3d(self.vel.X(), self.vel.Y(), 0)
                self.vel = self.vel * (1 - const.FRICTION * const.PERIOD / subticks)

            self._handle_field_collisions(subticks, blue_hub, red_hub, relevant_lines, relevant_trenches)

        def _handle_xz_line_collision(self, line_start: Translation3d, line_end: Translation3d) -> None:
            """
            Handles collision with a line in the XZ plane.
            """
            if self.pos.Y() < line_start.Y() or self.pos.Y() > line_end.Y():
                return  # not within y range
            
            # Convert into 2D
            start_2d = Translation2d(line_start.X(), line_start.Z())
            end_2d = Translation2d(line_end.X(), line_end.Z())
            pos_2d = Translation2d(self.pos.X(), self.pos.Z())
            line_vec = end_2d - start_2d

            # Get closest point on line using squared distance to avoid sqrt
            line_vec_sq = line_vec.X() * line_vec.X() + line_vec.Y() * line_vec.Y()
            if line_vec_sq < 1e-6:
                return
            
            t = (pos_2d - start_2d).dot(line_vec) / line_vec_sq
            t = max(0, min(1, t))  # Clamp to line segment
            projected = start_2d + (line_vec * t)
            
            dx = pos_2d.X() - projected.X()
            dy = pos_2d.Y() - projected.Y()
            dist_sq = dx*dx + dy*dy
            if dist_sq > const.FUEL_RADIUS_SQUARED:
                return

            # Compute normal without sqrt by normalizing the rotated vector
            line_vec_norm = (line_vec_sq) ** 0.5
            normal = Translation3d(-line_vec.Y(), 0, line_vec.X()) / line_vec_norm

            # Apply collision response
            dist = (dist_sq) ** 0.5
            self.pos = self.pos + (normal * (const.FUEL_RADIUS - dist))
            if self.vel.dot(normal) > 0:
                return
            
            self.vel = self.vel - (normal * ((1 + const.FIELD_COR) * self.vel.dot(normal)))

        def _handle_field_collisions(
            self, 
            subticks: int, 
            blue_hub: 'FuelSim.Hub', 
            red_hub: 'FuelSim.Hub', 
            relevant_lines: Optional[List[int]] = None, 
            relevant_trenches: Optional[List[int]] = None
        ) -> None:
            """
            Handles all field collisions.
            """
            # Floor and bumps - only check relevant line segments for this fuel's position
            if relevant_lines is not None:
                for line_idx in relevant_lines:
                    self._handle_xz_line_collision(
                        const.FIELD_XZ_LINE_STARTS[line_idx],
                        const.FIELD_XZ_LINE_ENDS[line_idx]
                    )
            else:
                # Fallback: check all lines if no spatial partitioning info available
                for i in range(len(const.FIELD_XZ_LINE_STARTS)):
                    self._handle_xz_line_collision(
                        const.FIELD_XZ_LINE_STARTS[i],
                        const.FIELD_XZ_LINE_ENDS[i]
                    )

            # edges - X axis
            if self.pos.X() < const.FUEL_RADIUS and self.vel.X() < 0:
                self.pos = self.pos + Translation3d(const.FUEL_RADIUS - self.pos.X(), 0, 0)
                self.vel = self.vel + Translation3d(-(1 + const.FIELD_COR) * self.vel.X(), 0, 0)
            elif self.pos.X() > const.FIELD_LENGTH - const.FUEL_RADIUS and self.vel.X() > 0:
                self.pos = self.pos + Translation3d(const.FIELD_LENGTH - const.FUEL_RADIUS - self.pos.X(), 0, 0)
                self.vel = self.vel + Translation3d(-(1 + const.FIELD_COR) * self.vel.X(), 0, 0)

            # edges - Y axis
            if self.pos.Y() < const.FUEL_RADIUS and self.vel.Y() < 0:
                self.pos = self.pos + Translation3d(0, const.FUEL_RADIUS - self.pos.Y(), 0)
                self.vel = self.vel + Translation3d(0, -(1 + const.FIELD_COR) * self.vel.Y(), 0)
            elif self.pos.Y() > const.FIELD_WIDTH - const.FUEL_RADIUS and self.vel.Y() > 0:
                self.pos = self.pos + Translation3d(0, const.FIELD_WIDTH - const.FUEL_RADIUS - self.pos.Y(), 0)
                self.vel = self.vel + Translation3d(0, -(1 + const.FIELD_COR) * self.vel.Y(), 0)

            # hubs
            blue_hub.handle_hub_interaction(self, subticks)
            red_hub.handle_hub_interaction(self, subticks)

            self._handle_trench_collisions(relevant_trenches)

        def _handle_hub_collisions(self, hub: 'FuelSim.Hub', subticks: int) -> None:
            """
            Handles collisions with hubs.
            """
            hub.handle_hub_interaction(self, subticks)
            hub.fuel_collide_side(self)

            net_collision = hub.fuel_hit_net(self)
            if net_collision != 0:
                self.pos = self.pos + Translation3d(net_collision, 0, 0)
                self.vel = Translation3d(
                    -self.vel.X() * const.NET_COR, 
                    self.vel.Y() * const.NET_COR, 
                    self.vel.Z()
                )

        def _handle_trench_collisions(self, relevant_trenches: Optional[List[int]] = None) -> None:
            """
            Handles collisions with trenches.
            """
            if relevant_trenches is not None:
                # Only check relevant trenches for this fuel's position
                for trench_idx in relevant_trenches:
                    start, end = FuelSim.TRENCH_RECTANGLES[trench_idx]
                    FuelSim._fuel_collide_rectangle(self, start, end)
            else:
                for start, end in FuelSim.TRENCH_RECTANGLES:
                    FuelSim._fuel_collide_rectangle(self, start, end)

        def add_impulse(self, impulse: Translation3d):
            """
            Adds an impulse to the fuel's velocity.
            """
            self.vel = self.vel + impulse

    @staticmethod
    def _handle_fuel_collision(a: 'FuelSim.Fuel', b: 'FuelSim.Fuel') -> None:
        """
        Handle collision between two fuel objects.
        """
        normal = a.pos - b.pos
        distance = normal.norm()
        if distance == 0:
            normal = Translation3d(1, 0, 0)
            distance = 1
        normal = normal / distance
        impulse = 0.5 * (1 + const.FUEL_COR) * ((b.vel - a.vel).dot(normal))
        intersection = const.FUEL_RADIUS * 2 - distance
        a.pos = a.pos + (normal * (intersection / 2))
        b.pos = b.pos - (normal * (intersection / 2))
        a.add_impulse(normal * impulse)
        b.add_impulse(normal * (-impulse))

    @staticmethod
    def _fuel_collide_rectangle(fuel: 'FuelSim.Fuel', start: Translation3d, end: Translation3d):
        """
        Handle collision between fuel and a rectangular obstacle. Optimized to avoid sqrt calls.
        """
        if fuel.pos.Z() > end.Z() + const.FUEL_RADIUS or fuel.pos.Z() < start.Z() - const.FUEL_RADIUS:
            return  # above rectangle
        
        distance_to_left = start.X() - const.FUEL_RADIUS - fuel.pos.X()
        distance_to_right = fuel.pos.X() - end.X() - const.FUEL_RADIUS
        distance_to_top = fuel.pos.Y() - end.Y() - const.FUEL_RADIUS
        distance_to_bottom = start.Y() - const.FUEL_RADIUS - fuel.pos.Y()

        # not inside rectangle
        if distance_to_left > 0 or distance_to_right > 0 or distance_to_top > 0 or distance_to_bottom > 0:
            return

        # Find minimum distance (which side fuel hits first)
        # Avoid branch prediction issues by finding the maximum negative distance
        max_dist = max(distance_to_left, distance_to_right, distance_to_top, distance_to_bottom)
        
        if max_dist == distance_to_left:
            fuel.pos = fuel.pos + Translation3d(distance_to_left, 0, 0)
            fuel.vel = fuel.vel + Translation3d(-(1 + const.FIELD_COR) * fuel.vel.X(), 0, 0)
        elif max_dist == distance_to_right:
            fuel.pos = fuel.pos + Translation3d(-distance_to_right, 0, 0)
            fuel.vel = fuel.vel + Translation3d(-(1 + const.FIELD_COR) * fuel.vel.X(), 0, 0)
        elif max_dist == distance_to_top:
            fuel.pos = fuel.pos + Translation3d(0, -distance_to_top, 0)
            fuel.vel = fuel.vel + Translation3d(0, -(1 + const.FIELD_COR) * fuel.vel.Y(), 0)
        else:  # distance_to_bottom
            fuel.pos = fuel.pos + Translation3d(0, distance_to_bottom, 0)
            fuel.vel = fuel.vel + Translation3d(0, -(1 + const.FIELD_COR) * fuel.vel.Y(), 0)

    CELL_SIZE = const.CELL_SIZE
    GRID_COLS = const.GRID_COLS
    GRID_ROWS = const.GRID_ROWS
    
    # Cached constants for hot path
    FUEL_COLLISION_RADIUS_SQUARED = (const.FUEL_RADIUS * 2) ** 2
    
    # Static trench rectangles for spatial partitioning
    TRENCH_RECTANGLES = [
        (Translation3d(3.96, const.TRENCH_WIDTH, 0), 
         Translation3d(5.18, const.TRENCH_WIDTH + const.TRENCH_BLOCK_WIDTH, const.TRENCH_HEIGHT)),
        (Translation3d(3.96, const.FIELD_WIDTH - 1.57, 0),
         Translation3d(5.18, const.FIELD_WIDTH - 1.57 + const.TRENCH_BLOCK_WIDTH, const.TRENCH_HEIGHT)),
        (Translation3d(const.FIELD_LENGTH - 5.18, const.TRENCH_WIDTH, 0),
         Translation3d(const.FIELD_LENGTH - 3.96, const.TRENCH_WIDTH + const.TRENCH_BLOCK_WIDTH, const.TRENCH_HEIGHT)),
        (Translation3d(const.FIELD_LENGTH - 5.18, const.FIELD_WIDTH - 1.57, 0),
         Translation3d(const.FIELD_LENGTH - 3.96, const.FIELD_WIDTH - 1.57 + const.TRENCH_BLOCK_WIDTH, const.TRENCH_HEIGHT)),
        (Translation3d(4.61 - const.TRENCH_BAR_WIDTH / 2, 0, const.TRENCH_HEIGHT),
         Translation3d(4.61 + const.TRENCH_BAR_WIDTH / 2, const.TRENCH_WIDTH + const.TRENCH_BLOCK_WIDTH, 
                      const.TRENCH_HEIGHT + const.TRENCH_BAR_HEIGHT)),
        (Translation3d(4.61 - const.TRENCH_BAR_WIDTH / 2, const.FIELD_WIDTH - 1.57, const.TRENCH_HEIGHT),
         Translation3d(4.61 + const.TRENCH_BAR_WIDTH / 2, const.FIELD_WIDTH, const.TRENCH_HEIGHT + const.TRENCH_BAR_HEIGHT)),
        (Translation3d(const.FIELD_LENGTH - 4.61 - const.TRENCH_BAR_WIDTH / 2, 0, const.TRENCH_HEIGHT),
         Translation3d(const.FIELD_LENGTH - 4.61 + const.TRENCH_BAR_WIDTH / 2, const.TRENCH_WIDTH + const.TRENCH_BLOCK_WIDTH,
                      const.TRENCH_HEIGHT + const.TRENCH_BAR_HEIGHT)),
        (Translation3d(const.FIELD_LENGTH - 4.61 - const.TRENCH_BAR_WIDTH / 2, const.FIELD_WIDTH - 1.57, const.TRENCH_HEIGHT),
         Translation3d(const.FIELD_LENGTH - 4.61 + const.TRENCH_BAR_WIDTH / 2, const.FIELD_WIDTH,
                      const.TRENCH_HEIGHT + const.TRENCH_BAR_HEIGHT)),
    ]

    def __init__(self, table_key: str = "/Fuel Simulation") -> None:
        """
        Initializes the FuelSim. Creates a new instance of FuelSim.

        :param table_key: NetworkTable key to log fuel positions to as an array of Translation3d structs.
        """
        # Initialize grid
        self.grid: List[List[List['FuelSim.Fuel']]] = [
            [[] for _ in range(self.GRID_ROWS)] 
            for _ in range(self.GRID_COLS)
        ]
        
        # Pre-compute which line segments are relevant to each grid cell
        self.grid_to_lines: List[List[List[int]]] = [
            [[] for _ in range(self.GRID_ROWS)] 
            for _ in range(self.GRID_COLS)
        ]
        
        # Pre-compute which trench rectangles are relevant to each grid cell
        self.grid_to_trenches: List[List[List[int]]] = [
            [[] for _ in range(self.GRID_ROWS)] 
            for _ in range(self.GRID_COLS)
        ]
        
        # Track dirty cells to avoid clearing all cells every frame
        self.dirty_cells: list[tuple[int, int]] = []
        self.active_cells: List[List['FuelSim.Fuel']] = []
        self.fuels: List['FuelSim.Fuel'] = []
        self.running = False
        self.simulate_air_resistance = False
        self.robot_pose_supplier: Optional[Callable[[], Pose2d]] = None
        self.robot_field_speeds_supplier: Optional[Callable[[], ChassisSpeeds]] = None
        self.robot_width = 0.0  # size along the robot's y axis
        self.robot_length = 0.0  # size along the robot's x axis
        self.bumper_height = 0.0
        self.intakes: List['FuelSim.SimIntake'] = []
        self.subticks = 5
        self.logging_freq_hz = 10
        self.logging_timer = Timer()

        # Create hubs once for the entire simulation
        self.blue_hub = FuelSim.Hub(
            Translation2d(4.61, const.FIELD_WIDTH / 2),
            Translation3d(5.3, const.FIELD_WIDTH / 2, 0.89),
            1
        )
        self.red_hub = FuelSim.Hub(
            Translation2d(const.FIELD_LENGTH - 4.61, const.FIELD_WIDTH / 2),
            Translation3d(const.FIELD_LENGTH - 5.3, const.FIELD_WIDTH / 2, 0.89),
            -1
        )

        self.fuel_publisher = NetworkTableInstance.getDefault() \
            .getStructArrayTopic(table_key + "/Fuels", Translation3d) \
            .publish()
        
        # Pre-compute static collision geometry
        self._precompute_static_collisions()

    def _precompute_static_collisions(self) -> None:
        """
        Pre-compute which static obstacles (line segments and trenches) are in each grid cell.
        """
        # Pre-compute line segments
        for line_idx in range(len(const.FIELD_XZ_LINE_STARTS)):
            start = const.FIELD_XZ_LINE_STARTS[line_idx]
            end = const.FIELD_XZ_LINE_ENDS[line_idx]
            
            # Find bounding box of line segment (with fuel radius buffer)
            min_x = min(start.X(), end.X()) - const.FUEL_RADIUS
            max_x = max(start.X(), end.X()) + const.FUEL_RADIUS
            min_y = min(start.Y(), end.Y())
            max_y = max(start.Y(), end.Y())
            
            # Mark all grid cells this line could affect
            col_min = max(0, int(min_x / const.CELL_SIZE))
            col_max = min(const.GRID_COLS - 1, int(max_x / const.CELL_SIZE))
            row_min = max(0, int(min_y / const.CELL_SIZE))
            row_max = min(const.GRID_ROWS - 1, int(max_y / const.CELL_SIZE))
            
            for col in range(col_min, col_max + 1):
                for row in range(row_min, row_max + 1):
                    self.grid_to_lines[col][row].append(line_idx)
        
        # Pre-compute trench rectangles
        for trench_idx, (start, end) in enumerate(self.TRENCH_RECTANGLES):
            # Find bounding box of rectangle (with fuel radius buffer)
            min_x = start.X() - const.FUEL_RADIUS
            max_x = end.X() + const.FUEL_RADIUS
            min_y = start.Y() - const.FUEL_RADIUS
            max_y = end.Y() + const.FUEL_RADIUS
            
            # Mark all grid cells this trench could affect
            col_min = max(0, int(min_x / const.CELL_SIZE))
            col_max = min(const.GRID_COLS - 1, int(max_x / const.CELL_SIZE))
            row_min = max(0, int(min_y / const.CELL_SIZE))
            row_max = min(const.GRID_ROWS - 1, int(max_y / const.CELL_SIZE))
            
            for col in range(col_min, col_max + 1):
                for row in range(row_min, row_max + 1):
                    self.grid_to_trenches[col][row].append(trench_idx)
    
    def clear_fuel(self) -> None:
        """
        Clear the field of fuel.
        """
        self.fuels.clear()

    def spawn_starting_fuel(self) -> None:
        """
        Spawn fuel in the neutral zone and depots.
        """
        # Center fuel
        center = Translation3d(const.FIELD_LENGTH / 2, const.FIELD_WIDTH / 2, const.FUEL_RADIUS)
        for i in range(15):
            for j in range(6):
                self.fuels.append(FuelSim.Fuel(center + Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0)))
                self.fuels.append(FuelSim.Fuel(center + Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0)))
                self.fuels.append(FuelSim.Fuel(center + Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0)))
                self.fuels.append(FuelSim.Fuel(center + Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0)))

        # Depots
        for i in range(3):
            for j in range(4):
                self.fuels.append(FuelSim.Fuel(Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, const.FUEL_RADIUS)))
                self.fuels.append(FuelSim.Fuel(Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, const.FUEL_RADIUS)))
                self.fuels.append(FuelSim.Fuel(
                    Translation3d(const.FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, const.FUEL_RADIUS)))
                self.fuels.append(FuelSim.Fuel(
                    Translation3d(const.FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, const.FUEL_RADIUS)))

    def log_fuels(self) -> None:
        """
        Adds array of Translation3d's to NetworkTables at tableKey + "/Fuels".
        """
        self.fuel_publisher.set([fuel.pos for fuel in self.fuels])

    def start(self) -> None:
        """
        Starts the simulation. updateSim must still be called every loop.
        """
        self.running = True
        self.logging_timer.restart()

    def stop(self) -> None:
        """
        Pauses the simulation.
        """
        self.running = False
        self.logging_timer.stop()

    def enable_air_resistance(self) -> None:
        """
        Enable accounting for drag force in physics step.
        """
        self.simulate_air_resistance = True

    def set_subticks(self, subticks: int) -> None:
        """
        Sets the number of physics iterations per loop (0.02s).

        :param subticks: physics iteration per loop (default: 5)
        """
        self.subticks = subticks

    def set_logging_frequency(self, logging_freq_hz: float) -> None:
        """
        Sets the frequency to publish fuel translations to NetworkTables.
        Used to improve performance in AdvantageScope.

        :param logging_freq_hz: update frequency in hertz
        """
        self.logging_freq_hz = logging_freq_hz

    def register_robot(
        self, 
        width: float, 
        length: float, 
        bumper_height: float,
        pose_supplier: Callable[[], Pose2d],
        field_speeds_supplier: Callable[[], ChassisSpeeds]
    ) -> None:
        """
        Registers a robot with the fuel simulator.

        :param width: from left to right (y-axis)
        :param length: from front to back (x-axis)
        :param bumper_height: height of robot bumpers
        :param pose_supplier: callable that returns current robot pose
        :param field_speeds_supplier: callable that returns field-relative ChassisSpeeds
        """
        self.robot_pose_supplier = pose_supplier
        self.robot_field_speeds_supplier = field_speeds_supplier
        self.robot_width = width
        self.robot_length = length
        self.bumper_height = bumper_height

    def update_sim(self) -> None:
        """
        Updates simulation. Should be called periodically.
        Will do nothing if sim is not running.
        """
        if not self.running:
            return
        self.step_sim()

    def step_sim(self) -> None:
        """
        Run the simulation forward 1 time step (0.02s).
        """
        for _ in range(self.subticks):
            for fuel in self.fuels:
                # Get relevant static collision geometry for this fuel's position
                col = int(fuel.pos.X() / const.CELL_SIZE)
                row = int(fuel.pos.Y() / const.CELL_SIZE)
                relevant_lines = []
                relevant_trenches = []
                if 0 <= col < const.GRID_COLS and 0 <= row < const.GRID_ROWS:
                    relevant_lines = self.grid_to_lines[col][row]
                    relevant_trenches = self.grid_to_trenches[col][row]
                
                fuel.update(self.simulate_air_resistance, self.subticks, self.blue_hub, self.red_hub, 
                          relevant_lines, relevant_trenches)

        self._handle_fuel_collisions(self.fuels)

        if self.robot_pose_supplier is not None:
            self._handle_robot_collisions(self.fuels)
            
            self._handle_intakes(self.fuels)

        if self.logging_timer.advanceIfElapsed(1.0 / self.logging_freq_hz):
            self.log_fuels()

    def spawn_fuel(self, pos: Translation3d, vel: Translation3d) -> None:
        """
        Add a fuel onto the field.

        :param pos: Position to spawn at
        :param vel: Initial velocity vector
        """
        self.fuels.append(FuelSim.Fuel(pos, vel))

    def launch_fuel(self, launch_velocity: float, hood_angle: float, shooter_yaw: float, launch_height: float) -> None:
        """
        Spawn a fuel onto the field with specified launch velocity and angles.
        Accounts for robot movement.

        :param launch_velocity: Initial launch velocity in m/s
        :param hood_angle: Hood angle where 0 is horizontal and π/2 (pi/2) is straight up (in radians)
        :param shooter_yaw: Robot-relative shooter yaw (in radians)
        :param launch_height: Height of the fuel to launch at in meters

        :raises RuntimeError: if robot is not registered
        """
        if self.robot_pose_supplier is None or self.robot_field_speeds_supplier is None:
            raise RuntimeError("Robot must be registered before launching fuel.")

        robot_pose = self.robot_pose_supplier()
        launch_pose = Pose3d(robot_pose).transformBy(
            Transform3d(Translation3d(0, 0, launch_height), Rotation3d()))
        field_speeds = self.robot_field_speeds_supplier()

        horizontal_vel = math.cos(hood_angle) * launch_velocity
        vertical_vel = math.sin(hood_angle) * launch_velocity
        x_vel = horizontal_vel * math.cos(shooter_yaw + launch_pose.rotation().Z())
        y_vel = horizontal_vel * math.sin(shooter_yaw + launch_pose.rotation().Z())

        x_vel += field_speeds.vx
        y_vel += field_speeds.vy

        self.spawn_fuel(launch_pose.translation(), Translation3d(x_vel, y_vel, vertical_vel))

    def _handle_fuel_collisions(self, fuels: List['FuelSim.Fuel']) -> None:
        """
        Handle collisions between fuel objects.
        """
        # Clear only dirty cells from previous frame
        for col, row in self.dirty_cells:
            self.grid[col][row].clear()
        self.dirty_cells.clear()
        self.active_cells.clear()

        # Populate grid and track dirty cells
        for fuel in fuels:
            col = int(fuel.pos.X() / const.CELL_SIZE)
            row = int(fuel.pos.Y() / const.CELL_SIZE)

            if 0 <= col < const.GRID_COLS and 0 <= row < const.GRID_ROWS:
                self.grid[col][row].append(fuel)
                self.dirty_cells.append((col, row))

                if len(self.grid[col][row]) == 1:
                    self.active_cells.append(self.grid[col][row])

        collision_radius_sq = self.FUEL_COLLISION_RADIUS_SQUARED
        for fuel in fuels:
            col = int(fuel.pos.X() / const.CELL_SIZE)
            row = int(fuel.pos.Y() / const.CELL_SIZE)

            # Check 3x3 neighbor cells
            for i in range(col - 1, col + 2):
                for j in range(row - 1, row + 2):
                    if 0 <= i < const.GRID_COLS and 0 <= j < const.GRID_ROWS:
                        for other in self.grid[i][j]:
                            if fuel is not other and id(fuel) < id(other):
                                # Use squared distance to avoid sqrt
                                dx = fuel.pos.X() - other.pos.X()
                                dy = fuel.pos.Y() - other.pos.Y()
                                dz = fuel.pos.Z() - other.pos.Z()
                                dist_sq = dx*dx + dy*dy + dz*dz
                                if dist_sq < collision_radius_sq:
                                    FuelSim._handle_fuel_collision(fuel, other)

    def _handle_robot_collision(self, fuel: 'FuelSim.Fuel', robot: Pose2d, robot_vel: Translation2d) -> None:
        """
        Handles collision between fuel and robot.
        """
        relative_pos = Pose2d(fuel.pos.toTranslation2d(), Rotation2d()) \
            .relativeTo(robot) \
            .translation()

        if fuel.pos.Z() > self.bumper_height:
            return  # above bumpers
        
        distance_to_bottom = -const.FUEL_RADIUS - self.robot_length / 2 - relative_pos.X()
        distance_to_top = -const.FUEL_RADIUS + self.robot_length / 2 + relative_pos.X()
        distance_to_right = -const.FUEL_RADIUS - self.robot_width / 2 - relative_pos.Y()
        distance_to_left = -const.FUEL_RADIUS + self.robot_width / 2 + relative_pos.Y()

        # not inside robot
        if distance_to_bottom > 0 or distance_to_top > 0 or distance_to_right > 0 or distance_to_left > 0:
            return

        # find minimum distance to side and send corresponding collision response
        if (distance_to_bottom >= distance_to_top
                and distance_to_bottom >= distance_to_right
                and distance_to_bottom >= distance_to_left):
            pos_offset = Translation2d(distance_to_bottom, 0)
        elif (distance_to_top >= distance_to_bottom
                and distance_to_top >= distance_to_right
                and distance_to_top >= distance_to_left):
            pos_offset = Translation2d(-distance_to_top, 0)
        elif (distance_to_right >= distance_to_bottom
                and distance_to_right >= distance_to_top
                and distance_to_right >= distance_to_left):
            pos_offset = Translation2d(0, distance_to_right)
        else:
            pos_offset = Translation2d(0, -distance_to_left)

        pos_offset = pos_offset.rotateBy(robot.rotation())
        fuel.pos = fuel.pos + Translation3d(pos_offset)
        normal = pos_offset / pos_offset.norm()
        
        if fuel.vel.toTranslation2d().dot(normal) < 0:
            fuel.add_impulse(
                Translation3d(normal * (-fuel.vel.toTranslation2d().dot(normal) * (1 + const.ROBOT_COR))))
        if robot_vel.dot(normal) > 0:
            fuel.add_impulse(Translation3d(normal * robot_vel.dot(normal)))

    def _handle_robot_collisions(self, fuels: List['FuelSim.Fuel']) -> None:
        """
        Handles all collisions with the robot. Only checks fuels near the robot.
        """
        if self.robot_pose_supplier is not None:
            robot = self.robot_pose_supplier()
        else:
            raise RuntimeError("ERROR: No robot pose supplier given.")

        if self.robot_field_speeds_supplier is not None:
            speeds = self.robot_field_speeds_supplier()
        else:
            raise RuntimeError("ERROR: No field speeds supplier given.")

        robot_vel = Translation2d(speeds.vx, speeds.vy)
        robot_pos_2d = robot.translation()
        
        # Pre-compute robot interaction radius to early-exit fuels far away
        robot_interaction_radius = (self.robot_width / 2 + self.robot_length / 2) * 1.5 + const.FUEL_RADIUS

        for fuel in fuels:
            # Early exit: skip fuels far from robot (squared distance to avoid sqrt)
            dx = fuel.pos.X() - robot_pos_2d.X()
            dy = fuel.pos.Y() - robot_pos_2d.Y()
            dist_sq = dx*dx + dy*dy
            if dist_sq > robot_interaction_radius * robot_interaction_radius:
                continue
            
            self._handle_robot_collision(fuel, robot, robot_vel)

    def _handle_intakes(self, fuels: List['FuelSim.Fuel']) -> None:
        """
        Handles intakes removing fuel from the field.
        """
        if self.robot_pose_supplier is not None:
            robot = self.robot_pose_supplier()
        else:
            raise RuntimeError("ERROR: No robot pose supplier given.")

        # Use swap-and-pop pattern for efficient removal
        i = 0
        while i < len(fuels):
            removed = False
            for intake in self.intakes:
                if intake.should_intake(fuels[i], robot):
                    # Swap with last element and pop
                    fuels[i] = fuels[-1]
                    fuels.pop()
                    removed = True
                    break
            if not removed:
                i += 1

    def register_intake(
        self, 
        x_min: float, 
        x_max: float, 
        y_min: float, 
        y_max: float,
        able_to_intake: Optional[Callable[[], bool]] = None,
        intake_callback: Optional[Callable[[], None]] = None
    ) -> None:
        """
        Registers an intake with the fuel simulator.

        :param x_min: Minimum x position for the bounding box
        :param x_max: Maximum x position for the bounding box
        :param y_min: Minimum y position for the bounding box
        :param y_max: Maximum y position for the bounding box
        :param able_to_intake: Callable that returns whether intake is active (default: always True)
        :param intake_callback: Callback to call when fuel is intaken (default: no-op)
        """
        if able_to_intake is None:
            able_to_intake = lambda: True
        if intake_callback is None:
            intake_callback = lambda: None
        self.intakes.append(FuelSim.SimIntake(x_min, x_max, y_min, y_max, able_to_intake, intake_callback))

    def get_blue_score(self) -> int:
        """
        Returns the current blue alliance score.
        """
        return self.blue_hub.get_score()

    def get_red_score(self) -> int:
        """
        Returns the current red alliance score.
        """
        return self.red_hub.get_score()

    def reset_scores(self) -> None:
        """
        Resets both hub scores to 0.
        """
        self.blue_hub.reset_score()
        self.red_hub.reset_score()

    class Hub:
        """
        Represents a hub on the field.
        """
        ENTRY_HEIGHT = const.HUB_ENTRY_HEIGHT
        ENTRY_RADIUS = const.HUB_ENTRY_RADIUS
        SIDE = const.HUB_SIDE
        NET_HEIGHT_MAX = const.HUB_NET_HEIGHT_MAX
        NET_HEIGHT_MIN = const.HUB_NET_HEIGHT_MIN
        NET_OFFSET = const.HUB_NET_OFFSET
        NET_WIDTH = const.HUB_NET_WIDTH

        def __init__(self, center: Translation2d, exit_pos: Translation3d, exit_vel_x_mult: int):
            self.center = center
            self.exit = exit_pos
            self.exit_vel_x_mult = exit_vel_x_mult
            self.score = 0

        def handle_hub_interaction(self, fuel: 'FuelSim.Fuel', subticks: int) -> None:
            """
            Handles interactions with the hub.
            """
            if self.did_fuel_score(fuel, subticks):
                fuel.pos = self.exit
                fuel.vel = self.get_dispersal_velocity()
                self.score += 1

        def did_fuel_score(self, fuel: 'FuelSim.Fuel', subticks: int) -> bool:
            """
            Checks if fuel scored in this hub.
            """
            return (fuel.pos.toTranslation2d().distance(self.center) <= const.HUB_ENTRY_RADIUS
                    and fuel.pos.Z() <= const.HUB_ENTRY_HEIGHT
                    and (fuel.pos - (fuel.vel * (const.PERIOD / subticks))).Z() > const.HUB_ENTRY_HEIGHT)

        def get_dispersal_velocity(self) -> Translation3d:
            """
            Returns a random dispersal velocity for scored fuel.
            """
            return Translation3d(
                self.exit_vel_x_mult * (random.random() + 0.1) * 1.5,
                random.random() * 2 - 1,
                0)

        def reset_score(self) -> None:
            """
            Resets this hub's score to 0.
            """
            self.score = 0

        def get_score(self) -> int:
            """
            Returns the current count of fuel scored in this hub.
            """
            return self.score

        def fuel_collide_side(self, fuel: 'FuelSim.Fuel') -> None:
            """
            Handles collision with the side of the hub.
            """
            FuelSim._fuel_collide_rectangle(
                fuel,
                Translation3d(self.center.X() - const.HUB_SIDE / 2,
                            self.center.Y() - const.HUB_SIDE / 2, 0),
                Translation3d(self.center.X() + const.HUB_SIDE / 2,
                            self.center.Y() + const.HUB_SIDE / 2,
                            const.HUB_ENTRY_HEIGHT - 0.1))

        def fuel_hit_net(self, fuel: 'FuelSim.Fuel') -> float:
            """
            Check if fuel hit the net and return collision offset.
            """
            if fuel.pos.Z() > const.HUB_NET_HEIGHT_MAX or fuel.pos.Z() < const.HUB_NET_HEIGHT_MIN:
                return 0
            if (fuel.pos.Y() > self.center.Y() + const.HUB_NET_WIDTH / 2
                    or fuel.pos.Y() < self.center.Y() - const.HUB_NET_WIDTH / 2):
                return 0
            
            if fuel.pos.X() > self.center.X() + const.HUB_NET_OFFSET * self.exit_vel_x_mult:
                return max(0, self.center.X() + const.HUB_NET_OFFSET * self.exit_vel_x_mult - (fuel.pos.X() - const.FUEL_RADIUS))
            else:
                return min(0, self.center.X() + const.HUB_NET_OFFSET * self.exit_vel_x_mult - (fuel.pos.X() + const.FUEL_RADIUS))

    class SimIntake:
        """
        Represents an intake mechanism in the simulation.
        """
        def __init__(
            self, 
            x_min: float, 
            x_max: float, 
            y_min: float, 
            y_max: float,
            able_to_intake: Callable[[], bool], 
            intake_callback: Callable[[], None]
        ) -> None:
            self.x_min = x_min
            self.x_max = x_max
            self.y_min = y_min
            self.y_max = y_max
            self.able_to_intake = able_to_intake
            self.callback = intake_callback

        def should_intake(self, fuel: 'FuelSim.Fuel', robot_pose: Pose2d) -> bool:
            """
            Checks if this intake should grab the fuel.
            """
            if not self.able_to_intake() or fuel.pos.Z() > 0.3:  # Assuming default bumper height
                return False

            fuel_relative_pos = Pose2d(fuel.pos.toTranslation2d(), Rotation2d()) \
                .relativeTo(robot_pose) \
                .translation()

            result = (self.x_min <= fuel_relative_pos.X() <= self.x_max
                    and self.y_min <= fuel_relative_pos.Y() <= self.y_max)
            if result:
                self.callback()
            return result
