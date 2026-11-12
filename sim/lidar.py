"""Ray-casting lidar sensor for rectangle-based simulation walls."""

from __future__ import annotations

import math
from dataclasses import dataclass
from tkinter import Canvas
from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:
    from .robot import Robot


Rect = tuple[float, float, float, float]
WALL_GROUP_COLORS = (
    "#00a6fb",
    "#ff6b35",
    "#70e000",
    "#c77dff",
    "#ffd60a",
    "#00f5d4",
    "#f72585",
    "#8ac926",
)


@dataclass(frozen=True)
class LidarReading:
    """One range measurement; angle is relative to the robot's forward axis."""

    angle_degrees: float
    distance: float
    hit: bool


class Lidar:
    """A 2D lidar that measures the nearest wall along each ray."""

    def __init__(
        self,
        robot: Robot,
        walls: Sequence[Rect],
        ray_count: int = 180,
        max_range: float = 350.0,
    ) -> None:
        if ray_count <= 0:
            raise ValueError("ray_count must be greater than zero")
        if max_range <= 0:
            raise ValueError("max_range must be greater than zero")

        self.robot = robot
        self.walls = walls
        self.ray_count = ray_count
        self.max_range = max_range
        self._readings: tuple[LidarReading, ...] = ()

    def scan(self) -> tuple[LidarReading, ...]:
        """Return the latest full scan, including max-range no-hit readings."""
        readings: list[LidarReading] = []
        for index in range(self.ray_count):
            relative_angle = index * 360.0 / self.ray_count
            angle = self.robot.angle + math.radians(relative_angle)
            direction_x = math.cos(angle)
            direction_y = math.sin(angle)

            nearest_distance = self.max_range
            hit = False
            for wall in self.walls:
                distance, wall_hit = self._ray_rect_distance(
                    self.robot.x,
                    self.robot.y,
                    direction_x,
                    direction_y,
                    wall,
                )
                if wall_hit and distance <= nearest_distance:
                    nearest_distance = distance
                    hit = True

            readings.append(LidarReading(relative_angle, nearest_distance, hit))

        self._readings = tuple(readings)
        return self._readings

    def get_readings(self) -> tuple[LidarReading, ...]:
        """Return the most recent scan; call ``scan`` to refresh it."""
        return self._readings

    def draw(
        self,
        canvas: Canvas,
        wall_group_ids: Sequence[int | None] | None = None,
    ) -> None:
        """Draw the scan, coloring hit dots by their recognized wall group."""
        if wall_group_ids is not None and len(wall_group_ids) != len(self._readings):
            raise ValueError("wall_group_ids must match the number of lidar readings")

        for index, reading in enumerate(self._readings):
            angle = self.robot.angle + math.radians(reading.angle_degrees)
            end_x = self.robot.x + math.cos(angle) * reading.distance
            end_y = self.robot.y + math.sin(angle) * reading.distance
            color = "#d94841" if reading.hit else "#b8c5a5"
            canvas.create_line(
                self.robot.x,
                self.robot.y,
                end_x,
                end_y,
                fill=color,
                width=1,
            )
            if reading.hit:
                group_id = None if wall_group_ids is None else wall_group_ids[index]
                dot_color = (
                    WALL_GROUP_COLORS[group_id % len(WALL_GROUP_COLORS)]
                    if group_id is not None
                    else color
                )
                canvas.create_oval(
                    end_x - 2,
                    end_y - 2,
                    end_x + 2,
                    end_y + 2,
                    fill=dot_color,
                    outline=dot_color,
                )

    def _ray_rect_distance(
        self,
        origin_x: float,
        origin_y: float,
        direction_x: float,
        direction_y: float,
        rect: Rect,
    ) -> tuple[float, bool]:
        """Find the first ray intersection using the rectangle slab method."""
        x, y, width, height = rect
        entry = 0.0
        exit = self.max_range

        for origin, direction, lower, upper in (
            (origin_x, direction_x, x, x + width),
            (origin_y, direction_y, y, y + height),
        ):
            if abs(direction) < 1e-12:
                if origin < lower or origin > upper:
                    return self.max_range, False
                continue

            first = (lower - origin) / direction
            second = (upper - origin) / direction
            if first > second:
                first, second = second, first
            entry = max(entry, first)
            exit = min(exit, second)
            if entry > exit:
                return self.max_range, False

        if exit < 0.0 or entry > self.max_range:
            return self.max_range, False
        return entry, True