"""A small, keyboard-driven differential-drive-style robot model."""

from __future__ import annotations

import math
from tkinter import Canvas
from typing import TYPE_CHECKING, AbstractSet, Sequence

if TYPE_CHECKING:
    from .lidar import LidarReading

Rect = tuple[float, float, float, float]


class Robot:
    """Robot pose and motion in canvas coordinates (positive y points down)."""

    def __init__(
        self,
        x: float,
        y: float,
        radius: float = 18.0,
        speed: float = 180.0,
        turn_speed: float = 2.6,
    ) -> None:
        self.x = x
        self.y = y
        self.angle = 0.0
        self.radius = radius
        self.speed = speed
        self.turn_speed = turn_speed
        self.lidar_readings: tuple[LidarReading, ...] = ()
        self.lidar_group_ids: tuple[int | None, ...] = ()

    def update(
        self,
        dt: float,
        pressed_keys: AbstractSet[str],
        walls: Sequence[Rect],
    ) -> None:
        """Move for ``dt`` seconds; W/S drive and A/D turn."""
        turn_direction = float("d" in pressed_keys) - float("a" in pressed_keys)
        self.angle = (self.angle + turn_direction * self.turn_speed * dt) % math.tau

        drive_direction = float("w" in pressed_keys) - float("s" in pressed_keys)
        if drive_direction == 0:
            return

        distance = drive_direction * self.speed * dt
        dx = math.cos(self.angle) * distance
        dy = math.sin(self.angle) * distance

        # Sweep each axis to prevent a large time step from jumping through a wall.
        self.x = self._sweep_x(dx, walls)
        self.y = self._sweep_y(dy, walls)

    # Lidar analysis: add obstacle detection and decision logic here.
    def analyze_lidar(
        self, readings: Sequence[LidarReading]
    ) -> tuple[int | None, ...]:
        """Group neighboring hit points and return a group ID for each reading."""
        self.lidar_readings = tuple(readings)

        coordinates: list[tuple[float, float] | None] = []
        for reading in self.lidar_readings:
            if reading.hit:
                angle = self.angle + math.radians(reading.angle_degrees)
                coordinates.append(
                    (
                        self.x + math.cos(angle) * reading.distance,
                        self.y + math.sin(angle) * reading.distance,
                    )
                )
            else:
                coordinates.append(None)

        groups: list[list[int]] = []
        for reading_index, coordinate in enumerate(coordinates):
            if coordinate is None:
                continue

            if not groups:
                groups.append([])
            current_group = groups[-1]
            if current_group:
                previous = coordinates[current_group[-1]]
                assert previous is not None
                split_group = math.dist(coordinate, previous) > 150

                if len(current_group) >= 2 and not split_group:
                    before_previous = coordinates[current_group[-2]]
                    assert before_previous is not None
                    segment_angle = math.atan2(
                        coordinate[1] - previous[1],
                        coordinate[0] - previous[0],
                    )
                    previous_angle = math.atan2(
                        previous[1] - before_previous[1],
                        previous[0] - before_previous[0],
                    )
                    angle_change = abs(
                        (segment_angle - previous_angle + math.pi) % math.tau
                        - math.pi
                    )
                    split_group = angle_change > math.radians(10)

                if split_group:
                    groups.append([])

            groups[-1].append(reading_index)

        if groups[-1] and groups[0]:
            first = coordinates[groups[0][0]]
            last = coordinates[groups[-1][-1]]
            assert first is not None and last is not None
            if math.dist(first, last) <= 150:
                if len(groups[-1]) >= 2:
                    before_last = coordinates[groups[-1][-2]]
                    assert before_last is not None
                    segment_angle = math.atan2(
                        last[1] - before_last[1],
                        last[0] - before_last[0],
                    )
                    previous_angle = math.atan2(
                        first[1] - last[1],
                        first[0] - last[0],
                    )
                    angle_change = abs(
                        (segment_angle - previous_angle + math.pi) % math.tau
                        - math.pi
                    )
                    if angle_change > math.radians(10):
                        groups.append([])

                groups[-1].extend(groups[0])
                groups.pop(0)

        group_ids: list[int | None] = [None] * len(self.lidar_readings)
        for group_id, group in enumerate(groups):
            for reading_index in group:
                group_ids[reading_index] = group_id

        self.lidar_group_ids = tuple(group_ids)
        return self.lidar_group_ids

    def _sweep_x(self, distance: float, walls: Sequence[Rect]) -> float:
        target_x = self.x + distance
        for wall_x, wall_y, wall_width, wall_height in walls:
            vertical_gap = max(wall_y - self.y, 0.0, self.y - (wall_y + wall_height))
            if vertical_gap > self.radius:
                continue
            reach = math.sqrt(self.radius * self.radius - vertical_gap * vertical_gap)
            wall_left = wall_x - reach
            wall_right = wall_x + wall_width + reach
            if distance > 0 and self.x <= wall_left <= target_x:
                target_x = min(target_x, wall_left)
            elif distance < 0 and target_x <= wall_right <= self.x:
                target_x = max(target_x, wall_right)
        return target_x

    def _sweep_y(self, distance: float, walls: Sequence[Rect]) -> float:
        target_y = self.y + distance
        for wall_x, wall_y, wall_width, wall_height in walls:
            horizontal_gap = max(wall_x - self.x, 0.0, self.x - (wall_x + wall_width))
            if horizontal_gap > self.radius:
                continue
            reach = math.sqrt(self.radius * self.radius - horizontal_gap * horizontal_gap)
            wall_top = wall_y - reach
            wall_bottom = wall_y + wall_height + reach
            if distance > 0 and self.y <= wall_top <= target_y:
                target_y = min(target_y, wall_top)
            elif distance < 0 and target_y <= wall_bottom <= self.y:
                target_y = max(target_y, wall_bottom)
        return target_y

    def draw(self, canvas: Canvas) -> None:
        """Draw the robot body and its forward direction on a Tk canvas."""
        radius = self.radius
        canvas.create_oval(
            self.x - radius,
            self.y - radius,
            self.x + radius,
            self.y + radius,
            fill="#2878d0",
            outline="#123d70",
            width=2,
        )

        nose_x = self.x + math.cos(self.angle) * radius
        nose_y = self.y + math.sin(self.angle) * radius
        left_angle = self.angle + math.radians(145)
        right_angle = self.angle - math.radians(145)
        canvas.create_polygon(
            nose_x,
            nose_y,
            self.x + math.cos(left_angle) * radius * 0.65,
            self.y + math.sin(left_angle) * radius * 0.65,
            self.x + math.cos(right_angle) * radius * 0.65,
            self.y + math.sin(right_angle) * radius * 0.65,
            fill="#ffffff",
            outline="#ffffff",
        )
        canvas.create_line(
            self.x,
            self.y,
            nose_x,
            nose_y,
            fill="#ffffff",
            width=2,
        )