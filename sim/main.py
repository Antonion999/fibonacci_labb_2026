"""Run the field simulation with ``python -m sim.main`` from the repository root."""

from __future__ import annotations

import time
import tkinter as tk

if __package__:
    from .lidar import Lidar, Rect
    from .robot import Robot
else:
    from lidar import Lidar, Rect
    from robot import Robot


FIELD_WIDTH = 1000
FIELD_HEIGHT = 700
BACKGROUND = "#f4f1e8"

# Rectangles are (x, y, width, height), in pixels. The outer boundary and
# central island create a rectangular circuit with four 90-degree corners.
WALLS: tuple[Rect, ...] = (
    (0, 0, FIELD_WIDTH, 18),
    (0, FIELD_HEIGHT - 18, FIELD_WIDTH, 18),
    (0, 0, 18, FIELD_HEIGHT),
    (FIELD_WIDTH - 18, 0, 18, FIELD_HEIGHT),
    (250, 180, 500, 340),
)
FINISH_LINE_X = 140


class Simulation:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Robot + Lidar Field Simulation")
        self.root.resizable(False, False)

        self.canvas = tk.Canvas(
            self.root,
            width=FIELD_WIDTH,
            height=FIELD_HEIGHT,
            background=BACKGROUND,
            highlightthickness=0,
        )
        self.canvas.pack()
        self.status = tk.StringVar()
        tk.Label(
            self.root,
            textvariable=self.status,
            anchor="w",
            padx=10,
            pady=6,
        ).pack(fill="x")

        self.robot = Robot(x=90, y=90)
        self.lidar = Lidar(self.robot, WALLS, ray_count=180, max_range=1000)
        self.pressed_keys: set[str] = set()
        self.last_frame = time.perf_counter()

        self.root.bind_all("<KeyPress>", self._key_down)
        self.root.bind_all("<KeyRelease>", self._key_up)
        self.root.bind_all("<Escape>", self._quit)
        self.canvas.focus_set()

    def _key_down(self, event: tk.Event) -> None:
        self.pressed_keys.add(event.keysym.lower())

    def _key_up(self, event: tk.Event) -> None:
        self.pressed_keys.discard(event.keysym.lower())

    def _quit(self, event: tk.Event) -> None:
        if event.keysym == "Escape":
            self.root.destroy()

    def _draw_field(self) -> None:
        self.canvas.delete("all")

        for x, y, width, height in WALLS:
            self.canvas.create_rectangle(
                x,
                y,
                x + width,
                y + height,
                fill="#343a40",
                outline="#202428",
            )

        for row in range(16):
            color = "#ffffff" if row % 2 == 0 else "#202428"
            self.canvas.create_rectangle(
                FINISH_LINE_X,
                20 + row * 10,
                FINISH_LINE_X + 10,
                30 + row * 10,
                fill=color,
                outline=color,
            )
            other_color = "#202428" if row % 2 == 0 else "#ffffff"
            self.canvas.create_rectangle(
                FINISH_LINE_X + 10,
                20 + row * 10,
                FINISH_LINE_X + 20,
                30 + row * 10,
                fill=other_color,
                outline=other_color,
            )

        self.lidar.draw(self.canvas, self.robot.lidar_group_ids)
        self.robot.draw(self.canvas)
        hit_count = sum(reading.hit for reading in self.lidar.get_readings())
        self.status.set(
            f"W/S: drive   A/D: turn   Esc: quit"
            f"     |     Lidar: {len(self.lidar.get_readings())} rays,"
            f" {hit_count} hits"
        )

    def _tick(self) -> None:
        now = time.perf_counter()
        dt = min(now - self.last_frame, 0.05)
        self.last_frame = now

        self.robot.update(dt, self.pressed_keys, WALLS)
        readings = self.lidar.scan()
        self.robot.analyze_lidar(readings)
        self._draw_field()
        self.root.after(16, self._tick)

    def run(self) -> None:
        self._tick()
        self.root.mainloop()


def main() -> None:
    Simulation().run()


if __name__ == "__main__":
    main()