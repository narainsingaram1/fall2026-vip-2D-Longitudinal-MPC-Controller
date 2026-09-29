"""
2D Top-Down Animated Visualizer for 4-Way Intersection Dynamics
"""
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter
from typing import List


class IntersectionAnimator2D:
    """
    Renders an animated bird's-eye view of the all-way stop intersection.
    Saves an animated GIF suitable for presentations and weekly VIP reports.
    """
    def __init__(self, vehicles: List, dt: float = 0.20, fps: int = 10):
        self.vehicles = vehicles
        self.dt = dt
        self.fps = fps

        # Spatial dimensions [m]
        self.box_half = 6.0     # Intersection box is [-6, 6] x [-6, 6] (12m width)
        self.road_half_width = 3.5  # Single lane per direction
        self.veh_len = 4.2
        self.veh_wid = 2.0

        # State colors
        self.state_colors = {
            "APPROACH": "#f39c12",  # Amber
            "WAIT": "#e74c3c",      # Red
            "GO": "#2ecc71",        # Emerald Green
            "DONE": "#7f8c8d"       # Slate Gray
        }

    def _get_vehicle_xy(self, approach: str, p: float):
        """Map 1D longitudinal coordinate p to 2D (x, y) and heading angle."""
        if approach == "W":
            x = -self.box_half + p
            y = -self.road_half_width / 2.0
            heading = 0.0  # Heading East (+x)
        elif approach == "E":
            x = self.box_half - p
            y = self.road_half_width / 2.0
            heading = 180.0  # Heading West (-x)
        elif approach == "S":
            x = self.road_half_width / 2.0
            y = -self.box_half + p
            heading = 90.0  # Heading North (+y)
        elif approach == "N":
            x = -self.road_half_width / 2.0
            y = self.box_half - p
            heading = 270.0  # Heading South (-y)
        else:
            x, y, heading = 0.0, 0.0, 0.0
        return x, y, heading

    def generate_gif(self, output_path: str = "intersection_animation.gif", max_time: float = 30.0):
        """Build and save animated GIF."""
        fig, ax = plt.subplots(figsize=(8, 8), dpi=120)

        # Time sequence
        all_times = sorted(list(set(t for v in self.vehicles for t in v.time_history if t <= max_time)))
        # Subsample frames to keep GIF compact and responsive
        step_stride = max(1, int(round((1.0 / self.fps) / self.dt)))
        frame_times = all_times[::step_stride]

        def draw_background():
            ax.clear()
            ax.set_facecolor("#2c3e50")

            # Road corridors
            view_limit = 35.0
            # Horizontal road
            ax.add_patch(patches.Rectangle(
                (-view_limit, -self.road_half_width),
                2 * view_limit, 2 * self.road_half_width,
                color="#34495e", zorder=1
            ))
            # Vertical road
            ax.add_patch(patches.Rectangle(
                (-self.road_half_width, -view_limit),
                2 * self.road_half_width, 2 * view_limit,
                color="#34495e", zorder=1
            ))

            # Intersection conflict zone box
            ax.add_patch(patches.Rectangle(
                (-self.box_half, -self.box_half),
                2 * self.box_half, 2 * self.box_half,
                color="#1abc9c", alpha=0.15, linestyle="--", linewidth=1.5, zorder=2
            ))

            # Stop lines
            lw = 3.0
            # West stop line
            ax.plot([-self.box_half, -self.box_half], [-self.road_half_width, 0], color="white", linewidth=lw, zorder=3)
            # East stop line
            ax.plot([self.box_half, self.box_half], [0, self.road_half_width], color="white", linewidth=lw, zorder=3)
            # South stop line
            ax.plot([0, self.road_half_width], [-self.box_half, -self.box_half], color="white", linewidth=lw, zorder=3)
            # North stop line
            ax.plot([-self.road_half_width, 0], [self.box_half, self.box_half], color="white", linewidth=lw, zorder=3)

            # Center markings
            ax.plot([-view_limit, -self.box_half], [0, 0], color="#f1c40f", linestyle="--", linewidth=1.2, zorder=2)
            ax.plot([self.box_half, view_limit], [0, 0], color="#f1c40f", linestyle="--", linewidth=1.2, zorder=2)
            ax.plot([0, 0], [-view_limit, -self.box_half], color="#f1c40f", linestyle="--", linewidth=1.2, zorder=2)
            ax.plot([0, 0], [self.box_half, view_limit], color="#f1c40f", linestyle="--", linewidth=1.2, zorder=2)

            ax.set_xlim(-view_limit, view_limit)
            ax.set_ylim(-view_limit, view_limit)
            ax.set_aspect("equal")
            ax.axis("off")

        def update(frame_idx):
            draw_background()
            t = frame_times[frame_idx]

            # Title & Clock
            ax.text(
                0.0, 31.0,
                f"CAV All-Way Stop (FIFO MPC)\nSimulation Time: {t:04.1f} s",
                color="white", fontsize=12, fontweight="bold", ha="center", va="center"
            )

            # Draw vehicles
            for v in self.vehicles:
                # Find closest index
                if t in v.time_history:
                    idx = v.time_history.index(t)
                else:
                    idx = min(range(len(v.time_history)), key=lambda i: abs(v.time_history[i] - t))

                p = v.position_history[idx]
                speed = v.speed_history[idx]
                state = v.state_history[idx]

                if state == "DONE":
                    continue

                x, y, heading = self._get_vehicle_xy(v.approach, p)
                color = self.state_colors.get(state, "#ecf0f1")

                # Vehicle box
                rect = patches.Rectangle(
                    (-self.veh_len / 2.0, -self.veh_wid / 2.0),
                    self.veh_len, self.veh_wid,
                    facecolor=color, edgecolor="black", linewidth=1.2, zorder=10
                )
                t_box = patches.transforms.Affine2D().rotate_deg(heading).translate(x, y) + ax.transData
                rect.set_transform(t_box)
                ax.add_patch(rect)

                # Vehicle ID and Speed Label
                ax.text(
                    x, y + 2.8,
                    f"V{v.id} [{state}]\n{speed:.1f} m/s",
                    color="white", fontsize=7.5, fontweight="bold", ha="center", va="bottom",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="#2c3e50", alpha=0.7, edgecolor="none"),
                    zorder=12
                )

            # Legend
            legend_elements = [
                patches.Patch(facecolor="#f39c12", label="APPROACH"),
                patches.Patch(facecolor="#e74c3c", label="WAIT (Stop Line)"),
                patches.Patch(facecolor="#2ecc71", label="GO (Crossing)"),
            ]
            ax.legend(handles=legend_elements, loc="lower right", facecolor="#34495e", edgecolor="none", labelcolor="white", fontsize=8)

        anim = FuncAnimation(fig, update, frames=len(frame_times), repeat=True)
        writer = PillowWriter(fps=self.fps)
        anim.save(output_path, writer=writer)
        plt.close(fig)
        return output_path
