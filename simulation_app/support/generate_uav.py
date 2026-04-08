import random
from typing import List, Optional, Tuple

from simulation_app.domain.ground import Ground
from simulation_app.domain.uav import UAV
from simulation_app.domain.vector import Vector


class GenerateUAV:
    """
    Generates UAV positions while keeping the swarm within one connected component.
    """

    COMM_FACTOR = 0.9
    MAX_ATTEMPTS = 1000

    @staticmethod
    def run(
        count: int,
        comm_thr: float,
        ground: Optional[Ground] = None,
        canvas_width: int = 800,
        canvas_height: int = 600,
        reference_position: Optional[Vector] = None,
    ) -> List[UAV]:
        if count <= 0:
            raise ValueError("Count must be a positive integer.")
        if comm_thr <= 0:
            raise ValueError("Communication threshold must be a positive number.")
        if canvas_width <= 0 or canvas_height <= 0:
            raise ValueError("Canvas dimensions must be positive integers.")

        primary_ref = None
        if ground is not None:
            primary_ref = ground.pos
        elif reference_position is not None:
            primary_ref = reference_position

        uavs: List[UAV] = []

        if primary_ref is not None:
            x, y = GenerateUAV.generate_position_near_point(
                primary_ref.x,
                primary_ref.y,
                comm_thr,
                canvas_width,
                canvas_height,
            )
        else:
            x = random.uniform(0, canvas_width)
            y = random.uniform(0, canvas_height)

        uavs.append(UAV(pos=Vector(x, y), uav_no=1, ground=ground))

        for index in range(2, count + 1):
            x, y = GenerateUAV.generate_position_relative_to_existing(
                uavs=uavs,
                comm_thr=comm_thr,
                primary_ref=primary_ref,
                canvas_width=canvas_width,
                canvas_height=canvas_height,
            )
            uavs.append(UAV(pos=Vector(x, y), uav_no=index, ground=ground))

        return uavs

    @staticmethod
    def generate_position_near_point(
        x0: float,
        y0: float,
        comm_thr: float,
        canvas_width: int,
        canvas_height: int,
    ) -> Tuple[float, float]:
        attempts = 0
        while attempts < GenerateUAV.MAX_ATTEMPTS:
            x_min = max(0, x0 - comm_thr)
            x_max = min(canvas_width, x0 + comm_thr)
            y_min = max(0, y0 - comm_thr)
            y_max = min(canvas_height, y0 + comm_thr)

            if x_min >= x_max or y_min >= y_max:
                x_min, x_max = 0, canvas_width
                y_min, y_max = 0, canvas_height

            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            if 0 <= x <= canvas_width and 0 <= y <= canvas_height:
                return x, y
            attempts += 1

        raise RuntimeError("Failed to place UAV near the reference point.")

    @staticmethod
    def generate_position_relative_to_existing(
        uavs: List[UAV],
        comm_thr: float,
        primary_ref: Optional[Vector],
        canvas_width: int,
        canvas_height: int,
    ) -> Tuple[float, float]:
        attempts = 0
        factor = GenerateUAV.COMM_FACTOR

        while attempts < GenerateUAV.MAX_ATTEMPTS:
            x = random.uniform(0, canvas_width)
            y = random.uniform(0, canvas_height)
            new_pos = Vector(x, y)

            found_connection = any(
                (new_pos - existing_uav.pos).length() < comm_thr * factor
                for existing_uav in uavs
            )

            if not found_connection and primary_ref is not None:
                found_connection = (new_pos - primary_ref).length() < comm_thr * factor

            if found_connection:
                return x, y

            attempts += 1

        raise RuntimeError("Failed to place UAV inside one connected component.")
