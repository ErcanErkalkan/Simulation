import random
from typing import List, Optional, Tuple
from vector import Vector
from ground import Ground
from uav import UAV
from functions import MatrixOperation
from distance import Distance


class GenerateUAV:
    """
    Provides functionality to generate UAVs with random positions forming a single connected component.
    """

    COMM_FACTOR = 0.9  # Factor to adjust the communication threshold
    MAX_ATTEMPTS = 1000  # Maximum number of attempts to find a valid position

    @staticmethod
    def run(
        count: int,
        comm_thr: float,
        ground: Optional[Ground] = None,
        canvas_width: int = 800,
        canvas_height: int = 600,
        reference_position: Optional[Vector] = None,  # <-- EKLENDİ
    ) -> List[UAV]:
        """
        Generates UAVs forming a single connected component within the communication threshold.

        :param count: The number of UAVs to generate.
        :param comm_thr: The communication threshold for UAVs.
        :param ground: The ground object to include in the connected component (optional).
        :param color: The color of the UAVs.
        :param canvas_width: The width of the canvas.
        :param canvas_height: The height of the canvas.
        :return: A list of UAVs forming a single connected component.
        """
        if count <= 0:
            raise ValueError("Count must be a positive integer.")
        if comm_thr <= 0:
            raise ValueError(
                "Communication threshold must be a positive number.")
        if canvas_width <= 0 or canvas_height <= 0:
            raise ValueError("Canvas dimensions must be positive integers.")

        uavs: List[UAV] = []
        

        # Add ground object as a reference point if provided
        if ground:
            primary_ref = ground.pos 
        else:
            primary_ref = reference_position if reference_position else None

        # Place the first UAV relative to the ground or randomly
        if primary_ref:
            x, y = GenerateUAV.generate_position_near_point(
                primary_ref.x, primary_ref.y, comm_thr, canvas_width, canvas_height
            )
        else:
            x = random.uniform(0, canvas_width)
            y = random.uniform(0, canvas_height)

        uav = UAV(pos=Vector(x, y), uav_no=1, ground=ground)
        uavs.append(uav)

        # Generate additional UAVs
        for i in range(1, count):
            x, y = GenerateUAV.generate_position_relative_to_existing(
                uavs, comm_thr, primary_ref, canvas_width, canvas_height
            )
            new_uav = UAV(pos=Vector(x, y), uav_no=i + 1, ground=ground)
            uavs.append(new_uav)

        return uavs

    @staticmethod
    def generate_position_relative_to_existing(
        uavs: List[UAV],
        comm_thr: float,
        ground_position: Optional[Vector],
        canvas_width: int,
        canvas_height: int,
    ) -> Tuple[float, float]:
        """
        Generates a position relative to existing UAVs or the ground to maintain a single connected component.

        :param uavs: List of existing UAVs.
        :param comm_thr: Communication threshold.
        :param ground_position: Position of the ground object (if any).
        :param canvas_width: Width of the canvas.
        :param canvas_height: Height of the canvas.
        :return: A tuple representing the (x, y) position.
        """
        attempts = 0
        while attempts < GenerateUAV.MAX_ATTEMPTS:
            x = random.uniform(0, canvas_width)
            y = random.uniform(0, canvas_height)

            # Check distance to existing UAVs
            for uav in uavs:
                distance = uav.pos.distance_to(Vector(x, y))
                if distance < comm_thr * GenerateUAV.COMM_FACTOR:
                    return x, y

            # If ground exists, check distance to ground as well
            if ground_position:
                distance_to_ground = ground_position.distance_to(Vector(x, y))
                if distance_to_ground < comm_thr * GenerateUAV.COMM_FACTOR:
                    return x, y

            attempts += 1

        raise RuntimeError(
            "Failed to generate a valid UAV position within the maximum number of attempts.")

    @staticmethod
    def generate_position_near_point(
        x0: float, y0: float, comm_thr: float, canvas_width: int, canvas_height: int
    ) -> Tuple[float, float]:
        """
        Generates a position near a specific point (e.g., the ground) within the communication threshold.

        :param x0: X-coordinate of the reference point.
        :param y0: Y-coordinate of the reference point.
        :param comm_thr: Communication threshold.
        :param canvas_width: Width of the canvas.
        :param canvas_height: Height of the canvas.
        :return: A tuple representing the (x, y) position.
        """
        attempts = 0
        while attempts < GenerateUAV.MAX_ATTEMPTS:
            x_min = max(0, x0 - comm_thr)
            x_max = min(canvas_width, x0 + comm_thr)
            y_min = max(0, y0 - comm_thr)
            y_max = min(canvas_height, y0 + comm_thr)

            if x_min >= x_max or y_min >= y_max:
                # Fallback to full canvas range
                x_min, x_max = 0, canvas_width
                y_min, y_max = 0, canvas_height

            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)

            if 0 <= x <= canvas_width and 0 <= y <= canvas_height:
                return x, y

            attempts += 1

        raise RuntimeError(
            "Failed to generate a valid UAV position near the specified point within the maximum number of attempts.")
