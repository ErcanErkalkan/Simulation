# generate_uav.py
import random
from typing import List, Optional, Tuple
from vector import Vector
from ground import Ground
from uav import UAV


class GenerateUAV:
    """
    Provides functionality to generate UAVs with random positions 
    forming a single connected component.
    """

    COMM_FACTOR = 0.9  # Factor to slightly reduce communication threshold
    MAX_ATTEMPTS = 1000  # Maximum number of attempts to find a valid position

    @staticmethod
    def run(
        count: int,
        comm_thr: float,
        ground: Optional[Ground] = None,
        canvas_width: int = 800,
        canvas_height: int = 600,
        reference_position: Optional[Vector] = None,
    ) -> List[UAV]:
        """
        Generates UAVs forming a single connected component within the communication threshold.
        
        :param count: Number of UAVs to generate.
        :param comm_thr: Communication threshold (max distance for connectivity).
        :param ground: Ground object (if any).
        :param canvas_width: Canvas width for random placement boundaries.
        :param canvas_height: Canvas height for random placement boundaries.
        :param reference_position: Optional Vector for CoDrone or other reference.
        :return: List of UAVs in a single connected component.
        """
        if count <= 0:
            raise ValueError("Count must be a positive integer.")
        if comm_thr <= 0:
            raise ValueError("Communication threshold must be a positive number.")
        if canvas_width <= 0 or canvas_height <= 0:
            raise ValueError("Canvas dimensions must be positive integers.")

        # Belirlenecek referans (Ground varsa ground.pos, yoksa reference_position, yoksa None)
        primary_ref = None
        if ground is not None:
            primary_ref = ground.pos
        elif reference_position is not None:
            primary_ref = reference_position

        uavs: List[UAV] = []

        # 1) İlk UAV
        if primary_ref:
            # Eğer bir referansımız varsa, ilk UAV referansa yakın konumlanır
            x, y = GenerateUAV.generate_position_near_point(
                primary_ref.x, primary_ref.y, comm_thr, canvas_width, canvas_height
            )
        else:
            # Hiç referans yoksa ilk UAV tamamen rastgele
            x = random.uniform(0, canvas_width)
            y = random.uniform(0, canvas_height)

        first_uav = UAV(pos=Vector(x, y), uav_no=1, ground=ground)
        uavs.append(first_uav)

        # 2) Diğer UAV’leri tek tek ekleyerek “connected component” koru
        for i in range(2, count + 1):
            x, y = GenerateUAV.generate_position_relative_to_existing(
                uavs=uavs,
                comm_thr=comm_thr,
                primary_ref=primary_ref,
                canvas_width=canvas_width,
                canvas_height=canvas_height,
            )
            new_uav = UAV(pos=Vector(x, y), uav_no=i, ground=ground)
            uavs.append(new_uav)

        return uavs

    @staticmethod
    def generate_position_near_point(
        x0: float,
        y0: float,
        comm_thr: float,
        canvas_width: int,
        canvas_height: int
    ) -> Tuple[float, float]:
        """
        Generates a position near a specific point (e.g., ground or coDrone pos) 
        within the communication threshold.
        """
        attempts = 0
        while attempts < GenerateUAV.MAX_ATTEMPTS:
            # Rastgele [x0 - comm_thr, x0 + comm_thr] içinde
            x_min = max(0, x0 - comm_thr)
            x_max = min(canvas_width, x0 + comm_thr)
            y_min = max(0, y0 - comm_thr)
            y_max = min(canvas_height, y0 + comm_thr)

            if x_min >= x_max or y_min >= y_max:
                # Yeterli alan yoksa tüm canvas'a fallback:
                x_min, x_max = 0, canvas_width
                y_min, y_max = 0, canvas_height

            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)

            if 0 <= x <= canvas_width and 0 <= y <= canvas_height:
                return x, y
            attempts += 1

        raise RuntimeError("Failed to place UAV near reference point within max attempts.")

    @staticmethod
    def generate_position_relative_to_existing(
        uavs: List[UAV],
        comm_thr: float,
        primary_ref: Optional[Vector],
        canvas_width: int,
        canvas_height: int
    ) -> Tuple[float, float]:
        """
        Generates a position that is within 'comm_thr * COMM_FACTOR' 
        distance of AT LEAST ONE existing UAV or the primary reference.
        """
        attempts = 0
        factor = GenerateUAV.COMM_FACTOR  # default 0.9

        while attempts < GenerateUAV.MAX_ATTEMPTS:
            x = random.uniform(0, canvas_width)
            y = random.uniform(0, canvas_height)
            new_pos = Vector(x, y)

            # Bir “bağlantı noktası” bulalım (mevcut UAV’lerden en az birine 
            # veya primary_ref'e mesafe < comm_thr*factor)
            found_connection = False

            # 1) Mevcut UAV’lere bak
            for existing_uav in uavs:
                dist = (new_pos - existing_uav.pos).length()
                if dist < comm_thr * factor:
                    found_connection = True
                    break

            # 2) (Opsiyonel) primary_ref’e de bak
            # (Eğer primary_ref varsa ve distance < comm_thr * factor ise bu da bağlantı sayılır)
            if not found_connection and primary_ref is not None:
                dist_ref = (new_pos - primary_ref).length()
                if dist_ref < comm_thr * factor:
                    found_connection = True

            if found_connection:
                return x, y

            attempts += 1

        raise RuntimeError("Failed to place UAV in connected component within max attempts.")
