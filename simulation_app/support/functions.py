from typing import List, Optional, Tuple

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from simulation_app.domain.goal import Goal
from simulation_app.domain.ground import Ground
from simulation_app.domain.uav import UAV


class Matrix:
    @staticmethod
    def zeros(rows: int, cols: int) -> np.ndarray:
        return np.zeros((rows, cols))

    @staticmethod
    def random(rows: int, cols: int, min_val: float, max_val: float) -> np.ndarray:
        return np.random.uniform(min_val, max_val, (rows, cols))

    @staticmethod
    def identity(n: int) -> np.ndarray:
        return np.eye(n)

    @staticmethod
    def product(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        return np.dot(a, b)

    @staticmethod
    def subtract(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        return np.subtract(a, b)

    @staticmethod
    def as_string(matrix: np.ndarray) -> str:
        return np.array2string(matrix, formatter={"float_kind": lambda x: f"{x:8.3f}"})


class Trigonometry:
    @staticmethod
    def degrees_to_radians(degrees: float) -> float:
        return np.deg2rad(degrees)

    @staticmethod
    def radians_to_degrees(radians: float) -> float:
        return np.rad2deg(radians)

    @staticmethod
    def normalize_radians_to_degrees(radians: float) -> float:
        return np.rad2deg(radians) % 360.0


class MatrixOperation:
    @staticmethod
    def adj_matrix(uavs: List[UAV], ground: Optional[Ground] = None) -> np.ndarray:
        positions = [[uav.pos.x, uav.pos.y] for uav in uavs]
        if ground is not None:
            positions.append([ground.pos.x, ground.pos.y])
        if not positions:
            return np.zeros((0, 0))
        points = np.array(positions, dtype=float)
        return np.linalg.norm(points[:, None] - points[None, :], axis=2)

    @staticmethod
    def uav_to_goal(uavs: List[UAV], goals: List[Goal]) -> np.ndarray:
        uav_positions = np.array([[uav.pos.x, uav.pos.y] for uav in uavs], dtype=float)
        goal_positions = np.array(
            [[goal.pos.x, goal.pos.y] for goal in goals],
            dtype=float,
        )
        if uav_positions.size == 0 or goal_positions.size == 0:
            return np.zeros((len(uavs), len(goals)))
        return np.linalg.norm(
            uav_positions[:, None] - goal_positions[None, :],
            axis=2,
        )

    @staticmethod
    def find_connected_components(adj: np.ndarray, comm_thr: float) -> List[List[int]]:
        if adj.size == 0:
            return []
        graph = csr_matrix(adj <= comm_thr * 0.9)
        _, labels = connected_components(csgraph=graph, directed=False)
        num_components = int(labels.max()) + 1 if labels.size else 0
        return [
            [index for index, label in enumerate(labels) if label == component_id]
            for component_id in range(num_components)
        ]

    @staticmethod
    def find_risky_links(
        components: List[List[int]],
        adj: np.ndarray,
        comm_thr: float,
    ) -> List[Tuple[int, int]]:
        connect_thr = comm_thr * 0.9
        risky_links = []

        for index, comp_a in enumerate(components):
            for comp_b in components[index + 1:]:
                min_risk_dist = float("inf")
                min_risk_link = None
                for node_a in comp_a:
                    for node_b in comp_b:
                        dist_ab = adj[node_a, node_b]
                        if connect_thr < dist_ab <= comm_thr and dist_ab < min_risk_dist:
                            min_risk_dist = dist_ab
                            min_risk_link = (node_a, node_b)
                if min_risk_link is not None:
                    risky_links.append(min_risk_link)

        return risky_links
