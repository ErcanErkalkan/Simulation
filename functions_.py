import numpy as np
from typing import TYPE_CHECKING, List, Optional, Tuple
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from uav import UAV
from goal import Goal
from ground import Ground


class Matrix:
    """Utility methods for matrix creation and operations using NumPy."""

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
    """Trigonometric calculations."""

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
        positions = np.array([[uav.pos.x, uav.pos.y] for uav in uavs])
        if ground:
            positions = np.vstack((positions, [ground.pos.x, ground.pos.y]))
        return np.linalg.norm(positions[:, None] - positions[None, :], axis=2)

    @staticmethod
    def uav_to_goal(uavs: List[UAV], goals: List[Goal]) -> np.ndarray:
        uav_positions = np.array([[uav.pos.x, uav.pos.y] for uav in uavs])
        goal_positions = np.array([[goal.pos.x, goal.pos.y] for goal in goals])
        return np.linalg.norm(uav_positions[:, None] - goal_positions[None, :], axis=2)

    @staticmethod
    def find_connected_components(adj: np.ndarray, cthr: float) -> List[List[int]]:
        graph = csr_matrix(adj <= cthr * 0.9)
        _, labels = connected_components(csgraph=graph, directed=False)
        return [
            [i for i, label in enumerate(labels) if label == group]
            for group in range(max(labels) + 1)
        ]

    """@staticmethod
    def find_risky_links(
        components: List[List[int]], adj: np.ndarray, cthr: float
    ) -> List[Tuple[int, int]]:
        return [
            (a, b)
            for i, comp_a in enumerate(components)
            for comp_b in components[i + 1 :]
            for a in comp_a
            for b in comp_b
            if adj[a, b] <= cthr
        ]"""
    
    @staticmethod
    def find_risky_links(
        components: List[List[int]], adj: np.ndarray, cthr: float
    ) -> List[Tuple[int, int]]:
        risky_links = []
        for i, comp_a in enumerate(components):
            for comp_b in components[i + 1:]:
                min_risk = float('inf')
                min_link = None
                for a in comp_a:
                    for b in comp_b:
                        if cthr * 0.9 < adj[a, b] <= cthr and adj[a, b] < min_risk:
                            min_risk = adj[a, b]
                            min_link = (a, b)
                if min_link:
                    risky_links.append(min_link)
        return risky_links

