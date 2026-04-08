from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from simulation_core.algorithm_api import Position


def distance_between(first: Position, second: Position) -> float:
    return float(np.hypot(first.x - second.x, first.y - second.y))


def build_distance_matrix(positions: Sequence[Position]) -> np.ndarray:
    if not positions:
        return np.zeros((0, 0))
    values = np.array([[position.x, position.y] for position in positions], dtype=float)
    return np.linalg.norm(values[:, None] - values[None, :], axis=2)


def build_uav_goal_distance_matrix(
    uavs: Sequence[Position], goals: Sequence[Position]
) -> np.ndarray:
    if not uavs or not goals:
        return np.zeros((len(uavs), len(goals)))
    uav_values = np.array([[position.x, position.y] for position in uavs], dtype=float)
    goal_values = np.array([[position.x, position.y] for position in goals], dtype=float)
    return np.linalg.norm(uav_values[:, None] - goal_values[None, :], axis=2)


def find_connected_components(adj_matrix: np.ndarray, comm_thr: float) -> List[List[int]]:
    if adj_matrix.size == 0:
        return []
    graph = csr_matrix(adj_matrix <= comm_thr * 0.9)
    _, labels = connected_components(csgraph=graph, directed=False)
    return [
        [index for index, label in enumerate(labels) if label == group]
        for group in range(max(labels) + 1)
    ]


def find_risky_links(
    components: List[List[int]], adj_matrix: np.ndarray, comm_thr: float
) -> List[Tuple[int, int]]:
    risky_links = []
    for index, component_a in enumerate(components):
        for component_b in components[index + 1 :]:
            min_risk = float("inf")
            min_link = None
            for node_a in component_a:
                for node_b in component_b:
                    if (
                        comm_thr * 0.9 < adj_matrix[node_a, node_b] <= comm_thr
                        and adj_matrix[node_a, node_b] < min_risk
                    ):
                        min_risk = float(adj_matrix[node_a, node_b])
                        min_link = (node_a, node_b)
            if min_link:
                risky_links.append(min_link)
    return risky_links
