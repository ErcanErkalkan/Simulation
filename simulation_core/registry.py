from __future__ import annotations

from importlib import import_module
from pkgutil import walk_packages
from typing import Dict, Iterable, List, Tuple, Type

from simulation_core.algorithm_api import SimulationAlgorithm


class AlgorithmRegistry:
    def __init__(self, search_packages: Tuple[str, ...] = ("algorithms", "user_algorithms")) -> None:
        self.search_packages = search_packages
        self._algorithms: Dict[str, Type[SimulationAlgorithm]] = {}
        self.discover()

    def discover(self) -> None:
        self._algorithms.clear()
        for package_name in self.search_packages:
            self._discover_from_package(package_name)
        if not self._algorithms:
            raise RuntimeError("No simulation algorithms were discovered.")

    def _discover_from_package(self, package_name: str) -> None:
        try:
            package = import_module(package_name)
        except ModuleNotFoundError:
            return

        for module_info in walk_packages(package.__path__, f"{package.__name__}."):
            short_name = module_info.name.rsplit(".", 1)[-1]
            if short_name.startswith("_"):
                continue
            module = import_module(module_info.name)
            for algorithm_type in self._iter_algorithm_types(module):
                self._algorithms[algorithm_type.name] = algorithm_type

    @staticmethod
    def _iter_algorithm_types(module) -> Iterable[Type[SimulationAlgorithm]]:
        for algorithm_type in getattr(module, "ALGORITHMS", []):
            if not issubclass(algorithm_type, SimulationAlgorithm):
                continue
            yield algorithm_type

    def create(self, name: str) -> SimulationAlgorithm:
        try:
            return self._algorithms[name]()
        except KeyError as exc:
            raise ValueError(f"Unknown algorithm: {name}") from exc

    def default_name(self) -> str:
        if "connectivity_current" in self._algorithms:
            return "connectivity_current"
        return sorted(self._algorithms)[0]

    def metadata(self) -> List[Dict[str, str]]:
        items = []
        for name, algorithm_type in self._algorithms.items():
            items.append(
                {
                    "name": name,
                    "display_name": algorithm_type.display_name,
                    "description": algorithm_type.description,
                }
            )
        return sorted(items, key=lambda item: item["display_name"])
