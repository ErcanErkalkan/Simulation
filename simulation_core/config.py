from dataclasses import dataclass


@dataclass(frozen=True)
class SimulationConfig:
    target_eval_mode: str = "single_visit"
    threshold1: float = 10.0
    threshold2: float = 30.0
    simulation_time: float = 100.0

    def validate(self) -> None:
        if self.target_eval_mode not in {"single_visit", "revisit"}:
            raise ValueError(
                "target_eval_mode must be either 'single_visit' or 'revisit'."
            )
        if self.threshold1 < 0 or self.threshold2 < 0:
            raise ValueError("Threshold values cannot be negative.")
        if self.threshold2 < self.threshold1:
            raise ValueError("Threshold 2 must be greater than or equal to Threshold 1.")
        if self.simulation_time <= 0:
            raise ValueError("Simulation time must be positive.")
