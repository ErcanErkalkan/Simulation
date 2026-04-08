import unittest

from simulation_app.domain.goal import Goal
from simulation_app.domain.uav import UAV
from simulation_app.domain.vector import Vector
from simulation_app.hardware.co_drone import co_Drone
from simulation_app.ui.main_window import MainWindow
from simulation_core.config import SimulationConfig
from simulation_core.engine import SimulationEngine
from simulation_core.environment import SimulationEnvironment


class SimulationArchitectureTests(unittest.TestCase):
    def test_registry_exposes_multiple_algorithms(self):
        engine = SimulationEngine()
        metadata = engine.get_algorithm_metadata()
        names = {item["name"] for item in metadata}
        self.assertIn("connectivity_current", names)
        self.assertIn("greedy_distance", names)
        self.assertIn("user_nearest_goal", names)

    def test_single_visit_current_algorithm_reaches_goal(self):
        engine = SimulationEngine(algorithm_name="connectivity_current")
        engine.set_config_provider(lambda: SimulationConfig(target_eval_mode="single_visit"))
        engine.add_uav(0, 0)
        engine.add_goal(3, 0)

        engine.start_simulation()
        stop_reason = None
        for _ in range(10):
            stop_reason = engine.move_uavs()
            if stop_reason:
                break

        self.assertEqual("all_goals_visited", stop_reason)
        self.assertEqual("Visited", engine.goals[0].state)

    def test_environment_reopens_visited_goal_in_revisit_mode(self):
        environment = SimulationEnvironment()
        goal = environment.add_goal(10, 10)
        goal.state = "Visited"
        goal.last_visited_time = 5.0

        environment.update_goal_states(current_time=20.0, threshold1=10.0)

        self.assertEqual("Free", goal.state)

    def test_save_and_load_round_trip_preserves_entities(self):
        engine = SimulationEngine(algorithm_name="greedy_distance")
        engine.generate_ground(50, 50)
        engine.add_uav(10, 20)
        engine.add_goal(100, 120)

        payload = engine.to_dict()

        restored = SimulationEngine()
        restored.load_from_dict(payload)

        self.assertEqual(1, len(restored.uavs))
        self.assertEqual(1, len(restored.goals))
        self.assertIsNotNone(restored.ground)
        self.assertEqual(10, restored.uavs[0].pos.x)
        self.assertEqual(120, restored.goals[0].pos.y)

    def test_algorithm_can_be_switched_at_runtime(self):
        engine = SimulationEngine()
        engine.set_algorithm("greedy_distance")
        self.assertEqual("greedy_distance", engine.get_current_algorithm_name())

    def test_user_algorithm_can_run_via_stable_interface(self):
        engine = SimulationEngine(algorithm_name="user_nearest_goal")
        engine.set_config_provider(lambda: SimulationConfig(target_eval_mode="single_visit"))
        engine.add_uav(0, 0)
        engine.add_goal(3, 0)

        engine.start_simulation()

        stop_reason = None
        for _ in range(10):
            stop_reason = engine.move_uavs()
            if stop_reason:
                break

        self.assertEqual("all_goals_visited", stop_reason)
        self.assertEqual("Visited", engine.goals[0].state)

    def test_save_normalizes_transient_assigned_goal_state(self):
        engine = SimulationEngine(algorithm_name="greedy_distance")
        engine.set_config_provider(lambda: SimulationConfig(target_eval_mode="single_visit"))
        engine.add_uav(0, 0)
        engine.add_goal(10, 0)
        engine.start_simulation()
        engine.move_uavs()

        payload = engine.to_dict()
        restored = SimulationEngine(algorithm_name="greedy_distance")
        restored.set_config_provider(lambda: SimulationConfig(target_eval_mode="single_visit"))
        restored.load_from_dict(payload)

        self.assertEqual("Free", payload["Goals"][0]["State"])
        self.assertEqual("Free", restored.goals[0].state)

        restored.start_simulation()
        stop_reason = None
        for _ in range(20):
            stop_reason = restored.move_uavs()
            if stop_reason:
                break

        self.assertEqual("all_goals_visited", stop_reason)
        self.assertEqual("Visited", restored.goals[0].state)

    def test_move_to_position_snaps_when_target_is_within_speed(self):
        uav = UAV(pos=Vector(0, 0))

        uav.move_to_position(Vector(0.5, 0))

        self.assertEqual(Vector(0.5, 0), uav.pos)

    def test_move_to_target_uses_configured_uav_speed(self):
        uav = UAV(pos=Vector(0, 0))
        uav.speed = 5.0
        goal = Goal(pos=Vector(12, 0), goal_no=1)

        uav.move_to_target(goal)

        self.assertEqual(Vector(5, 0), uav.pos)
        self.assertIs(goal, uav.target)
        self.assertEqual("Free", goal.state)

    def test_loaded_codrone_rebinds_command_queue(self):
        engine = SimulationEngine()
        engine.register_uav_type(co_Drone)
        engine.add_existing_uav(co_Drone(pos=Vector(1, 2), command_queue="old"))
        payload = engine.to_dict()

        restored = SimulationEngine()
        restored.register_uav_type(co_Drone)
        restored.load_from_dict(payload)
        self.assertIsNone(restored.uavs[0].command_queue)

        queue_token = object()
        MainWindow.attach_runtime_uav_dependencies(restored.uavs, queue_token)

        self.assertIs(queue_token, restored.uavs[0].command_queue)

    def test_connectivity_algorithm_handles_empty_environment(self):
        engine = SimulationEngine(algorithm_name="connectivity_current")
        engine.set_config_provider(lambda: SimulationConfig(target_eval_mode="single_visit"))

        engine.start_simulation()

        self.assertIsNone(engine.move_uavs())
        self.assertTrue(engine.simulation_running)

    def test_connectivity_algorithm_handles_ground_without_uavs(self):
        engine = SimulationEngine(algorithm_name="connectivity_current")
        engine.set_config_provider(lambda: SimulationConfig(target_eval_mode="single_visit"))
        engine.generate_ground(0, 0)

        engine.start_simulation()

        self.assertIsNone(engine.move_uavs())
        self.assertTrue(engine.simulation_running)

    def test_greedy_algorithm_applies_runtime_uav_speed(self):
        engine = SimulationEngine(algorithm_name="greedy_distance")
        engine.set_config_provider(
            lambda: SimulationConfig(target_eval_mode="single_visit", uav_speed=6.0)
        )
        engine.add_uav(0, 0)
        engine.add_goal(20, 0)

        engine.start_simulation()
        engine.move_uavs()

        self.assertEqual(Vector(6, 0), engine.uavs[0].pos)

    def test_connectivity_algorithm_applies_runtime_uav_speed(self):
        engine = SimulationEngine(algorithm_name="connectivity_current")
        engine.set_config_provider(
            lambda: SimulationConfig(target_eval_mode="single_visit", uav_speed=7.0)
        )
        engine.add_uav(0, 0)
        engine.add_goal(20, 0)

        engine.start_simulation()
        engine.move_uavs()

        self.assertEqual(Vector(7, 0), engine.uavs[0].pos)


if __name__ == "__main__":
    unittest.main()
