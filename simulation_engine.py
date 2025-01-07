# simulation_engine.py
import time
from tkinter import messagebox
from typing import List, Set, Optional
from scipy.optimize import linear_sum_assignment
from uav import UAV
from goal import Goal
from ground import Ground
from vector import Vector
from functions import MatrixOperation
from distance import Distance
from generate_goal import GenerateGoal
from generate_uav import GenerateUAV
from valuation import Valuation
import numpy as np


class SimulationEngine:
    def __init__(self, comm_thr=200):
        self.comm_thr = comm_thr
        self.goals: List[Goal] = []
        self.uavs: List[UAV] = []
        self.ground: Optional[Ground] = None
        self.simulation_running = False
        self.adj_matrix: Optional[np.ndarray] = None
        self.start_time = None  # For total simulation time
        self.target_eval_mode = None  # Will be set from MainWindow
        self.threshold1_entry = None  # Will be set from MainWindow
        self.threshold2_entry = None  # Will be set from MainWindow
        self.simulation_time_entry = None  # Will be set from MainWindow
        # Snapshot for rollback mechanism
        self._uav_snapshot = None
        self._goal_snapshot = None

    def add_goal(self, x: int, y: int):
        # Create a new goal object
        pos = Vector(x, y)
        new_goal = Goal(pos=pos, goal_no=len(self.goals) + 1)
        self.goals.append(new_goal)

    def generate_ground(self, x: int, y: int):
        pos = Vector(x, y)
        self.ground = Ground(pos=pos, color="lightgreen")

    def generate_goals(self, goal_count: int, canvas_width: int, canvas_height: int):
        self.goals = GenerateGoal.run(goal_count, canvas_width, canvas_height)

    def generate_uavs(self, uav_count: int, canvas_width: int, canvas_height: int):
        self.uavs = GenerateUAV.run(
            count=uav_count,
            comm_thr=self.comm_thr,
            ground=self.ground,
            canvas_width=canvas_width,
            canvas_height=canvas_height,
        )

    def add_uav(self, x: int, y: int):
        pos = Vector(x, y)
        new_uav = UAV(pos=pos)
        new_uav.uav_no = len(self.uavs) + 1
        self.uavs.append(new_uav)

    def reset_simulation(self):
        self.goals: List[Goal] = []
        self.uavs: List[UAV] = []
        self.ground: Optional[Ground] = None
        self.relay_uavs: Set[UAV] = set()
        self.simulation_running = False
        self.adj_matrix: Optional[np.ndarray] = None
        self.start_time = None  # For total simulation time
        self.simulation_time_entry = None  # Will be set from MainWindow
        # Snapshot for rollback mechanism
        self._uav_snapshot = None
        self._goal_snapshot = None

    def become_leader(self, uav: UAV, goal: Goal) -> bool:
        """
        Assign a UAV as a Leader for a specific Goal.
        Ensure the UAV is eligible and not in a higher-priority state.
        """
        if self.get_priority(uav) > 1 or uav.state != "Free":
            return False
        uav.state = "Leader"
        uav.target = goal
        goal.state = "Assigned"
        return True

    def all_goals_visited(self) -> bool:
        """
        Check if all goals in the simulation have been visited.

        Returns:
            bool: True if all goals are in the "Visited" state, False otherwise.
        """
        for goal in self.goals:
            if goal.state != "Visited":
                return False
        return True

    def assign_uavs_to_goals(self) -> bool:
        """
        Assign UAVs to Goals based on their availability and the evaluation mode.
        Returns True if changes were made, False otherwise.
        """
        if not self.uavs or not self.goals:
            return

        changed = False
        free_uavs = [uav for uav in self.uavs if uav.state == "Free"]
        if not free_uavs:
            return False

        mode = self.target_eval_mode.get() if self.target_eval_mode else "single_visit"
        if mode == "revisit":
            try:
                t_threshold1 = float(self.threshold1_entry.get())
                t_threshold2 = float(self.threshold2_entry.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid threshold values.")
                return

            # Sadece tekrar ziyaret edilebilir durumda olan hedefleri filtrele
            # "Free" olan veya "Visited" olup t_threshold1 süresini doldurmuş olan hedefler
            revisitable_goals = [
                g
                for g in self.goals
                if g.state == "Free"
                or (
                    g.state == "Visited"
                    and self.get_time_since_last_visit(g) >= t_threshold1
                )
            ]

            if not revisitable_goals:
                return False

            valuations = []
            for uav in free_uavs:
                for goal in revisitable_goals:
                    t = self.get_time_since_last_visit(goal)
                    valuation = Valuation.composite_valuation(
                        t, uav.pos, goal.pos, t_threshold1, t_threshold2
                    )
                    valuations.append((valuation, uav, goal))

            # Eğer değerleme listesi boşsa dönebiliriz
            if not valuations:
                return False

            valuations.sort(reverse=True, key=lambda x: x[0])

            assigned_uavs = set()
            assigned_goals = set()
            for valuation, uav, goal in valuations:
                if uav not in assigned_uavs and goal not in assigned_goals:
                    if self.become_leader(uav, goal):
                        assigned_uavs.add(uav)
                        assigned_goals.add(goal)
                        changed = True
                        if len(assigned_uavs) >= len(free_uavs):
                            break
        else:
            # Single Visit mode (mevcut kod)
            free_goals = [goal for goal in self.goals if goal.state == "Free"]
            if not free_goals:
                return False

            distance_matrix = MatrixOperation.uav_to_goal(free_uavs, free_goals)
            assigned_uavs = set()
            assigned_goals = set()
            while len(assigned_uavs) < len(free_uavs) and len(assigned_goals) < len(
                free_goals
            ):
                min_index = np.unravel_index(
                    np.argmin(distance_matrix, axis=None), distance_matrix.shape
                )
                uav_idx, goal_idx = min_index
                free_uavs[uav_idx].target = free_goals[goal_idx]
                free_uavs[uav_idx].state = "Leader"
                free_goals[goal_idx].state = "Assigned"
                assigned_uavs.add(uav_idx)
                assigned_goals.add(goal_idx)
                distance_matrix[uav_idx, :] = float("inf")
                distance_matrix[:, goal_idx] = float("inf")

    def move_uavs(self):
        """
        Perform UAV movements and manage role assignments.
        This function includes:
        - Snapshot and rollback mechanisms for consistency.
        - Progress checks to avoid unnecessary repetition.
        """
        if self.simulation_running:
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            mode = (
                self.target_eval_mode.get() if self.target_eval_mode else "single_visit"
            )
            if mode == "revisit":
                total_simulation_time = float(self.simulation_time_entry.get())
                if elapsed_time >= total_simulation_time:
                    # Stop the simulation
                    self.simulation_running = False
                    messagebox.showinfo(
                        "Simulation Ended", "Total simulation time has elapsed."
                    )
                    return

            # Update goal states based on time thresholds
            t_threshold1 = (
                float(self.threshold1_entry.get())
                if hasattr(self, "threshold1_entry")
                else 10
            )  # Default value

            for goal in self.goals:
                goal.update_state(current_time, t_threshold1)

        adj_matrix = MatrixOperation.adj_matrix(self.uavs, self.ground)
        components = MatrixOperation.find_connected_components(
            adj_matrix, self.comm_thr
        )
        if (
            len(
                MatrixOperation.find_connected_components(
                    adj_matrix, self.comm_thr * 1.1
                )
            )
            == 1
        ):
            self.reset_uavs(self.uavs)
            self.reset_goals(self.goals)
            self.relay_uavs = set()

        self.assign_uavs_to_goals()

        if len(components) > 1:
            self.assign_relay()

        for uav in self.uavs:
            if uav.state in ["Leader", "Ground_Leader"] and uav.target:
                if self.can_move(uav, uav.target.pos):
                    uav.move_to_target()
            elif uav.state == "Relay" and hasattr(uav, "target_position"):
                if self.can_move(uav, uav.target_position):
                    uav.move_to_position(uav.target_position)
                elif uav.my_leader != None:
                    if self.can_move(uav, uav.my_leader.pos):
                        uav.move_to_leader()
            elif uav.state == "Slave" and uav.my_leader:
                # Lider ile slave arası mesafeyi hesapla
                distance_to_leader = Distance.distance_between(uav, uav.my_leader)
                # Eğer mesafe, iletişim eşiğinin %80'inden fazlaysa yaklaşmaya devam et
                if distance_to_leader > self.comm_thr * 0.8:
                    if self.can_move(uav, uav.my_leader.pos):
                        uav.move_to_leader()
                # Aksi halde (mesafe zaten yeterince yakınsa) hareketsiz kal.

        # for uav in self.uavs:
        #    print(uav)

        # Update goal states based on time thresholds
        current_time = time.time()
        t_threshold1 = (
            float(self.threshold1_entry.get())
            if hasattr(self, "threshold1_entry")
            else 10
        )  # Default value

        for goal in self.goals:
            goal.update_state(current_time, t_threshold1)

    def can_move(self, uav: UAV, target_pos: Vector, ratio: int = 1.1) -> bool:
        """
        Determines if the UAV can move towards the target position without disconnecting the network.
        """
        # Save the current position
        original_pos = Vector(uav.pos.x, uav.pos.y)

        # Simulate moving to the next position
        next_pos = uav.get_next_position(target_pos)
        uav.pos = next_pos  # Temporarily move UAV to next position

        # Recalculate adjacency matrix
        adj_matrix = MatrixOperation.adj_matrix(self.uavs, self.ground)
        components = MatrixOperation.find_connected_components(
            adj_matrix, self.comm_thr * ratio
        )

        # Check if the network is still connected (only one component)
        is_connected = len(components) == 1

        # Move UAV back to original position
        uav.pos = original_pos

        return is_connected

    def reset_uavs(self, uavs: List[UAV]):
        for uav in uavs:
            uav.delete_my_slave_list()
            uav.state = "Free"
            uav.my_leader = None
            if uav.target:
                uav.target.state = "Free"
                uav.target = None
            if hasattr(uav, "target_position"):
                del uav.target_position

    def reset_goals(self, goals: List[Goal]):
        for goal in self.goals:
            if goal.state != "Visited":  # Only reset unachieved goals
                goal.state = "Free"  # Reset goal state"

    def assign_relay(self):
        """
        Assign relay UAVs to maintain network connectivity based on the communication threshold.
        """
        adj_matrix = MatrixOperation.adj_matrix(self.uavs, self.ground)
        components = MatrixOperation.find_connected_components(
            adj_matrix, self.comm_thr
        )
        risky_links = MatrixOperation.find_risky_links(
            components, adj_matrix, self.comm_thr
        )

        print(risky_links)

        for link in risky_links:
            # Ground station'ın indeksini belirleyin
            ground_idx = len(self.uavs)
            # Determine if the link involves the ground station
            involves_ground = link[0] == ground_idx or link[1] == ground_idx
            if involves_ground:
                # First, assign the most profitable UAV and its target
                self.reset_uavs(self.uavs)
                self.reset_goals(self.goals)
                leader_uav = self.assign_most_profitable_uav()
                if not leader_uav:
                    return self.relay_uavs  # No leader UAV assigned

                # Calculate the number of relays needed
                n_relay = self.calculate_number_of_relays(
                    leader_uav.target.pos, self.ground.pos
                )
                # Allocate relay UAVs along the line
                self.allocate_relay_uavs_line(leader_uav, n_relay)

        for link in risky_links:
            if link[0] == ground_idx or link[1] == ground_idx:
                # Zaten çözüldü veya ilgilenildi
                continue
            else:
                uav_a, uav_b = self.uavs[link[0]], self.uavs[link[1]]
                pa = self.get_priority(uav_a)
                pb = self.get_priority(uav_b)
                print("[", pa, pb, "]", end=" ")

                # Önce önceliklere bak, yüksek öncelikli olan lider, diğeri slave veya duruma göre relay yap
                if (pa > pb) or (pb > pa):
                    # Farklı öncelik varsa yüksek öncelikli kazansın
                    if pa > pb:
                        self.make_slave(uav_b, uav_a)
                        print("Ground Assigment")
                        print(pa, pb)
                    else:
                        self.make_slave(uav_a, uav_b)
                        print("Ground Assigment")
                        print(pa, pb)
                elif pa == pb == 2:
                    
                    # Both UAVs are Relay. Decide which one to make Slave based on their distance to Ground Leader
                    distance_a = Distance.distance_between_positions(uav_a.pos, uav_a.target_position)
                    distance_b = Distance.distance_between_positions(uav_b.pos, uav_b.target_position)

                    if distance_a < distance_b:
                        self.make_slave(uav_b, uav_a)
                        print(f"UAV {uav_b.uav_no} made Slave to UAV {uav_a.uav_no} based on proximity to Ground Leader")
                    else:
                        self.make_slave(uav_a, uav_b)
                        print(f"UAV {uav_a.uav_no} made Slave to UAV {uav_b.uav_no} based on proximity to Ground Leader")
   
                else:

                    if uav_a.state == "Leader" and uav_b.state == "Leader":
                        self.resolve_leader_conflict(uav_a, uav_b)
                    elif uav_a.state == "Free" and uav_b.state == "Free":
                        leader = self.get_closest_leader(uav_a)
                        if leader:
                            self.make_slave(uav_a, leader)
                            self.make_slave(uav_b, leader)

                    # One is Slave, the other is Free
                    elif (uav_a.state == "Slave" and uav_b.state == "Free") or (
                        uav_a.state == "Free" and uav_b.state == "Slave"
                    ):
                        leader = (
                            uav_a.get_leader()
                            if uav_a.state == "Slave"
                            else uav_b.get_leader()
                        )
                        slave = uav_b if uav_a.state == "Slave" else uav_a
                        self.make_slave(slave, leader)

                    # One is Leader, the other is Free
                    elif (uav_a.state == "Leader" and uav_b.state == "Free") or (
                        uav_a.state == "Free" and uav_b.state == "Leader"
                    ):
                        leader = uav_a if uav_a.state == "Leader" else uav_b
                        slave = uav_b if uav_a.state == "Leader" else uav_a
                        self.make_slave(slave, leader)

                    # Leader-Slave relationships
                    elif uav_a.state == "Leader" and uav_b.state == "Slave":
                        leader = uav_b.get_leader()
                        if leader != uav_a:
                            self.resolve_leader_conflict(uav_a, leader)

                    elif uav_a.state == "Slave" and uav_b.state == "Leader":
                        leader = uav_a.get_leader()
                        if leader != uav_b:
                            self.resolve_leader_conflict(leader, uav_b)

                    # Both are Slaves
                    elif uav_a.state == "Slave" and uav_b.state == "Slave":
                        leader_a = uav_a.get_leader()
                        leader_b = uav_b.get_leader()
                        if leader_a != leader_b:
                            self.resolve_leader_conflict(leader_a, leader_b)

        print()
        self.assign_uavs_to_goals()

    def get_priority(self, uav: UAV) -> int:
        if uav.state == "Ground_Leader":
            return 3
        elif uav.state == "Relay":
            return 2
        elif uav.state in ["Leader", "Slave", "Free"]:
            return 1
        return 0

    def make_slave(self, slave_uav: UAV, leader_uav: UAV):
        """
        Makes a UAV a slave to a leader UAV and updates the relationships.
        """
        # Aşağıda slave_uav, gerçekten daha düşük öncelikli olandır.
        if slave_uav.target:
            slave_uav.target.state = "Free"
            slave_uav.target = None

        for slave in slave_uav.my_slave_list:
            slave.state = "Slave"
            slave.my_leader = leader_uav
            leader_uav.my_slave_list.add(slave)  # Add to leader's slave set

        slave_uav.delete_my_slave_list()  # Clear old relationships
        slave_uav.state = "Slave"
        slave_uav.my_leader = leader_uav
        leader_uav.my_slave_list.add(slave_uav)  # Add to leader's slave set

    def resolve_leader_conflict(self, uav_a: UAV, uav_b: UAV):
        """
        Resolves conflict between two leader UAVs by making one a slave of the other.
        """
        distance_a = (
            Distance.distance_between(uav_a, uav_a.target)
            if uav_a.target
            else float("inf")
        )
        distance_b = (
            Distance.distance_between(uav_b, uav_b.target)
            if uav_b.target
            else float("inf")
        )

        if distance_a > distance_b:
            self.make_slave(uav_a, uav_b)
        else:
            self.make_slave(uav_b, uav_a)

    def get_closest_leader(self, uav: UAV) -> Optional[UAV]:
        """
        Finds the closest leader UAV to the given UAV.
        """
        candidates = [
            x for x in self.uavs if x.state in ["Ground_Leader", "Relay", "Leader"]
        ]
        if not candidates:
            return None
        return min(
            candidates, key=lambda leader: Distance.distance_between(leader, uav)
        )

    def assign_most_profitable_uav(self) -> UAV:
        """
        Assigns one UAV to one goal based on the current evaluation mode:
        - In "Single Visit" mode: Chooses the UAV-goal pair with the minimal distance.
        - In "Revisit" mode: Uses composite valuation (time + distance) and thresholds.
        Returns the UAV chosen as a leader, or None if no assignments can be made.
        """
        # Filter free UAVs and goals
        free_uavs = [uav for uav in self.uavs if uav.state == "Free"]

        # In revisit mode, consider "Free" and "Visited" goals since they can revert to "Free" after time.
        if self.target_eval_mode.get() == "revisit":
            available_goals = [
                goal for goal in self.goals if goal.state in ("Free", "Visited")
            ]
        else:
            available_goals = [goal for goal in self.goals if goal.state == "Free"]

        if not free_uavs or not available_goals:
            return None  # No assignment possible

        # Check the evaluation mode
        if self.target_eval_mode.get() == "revisit":
            # Revisit mode: Use composite valuation
            try:
                t_threshold1 = float(self.threshold1_entry.get())
                t_threshold2 = float(self.threshold2_entry.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid threshold values.")
                return None

            valuations = []
            for uav in free_uavs:
                for goal in available_goals:
                    t = self.get_time_since_last_visit(goal)
                    valuation = Valuation.composite_valuation(
                        t, uav.pos, goal.pos, t_threshold1, t_threshold2
                    )
                    valuations.append((valuation, uav, goal))

            if not valuations:
                return None

            # Select the best valuation
            max_valuation, best_uav, best_goal = max(valuations, key=lambda x: x[0])
        else:
            # Single Visit mode: Use inverse distance
            valuations = []
            for uav in free_uavs:
                for (
                    goal
                ) in available_goals:  # In single visit, these are just free goals
                    dist = Distance.distance_between(uav, goal)
                    valuation = 1 / dist if dist != 0 else float("inf")
                    valuations.append((valuation, uav, goal))

            if not valuations:
                return None

            # Select the best (lowest distance => highest valuation)
            max_valuation, best_uav, best_goal = max(valuations, key=lambda x: x[0])

        # Assign the UAV to the chosen goal
        best_uav.target = best_goal
        best_uav.state = "Ground_Leader"
        best_goal.state = "Assigned"

        return best_uav

    def get_time_since_last_visit(self, goal: Goal):
        """
        Returns the time since the goal was last visited. If never visited, returns a large value.
        """
        current_time = time.time()
        if goal.last_visited_time is not None:
            return max(current_time - goal.last_visited_time, 0)
        return float("inf")  # Treat as very high priority if never visited

    def calculate_number_of_relays(self, pos_uav: Vector, pos_station: Vector) -> int:
        """
        Calculate the number of relay UAVs required between a UAV and the central station.
        """
        distance = Distance.distance_between_positions(pos_uav, pos_station)
        n_relay = max(int(np.ceil(distance / (0.9 * self.comm_thr))) - 1, 0)
        return n_relay

    def allocate_relay_uavs_line(self, leader_uav: UAV, n_relay: int):
        """
        Allocate relay UAVs along the line between the central station and the leader UAV's target.
        Ensures relay UAVs are positioned only between the ground and the target.
        """
        # Generate relay positions between ground and leader's target
        positions = []
        for i in range(1, n_relay + 2):  # Start from 1 to exclude ground's position
            position_factor = i / (n_relay + 1)
            relay_pos = (
                self.ground.pos
                + (leader_uav.target.pos - self.ground.pos) * position_factor
            )
            positions.append(relay_pos)

        # Assign the leader UAV to its target position (the goal)
        leader_uav.target_position = positions[-1]

        # Find all free UAVs excluding the leader UAV
        free_uavs = [
            uav for uav in self.uavs if uav.state == "Free" and uav != leader_uav
        ]

        if not free_uavs:
            print("No free UAVs available for relay allocation.")
            return

        # Assign UAVs to relay positions based on proximity
        for pos in positions[
            :-1
        ]:  # Exclude the final position (the leader UAV's target)
            # Find the closest UAV to the current relay position
            closest_uav = min(
                free_uavs,
                key=lambda uav: Distance.distance_between_positions(uav.pos, pos),
            )
            # Ensure the UAV can actually move to the relay position
            # if self.can_move(closest_uav, pos, 1.1):
            # Assign the UAV to the relay position
            closest_uav.target_position = pos
            closest_uav.state = "Relay"
            closest_uav.my_leader = leader_uav
            leader_uav.my_slave_list.add(closest_uav)
            # Remove the assigned UAV from the free UAVs list
            free_uavs.remove(closest_uav)
            # else:
            #    print(
            #        f"UAV {closest_uav.uav_no} cannot move to the relay position {pos}."
            #    )

    

        

