# simulation_engine.py

import time
from tkinter import messagebox
from typing import List, Set, Optional
import numpy as np

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

        self._uav_snapshot = None
        self._goal_snapshot = None
        self.counter = 0
        self.relay_uavs: Set[UAV] = set()

        # Örnek: UAV'leri adım adım hareket ettirmek için sabit hız
        # (Bounce sorununu azaltmada faydalı olabilir)
        self.move_step = 5.0

    def add_goal(self, x: int, y: int):
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
        self.goals.clear()
        self.uavs.clear()
        self.ground = None
        self.relay_uavs.clear()
        self.simulation_running = False
        self.adj_matrix = None
        self.start_time = None
        self.simulation_time_entry = None
        self._uav_snapshot = None
        self._goal_snapshot = None
        self.counter = 0

    def become_leader(self, uav: UAV, goal: Goal) -> bool:
        """
        Bir UAV'yi belirli bir goal için Leader yapar.
        """
        if self.get_priority(uav) > 1 or uav.state != "Free":
            return False
        uav.state = "Leader"
        uav.target = goal
        goal.state = "Assigned"
        return True

    def all_goals_visited(self) -> bool:
        return all(goal.state == "Visited" for goal in self.goals)

    def assign_uavs_to_goals(self) -> bool:
        """
        Uygun UAV'leri uygun hedeflere atar.
        Bazen tekrar tekrar çağrılması, bouncing'i tetikleyebilir.
        Dolayısıyla, 'yeni bir atama gerekliyse' veya 'boşta UAV ve hedef varsa' gibi
        ek kontrollerle kullanılabilir.
        """
        if not self.uavs or not self.goals:
            return False

        free_uavs = [uav for uav in self.uavs if uav.state == "Free"]
        if not free_uavs:
            return False

        # Single-visit ya da Revisit moduna göre atama
        mode = self.target_eval_mode.get() if self.target_eval_mode else "single_visit"
        changed = False

        if mode == "revisit":
            try:
                t_threshold1 = float(self.threshold1_entry.get())
                t_threshold2 = float(self.threshold2_entry.get())
            except ValueError:
                messagebox.showerror("Error", "Invalid threshold values.")
                return False

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
            # Single Visit
            free_goals = [goal for goal in self.goals if goal.state == "Free"]
            if not free_goals:
                return False

            distance_matrix = MatrixOperation.uav_to_goal(free_uavs, free_goals)
            assigned_uavs = set()
            assigned_goals = set()

            while len(assigned_uavs) < len(free_uavs) and len(assigned_goals) < len(free_goals):
                uav_idx, goal_idx = np.unravel_index(
                    np.argmin(distance_matrix, axis=None), distance_matrix.shape
                )
                free_uavs[uav_idx].target = free_goals[goal_idx]
                free_uavs[uav_idx].state = "Leader"
                free_goals[goal_idx].state = "Assigned"
                assigned_uavs.add(uav_idx)
                assigned_goals.add(goal_idx)
                distance_matrix[uav_idx, :] = float("inf")
                distance_matrix[:, goal_idx] = float("inf")
                changed = True

        return changed

    def move_uavs(self):
        """
        Her döngüde UAV'lerin hareketini yöneten ana fonksiyon.
        Burada bouncing sorununu azaltmak için:
          1) Sürekli 'assign_uavs_to_goals()' çağırmıyoruz (örnek: her 2-3 döngüde bir veya 
             sadece gerçekten boşta UAV/goal varsa çağırabiliriz).
          2) Relay UAV'ler eğer hedef noktasına gidemiyorsa, 'küçük adım' yaklaşımı veya 
             'orada bekle' yaklaşımı uyguluyoruz.
          3) 'can_move()' kontrolünün ratio'sunu biraz daha düşük tutabilir veya 
             'hareket edemediyse bir sonraki frame'de tekrar deneme' kuralı getirebiliriz.
        """

        if not self.simulation_running:
            return

        current_time = time.time()
        elapsed_time = current_time - self.start_time if self.start_time else 0.0

        mode = self.target_eval_mode.get() if self.target_eval_mode else "single_visit"
        if mode == "revisit":
            try:
                total_simulation_time = float(self.simulation_time_entry.get())
                if elapsed_time >= total_simulation_time:
                    self.simulation_running = False
                    messagebox.showinfo(
                        "Simulation Ended", "Total simulation time has elapsed."
                    )
                    return
            except ValueError:
                pass  # Eğer time entry yoksa, bir şey yapma

        # Zaman eşiğine göre bazı goal state güncellemeleri
        try:
            t_threshold1 = float(self.threshold1_entry.get())
        except (ValueError, AttributeError):
            t_threshold1 = 10

        for goal in self.goals:
            goal.update_state(current_time, t_threshold1)

        # Adjacency ve bileşenleri kontrol et
        adj_matrix = MatrixOperation.adj_matrix(self.uavs, self.ground)
        components = MatrixOperation.find_connected_components(adj_matrix, self.comm_thr)

        # Aşırı sık reset yapmayalım, bouncing sorununu artırabilir.
        # Bu bloğu isterseniz kaldırabilir veya koşulu değiştirebilirsiniz.
        if self.counter > 0 and self.counter % 50 == 0:  # 20 yerine 50 veya daha fazlası
            # Bir de ratio'yu 1.0 tutabilirsiniz.
            if len(MatrixOperation.find_connected_components(adj_matrix, self.comm_thr * 1.0)) == 1:
                self.reset_uavs(self.uavs)
                self.reset_goals(self.goals)
                self.relay_uavs.clear()
        self.counter += 1

        # Burada atama yaparken, eğer gerçekten FREE UAV veya FREE goal varsa yapıyoruz
        free_uavs = [u for u in self.uavs if u.state == "Free"]
        free_goals = [g for g in self.goals if g.state == "Free"]
        if free_uavs and free_goals:
            self.assign_uavs_to_goals()

        # Eğer network birden fazla bileşene bölünmüşse relay ataması
        if len(components) > 1:
            self.assign_relay()

        # -------------- UAV’leri hareket ettirme --------------
        for uav in self.uavs:
            # Leader veya Ground_Leader -> Hedefine küçük adım yaklaş
            if uav.state in ["Leader", "Ground_Leader"] and uav.target:
                self.move_uav_step(uav, uav.target.pos)

            # Relay -> target_position'a küçük adım yaklaşma
            elif uav.state == "Relay" and hasattr(uav, "target_position"):
                # İlk denemede oraya gidebiliyorsak git
                if self.can_move_step(uav, uav.target_position):
                    self.move_uav_step(uav, uav.target_position)
                else:
                    # Olmuyorsa, geçici olarak bekle
                    # (Eskiden burası uav.move_to_leader() vb. fallback yapıyordu
                    #  ama bu bouncing'e neden olabiliyor.)
                    pass

            # Slave -> liderine yaklaş
            elif uav.state == "Slave" and uav.my_leader:
                distance_to_leader = Distance.distance_between(uav, uav.my_leader)
                if distance_to_leader > self.comm_thr * 0.8:
                    if self.can_move_step(uav, uav.my_leader.pos):
                        self.move_uav_step(uav, uav.my_leader.pos)
                    # else: bekle

        # Goal durumlarını tekrar güncelle
        current_time = time.time()
        for goal in self.goals:
            goal.update_state(current_time, t_threshold1)

    def can_move_step(self, uav: UAV, target_pos: Vector, ratio: float = 1.1) -> bool:
        """
        UAV'nin target_pos'a küçük bir adım (ör. self.move_step) atıp atamayacağını kontrol eder.
        Ağı koparmayacaksa True döndürür.
        """
        original_pos = Vector(uav.pos.x, uav.pos.y)

        # Küçük bir adım hesapla
        step_vector = (target_pos - original_pos)
        distance_to_target = step_vector.length()
        if distance_to_target > self.move_step:
            step_vector = step_vector.normalize() * self.move_step

        next_pos = original_pos + step_vector

        # Geçici olarak UAV'yi next_pos'a taşı
        uav.pos = next_pos
        adj_matrix = MatrixOperation.adj_matrix(self.uavs, self.ground)
        components = MatrixOperation.find_connected_components(adj_matrix, self.comm_thr * ratio)
        is_connected = (len(components) == 1)

        # Eski konumuna dön
        uav.pos = original_pos

        return is_connected

    def move_uav_step(self, uav: UAV, target_pos: Vector):
        """
        UAV'yi target_pos yönünde küçük adım (self.move_step) hareket ettirir;
        örn. 'uav.move_to_target()' yerine bu fonksiyonla adım adım hareket sağlıyoruz.
        """
        current_pos = uav.pos
        direction = target_pos - current_pos
        dist = direction.length()

        if dist <= self.move_step:
            # Hedefe çok yakınsa direkt hedefe git
            uav.pos = target_pos
        else:
            # Normalde 'uav.move_to_target()' tek seferde atlıyordu.
            # Onu parçalıyoruz:
            direction = direction.normalize()
            new_pos = current_pos + direction * self.move_step
            uav.pos = new_pos

    def can_move(self, uav: UAV, target_pos: Vector, ratio: int = 1.1) -> bool:
        """
        Orijinal can_move, TAMPONLU. Tam hedefe tek seferde gidebilir miyim?
        Bounce sorununa sebep olabilirdi. Yukarıda 'can_move_step()' kullanmak daha iyi.
        Burayı isterseniz tamamen kaldırabilir veya sadece geriye uyumluluk için bırakabilirsiniz.
        """
        original_pos = Vector(uav.pos.x, uav.pos.y)
        next_pos = uav.get_next_position(target_pos)
        uav.pos = next_pos

        adj_matrix = MatrixOperation.adj_matrix(self.uavs, self.ground)
        components = MatrixOperation.find_connected_components(adj_matrix, self.comm_thr * ratio)
        is_connected = len(components) == 1

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
        for goal in goals:
            if goal.state != "Visited":
                goal.state = "Free"

    def assign_relay(self):
        """
        Ağın kopması durumunda relay ataması yapar.
        Burada da bouncing'i azaltmak için UAV'leri küçük adımlarla konumlandıran
        'allocate_relay_uavs_line()' vb. yaklaşım kullanılabilir.
        """
        adj_matrix = MatrixOperation.adj_matrix(self.uavs, self.ground)
        components = MatrixOperation.find_connected_components(adj_matrix, self.comm_thr)
        risky_links = MatrixOperation.find_risky_links(components, adj_matrix, self.comm_thr)

        if not risky_links:
            return

        # Ground station'ın indeksini belirle
        ground_idx = len(self.uavs)

        for link in risky_links:
            involves_ground = (link[0] == ground_idx or link[1] == ground_idx)
            if involves_ground:
                # En karlı UAV ve hedefini leader yap
                self.reset_uavs(self.uavs)
                self.reset_goals(self.goals)
                leader_uav = self.assign_most_profitable_uav()
                if not leader_uav:
                    return

                n_relay = self.calculate_number_of_relays(leader_uav.target.pos, self.ground.pos)
                self.allocate_relay_uavs_line(leader_uav, n_relay)
            else:
                uav_a, uav_b = self.uavs[link[0]], self.uavs[link[1]]
                pa = self.get_priority(uav_a)
                pb = self.get_priority(uav_b)

                # Eğer farklı öncelik varsa yüksek öncelikli leader kalsın, öbürü slave olsun
                if (pa > pb):
                    self.make_slave(uav_b, uav_a)
                elif (pb > pa):
                    self.make_slave(uav_a, uav_b)
                elif pa == pb == 2:
                    # İkisi de Relay, hangisi lidere yakınsa Relay kalsın
                    dist_a = Distance.distance_between_positions(uav_a.pos, uav_a.target_position)
                    dist_b = Distance.distance_between_positions(uav_b.pos, uav_b.target_position)
                    if dist_a < dist_b:
                        self.make_slave(uav_b, uav_a)
                    else:
                        self.make_slave(uav_a, uav_b)
                else:
                    # Diğer durumlar: Leader- Leader, Slave-Slave vb.
                    self.resolve_conflicts(uav_a, uav_b)

    def resolve_conflicts(self, uav_a: UAV, uav_b: UAV):
        """
        Leader-Leader veya Slave-Slave çatışmalarını çözmek için yardımcı fonksiyon.
        """
        if uav_a.state == "Leader" and uav_b.state == "Leader":
            self.resolve_leader_conflict(uav_a, uav_b)
        elif uav_a.state == "Slave" and uav_b.state == "Slave":
            leader_a = uav_a.get_leader()
            leader_b = uav_b.get_leader()
            if leader_a != leader_b:
                self.resolve_leader_conflict(leader_a, leader_b)
        else:
            # Diğer olası kombinasyonlar
            pass

    def get_priority(self, uav: UAV) -> int:
        if uav.state == "Ground_Leader":
            return 3
        elif uav.state == "Relay":
            return 2
        elif uav.state in ["Leader", "Slave", "Free"]:
            return 1
        return 0

    def make_slave(self, slave_uav: UAV, leader_uav: UAV):
        if slave_uav.target:
            slave_uav.target.state = "Free"
            slave_uav.target = None

        for sub_slave in slave_uav.my_slave_list:
            sub_slave.state = "Slave"
            sub_slave.my_leader = leader_uav
            leader_uav.my_slave_list.add(sub_slave)

        slave_uav.delete_my_slave_list()
        slave_uav.state = "Slave"
        slave_uav.my_leader = leader_uav
        leader_uav.my_slave_list.add(slave_uav)

    def resolve_leader_conflict(self, uav_a: UAV, uav_b: UAV):
        """
        İki lider UAV'den hangisinin slave olacağına karar ver.
        """
        dist_a = (
            Distance.distance_between(uav_a, uav_a.target)
            if uav_a.target
            else float("inf")
        )
        dist_b = (
            Distance.distance_between(uav_b, uav_b.target)
            if uav_b.target
            else float("inf")
        )

        if dist_a > dist_b:
            self.make_slave(uav_a, uav_b)
        else:
            self.make_slave(uav_b, uav_a)

    def get_closest_leader(self, uav: UAV) -> Optional[UAV]:
        candidates = [x for x in self.uavs if x.state in ["Ground_Leader", "Relay", "Leader"]]
        if not candidates:
            return None
        return min(candidates, key=lambda leader: Distance.distance_between(leader, uav))

    def assign_most_profitable_uav(self) -> Optional[UAV]:
        """
        Tek bir UAV'yi hedefle eşleştirir. 
        (allocate_relay_uavs_line'da kullanılır)
        """
        free_uavs = [uav for uav in self.uavs if uav.state == "Free"]
        if self.target_eval_mode.get() == "revisit":
            available_goals = [goal for goal in self.goals if goal.state in ("Free", "Visited")]
        else:
            available_goals = [goal for goal in self.goals if goal.state == "Free"]

        if not free_uavs or not available_goals:
            return None

        if self.target_eval_mode.get() == "revisit":
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

            max_valuation, best_uav, best_goal = max(valuations, key=lambda x: x[0])
        else:
            valuations = []
            for uav in free_uavs:
                for goal in available_goals:
                    dist = Distance.distance_between(uav, goal)
                    valuation = 1 / dist if dist != 0 else float("inf")
                    valuations.append((valuation, uav, goal))

            if not valuations:
                return None

            max_valuation, best_uav, best_goal = max(valuations, key=lambda x: x[0])

        best_uav.target = best_goal
        best_uav.state = "Ground_Leader"
        best_goal.state = "Assigned"
        return best_uav

    def get_time_since_last_visit(self, goal: Goal):
        current_time = time.time()
        if goal.last_visited_time is not None:
            return max(current_time - goal.last_visited_time, 0)
        return float("inf")

    def calculate_number_of_relays(self, pos_uav: Vector, pos_station: Vector) -> int:
        distance = Distance.distance_between_positions(pos_uav, pos_station)
        n_relay = max(int(np.ceil(distance / (0.9 * self.comm_thr))) - 1, 0)
        return n_relay

    def allocate_relay_uavs_line(self, leader_uav: UAV, n_relay: int):
        """
        Orijinal allocate_relay_uavs_line örneği, Hungarian assignment yapar ve 
        UAV'leri optimal olarak yerleştirir. 
        Bu örnekte 'bouncing'i azaltmak için, her UAV'ye 'Relay' rolü verip, 
        target_position'ları belirliyoruz. 
        """
        positions = []
        for i in range(1, n_relay + 2):
            factor = i / (n_relay + 1)
            relay_pos = self.ground.pos + (leader_uav.target.pos - self.ground.pos) * factor
            positions.append(relay_pos)

        candidate_uavs = [u for u in self.uavs if u.state == "Free"]
        if leader_uav not in candidate_uavs:
            candidate_uavs.append(leader_uav)

        if len(candidate_uavs) < len(positions):
            print("Not enough UAVs to fill all relay positions!")
            return

        cost_matrix = np.zeros((len(candidate_uavs), len(positions)), dtype=float)
        for i, uav in enumerate(candidate_uavs):
            for j, pos in enumerate(positions):
                dist = Distance.distance_between_positions(uav.pos, pos)
                cost_matrix[i, j] = dist

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Tüm candidate UAV'leri sıfırla
        for u in candidate_uavs:
            u.delete_my_slave_list()
            u.state = "Free"
            if hasattr(u, "target_position"):
                del u.target_position

        final_pos_index = len(positions) - 1
        new_leader = None

        for i, uav_index in enumerate(row_ind):
            chosen_pos_index = col_ind[i]
            chosen_uav = candidate_uavs[uav_index]
            chosen_pos = positions[chosen_pos_index]

            if chosen_pos_index == final_pos_index:
                chosen_uav.state = "Ground_Leader"
                chosen_uav.target_position = chosen_pos
                new_leader = chosen_uav
            else:
                chosen_uav.state = "Relay"
                chosen_uav.target_position = chosen_pos
                chosen_uav.my_leader = None

        if new_leader:
            new_leader.my_slave_list.clear()
            for u in candidate_uavs:
                if u is not new_leader and u.state == "Relay":
                    u.my_leader = new_leader
                    new_leader.my_slave_list.add(u)

        print("Hungarian assignment done for relay positions.")
        if new_leader and new_leader != leader_uav:
            print(f"Leader changed! New leader: UAV {new_leader.uav_no}")
        else:
            print(f"Leader stays the same (UAV {leader_uav.uav_no}).")
