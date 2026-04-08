import json
import queue
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from simulation_app.domain.vector import Vector
from simulation_app.hardware.co_drone import CoDroneDependencyError, co_Drone
from simulation_app.hardware.real_drone_thread import RealDroneThread
from simulation_app.support.functions import MatrixOperation
from simulation_app.support.generate_uav import GenerateUAV
from simulation_core.config import SimulationConfig
from simulation_core.engine import SimulationEngine


class MainWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Heterogeneous Unmanned Network Simulation Environment V01")
        self.geometry("1381x763")
        self.resizable(False, False)

        self.simulation_engine = SimulationEngine()
        self.simulation_engine.register_uav_type(co_Drone)

        self.real_drone_commands = queue.Queue()
        self.real_drone_thread = None
        self.algorithm_name_by_label = {}

        self.initialize_components()
        self.simulation_engine.set_config_provider(self.build_simulation_config)
        self.populate_algorithms()
        self.update_loop()

    def initialize_components(self):
        self.canvas = tk.Canvas(self, width=960, height=760, bg="white")
        self.canvas.place(x=0, y=0)

        self.groupBox1 = tk.Frame(self, borderwidth=2, relief="groove")
        self.groupBox1.place(x=968, y=0, width=399, height=749)

        self.ResetButton = tk.Button(
            self.groupBox1,
            text="Reset",
            font=("Microsoft Sans Serif", 15),
            command=self.reset_simulation,
        )
        self.ResetButton.place(x=306, y=690, width=75, height=35)

        self.LoadButton = tk.Button(
            self.groupBox1,
            text="Load",
            font=("Microsoft Sans Serif", 15),
            command=self.load_from_file,
        )
        self.LoadButton.place(x=231, y=690, width=75, height=35)

        self.SaveButton = tk.Button(
            self.groupBox1,
            text="Save",
            font=("Microsoft Sans Serif", 15),
            command=self.save_to_file,
        )
        self.SaveButton.place(x=156, y=690, width=75, height=35)

        self.groupBox7 = tk.LabelFrame(
            self.groupBox1, text="Add Goal", font=("Microsoft Sans Serif", 15)
        )
        self.groupBox7.place(x=7, y=602, width=385, height=81)

        tk.Label(self.groupBox7, text="X").place(x=7, y=15)
        self.textBox8 = tk.Entry(self.groupBox7)
        self.textBox8.place(x=42, y=15, width=66)

        tk.Label(self.groupBox7, text="Y").place(x=122, y=15)
        self.textBox7 = tk.Entry(self.groupBox7)
        self.textBox7.place(x=158, y=15, width=66)

        self.GoalAdd = tk.Button(self.groupBox7, text="Add", command=self.add_goal)
        self.GoalAdd.place(x=240, y=5, width=114, height=39)

        self.groupBox6 = tk.LabelFrame(
            self.groupBox1, text="Add UAV", font=("Microsoft Sans Serif", 15)
        )
        self.groupBox6.place(x=7, y=515, width=385, height=81)

        tk.Label(self.groupBox6, text="X").place(x=7, y=15)
        self.textBox6 = tk.Entry(self.groupBox6)
        self.textBox6.place(x=42, y=15, width=66)

        tk.Label(self.groupBox6, text="Y").place(x=122, y=15)
        self.textBox5 = tk.Entry(self.groupBox6)
        self.textBox5.place(x=158, y=15, width=66)

        self.UAVAdd = tk.Button(self.groupBox6, text="Add", command=self.add_uav)
        self.UAVAdd.place(x=240, y=5, width=114, height=39)

        self.ReStartButton = tk.Button(
            self.groupBox1,
            text="Restart",
            font=("Microsoft Sans Serif", 15),
            command=self.reset_simulation,
        )
        self.ReStartButton.place(x=82, y=690, width=75, height=35)

        self.StartButton = tk.Button(
            self.groupBox1,
            text="Start",
            font=("Microsoft Sans Serif", 15),
            command=self.start_simulation,
        )
        self.StartButton.place(x=7, y=690, width=75, height=35)

        self.groupBox5 = tk.LabelFrame(
            self.groupBox1, text="Generate Ground", font=("Microsoft Sans Serif", 15)
        )
        self.groupBox5.place(x=7, y=427, width=385, height=81)

        tk.Label(self.groupBox5, text="X").place(x=7, y=15)
        self.textBox3 = tk.Entry(self.groupBox5)
        self.textBox3.place(x=42, y=15, width=66)

        tk.Label(self.groupBox5, text="Y").place(x=122, y=15)
        self.textBox4 = tk.Entry(self.groupBox5)
        self.textBox4.place(x=158, y=15, width=66)

        self.GroundGenerate = tk.Button(
            self.groupBox5, text="Generate", command=self.generate_ground
        )
        self.GroundGenerate.place(x=240, y=5, width=114, height=39)

        self.groupBox4 = tk.LabelFrame(
            self.groupBox1, text="Generate Goal", font=("Microsoft Sans Serif", 15)
        )
        self.groupBox4.place(x=7, y=339, width=385, height=81)

        tk.Label(self.groupBox4, text="Goal Number").place(x=7, y=15)
        self.textBox2 = tk.Entry(self.groupBox4)
        self.textBox2.place(x=134, y=15, width=98)

        self.GoalGenerate = tk.Button(
            self.groupBox4, text="Generate", command=self.generate_goals
        )
        self.GoalGenerate.place(x=240, y=5, width=114, height=39)

        self.groupBox3 = tk.LabelFrame(
            self.groupBox1, text="Generate UAV", font=("Microsoft Sans Serif", 15)
        )
        self.groupBox3.place(x=7, y=252, width=385, height=81)

        tk.Label(self.groupBox3, text="UAV Number").place(x=7, y=5)
        self.textBox1 = tk.Entry(self.groupBox3)
        self.textBox1.place(x=134, y=5, width=98)

        self.use_co_drone_var = tk.BooleanVar(value=False)
        self.co_drone_check = tk.Checkbutton(
            self.groupBox3,
            text="Use CoDrone",
            variable=self.use_co_drone_var,
        )
        self.co_drone_check.place(x=10, y=25)

        self.UAVGenerate = tk.Button(
            self.groupBox3, text="Generate", command=self.generate_uavs
        )
        self.UAVGenerate.place(x=240, y=5, width=114, height=39)

        self.groupBox2 = tk.LabelFrame(
            self.groupBox1,
            text="Connection Threshold",
            font=("Microsoft Sans Serif", 15),
        )
        self.groupBox2.place(x=7, y=14, width=385, height=100)

        tk.Label(self.groupBox2, text="Threshold").place(x=10, y=20)
        self.threshold_scale = tk.Scale(
            self.groupBox2,
            from_=100.0,
            to=300.0,
            resolution=0.1,
            orient="horizontal",
            length=200,
            command=self.update_connection_threshold,
        )
        self.threshold_scale.set(self.simulation_engine.comm_thr)
        self.threshold_scale.place(x=100, y=10)

        self.groupBox8 = tk.LabelFrame(
            self.groupBox1,
            text="Target Evaluation",
            font=("Microsoft Sans Serif", 15),
        )
        self.groupBox8.place(x=7, y=125, width=385, height=120)

        self.target_eval_mode = tk.StringVar(value="single_visit")

        def update_threshold_entries(*args):
            state = "normal" if self.target_eval_mode.get() == "revisit" else "disabled"
            self.threshold1_entry.config(state=state)
            self.threshold2_entry.config(state=state)
            self.simulation_time_entry.config(state=state)

        self.target_eval_mode.trace_add("write", update_threshold_entries)

        tk.Radiobutton(
            self.groupBox8,
            text="Single Visit",
            variable=self.target_eval_mode,
            value="single_visit",
        ).place(x=10, y=10)

        tk.Radiobutton(
            self.groupBox8,
            text="Revisit",
            variable=self.target_eval_mode,
            value="revisit",
        ).place(x=150, y=10)

        tk.Label(self.groupBox8, text="Algorithm").place(x=10, y=44)
        self.algorithm_combo = ttk.Combobox(
            self.groupBox8, state="readonly", width=28
        )
        self.algorithm_combo.place(x=82, y=42)
        self.algorithm_combo.bind("<<ComboboxSelected>>", self.on_algorithm_selected)

        tk.Label(self.groupBox8, text="T1").place(x=10, y=80)
        self.threshold1_entry = tk.Entry(self.groupBox8)
        self.threshold1_entry.place(x=32, y=80, width=40)
        self.threshold1_entry.insert(0, "10")

        tk.Label(self.groupBox8, text="T2").place(x=82, y=80)
        self.threshold2_entry = tk.Entry(self.groupBox8)
        self.threshold2_entry.place(x=104, y=80, width=40)
        self.threshold2_entry.insert(0, "30")

        tk.Label(self.groupBox8, text="Sim Time").place(x=160, y=80)
        self.simulation_time_entry = tk.Entry(self.groupBox8)
        self.simulation_time_entry.place(x=226, y=80, width=50)
        self.simulation_time_entry.insert(0, "100")

        tk.Label(self.groupBox8, text="Speed").place(x=286, y=80)
        self.uav_speed_entry = tk.Entry(self.groupBox8)
        self.uav_speed_entry.place(x=334, y=80, width=36)
        self.uav_speed_entry.insert(0, "5")

        update_threshold_entries()

    def populate_algorithms(self) -> None:
        metadata = self.simulation_engine.get_algorithm_metadata()
        self.algorithm_name_by_label = {
            item["display_name"]: item["name"] for item in metadata
        }
        labels = list(self.algorithm_name_by_label)
        self.algorithm_combo["values"] = labels

        current_name = self.simulation_engine.get_current_algorithm_name()
        current_label = next(
            (
                item["display_name"]
                for item in metadata
                if item["name"] == current_name
            ),
            labels[0] if labels else "",
        )
        if current_label:
            self.algorithm_combo.set(current_label)

    def on_algorithm_selected(self, event=None) -> None:
        label = self.algorithm_combo.get()
        algorithm_name = self.algorithm_name_by_label.get(label)
        if algorithm_name:
            self.stop_active_simulation()
            self.simulation_engine.set_algorithm(algorithm_name)

    def build_simulation_config(self) -> SimulationConfig:
        return SimulationConfig(
            target_eval_mode=self.target_eval_mode.get(),
            threshold1=float(self.threshold1_entry.get()),
            threshold2=float(self.threshold2_entry.get()),
            simulation_time=float(self.simulation_time_entry.get()),
            uav_speed=float(self.uav_speed_entry.get()),
        )

    def update_connection_threshold(self, value):
        try:
            self.simulation_engine.comm_thr = float(value)
            self.draw_canvas()
        except ValueError:
            messagebox.showerror("Error", "Invalid threshold value!")

    def add_goal(self):
        try:
            x = int(self.textBox8.get())
            y = int(self.textBox7.get())
            self.stop_active_simulation()
            self.simulation_engine.add_goal(x, y)
            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"Goal {len(self.simulation_engine.goals)} added at ({x}, {y})."
            )
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input for X or Y.")

    def generate_ground(self):
        try:
            x = int(self.textBox3.get())
            y = int(self.textBox4.get())
            self.stop_active_simulation()
            self.simulation_engine.generate_ground(x, y)
            self.draw_canvas()
            messagebox.showinfo("Success", f"Ground generated at ({x}, {y}).")
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input for X or Y.")

    def generate_goals(self):
        try:
            goal_count = int(self.textBox2.get())
            self.stop_active_simulation()
            self.simulation_engine.generate_goals(
                goal_count, self.canvas.winfo_width(), self.canvas.winfo_height()
            )
            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"{goal_count} goals generated successfully!"
            )
        except ValueError:
            messagebox.showerror("Error", "Invalid input! Please enter a numeric value.")

    def generate_uavs(self):
        try:
            uav_count = int(self.textBox1.get())
            if uav_count <= 0:
                messagebox.showerror("Error", "UAV number must be a positive integer.")
                return
            self.stop_active_simulation()

            self.simulation_engine.uavs = []
            if self.use_co_drone_var.get():
                co_uav = co_Drone(
                    pos=Vector(50, 50),
                    uav_no=1,
                    ground=self.simulation_engine.ground,
                    command_queue=self.real_drone_commands,
                )
                self.simulation_engine.add_existing_uav(co_uav)

                remaining_count = uav_count - 1
                if remaining_count > 0:
                    normal_uavs = GenerateUAV.run(
                        remaining_count,
                        self.simulation_engine.comm_thr,
                        ground=self.simulation_engine.ground,
                        canvas_width=self.canvas.winfo_width(),
                        canvas_height=self.canvas.winfo_height(),
                    )
                    for uav in normal_uavs:
                        self.simulation_engine.add_existing_uav(uav)
            else:
                self.simulation_engine.generate_uavs(
                    uav_count, self.canvas.winfo_width(), self.canvas.winfo_height()
                )

            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"{uav_count} UAVs generated successfully!"
            )
        except ValueError:
            messagebox.showerror("Error", "Invalid input! Please enter a numeric value.")

    def add_uav(self):
        try:
            x = int(self.textBox6.get())
            y = int(self.textBox5.get())
            self.stop_active_simulation()
            self.simulation_engine.add_uav(x, y)
            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"UAV {len(self.simulation_engine.uavs)} added at ({x}, {y})."
            )
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input for X or Y.")

    def draw_canvas(self):
        self.canvas.delete("all")

        if self.simulation_engine.ground:
            self.simulation_engine.ground.draw(self.canvas)

        for goal in self.simulation_engine.goals:
            goal.draw(self.canvas)

        for uav in self.simulation_engine.uavs:
            uav.draw(self.canvas)

        self.draw_communication_lines()

    def reset_simulation(self):
        self.stop_active_simulation()
        self.simulation_engine.reset_simulation()
        self.draw_canvas()
        messagebox.showinfo("Reset", "Simulation has been reset.")

    def save_to_file(self):
        file_path = filedialog.asksaveasfilename(
            title="Save Simulation Data",
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
        )
        if not file_path:
            return

        try:
            data = self.simulation_engine.to_dict()
            data["CommThreshold"] = self.simulation_engine.comm_thr
            data["Algorithm"] = self.simulation_engine.get_current_algorithm_name()
            config = self.build_simulation_config()
            data["TargetEvalMode"] = config.target_eval_mode
            data["Threshold1"] = config.threshold1
            data["Threshold2"] = config.threshold2
            data["SimulationTime"] = config.simulation_time
            data["UavSpeed"] = config.uav_speed
            with open(file_path, "w", encoding="utf-8") as file:
                json.dump(data, file, indent=4)
            messagebox.showinfo("Success", f"Simulation data saved to {file_path}.")
        except Exception as exc:
            messagebox.showerror("Error", f"Failed to save data: {exc}")

    def load_from_file(self):
        file_path = filedialog.askopenfilename(
            title="Load Simulation Data",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
        )
        if not file_path:
            return

        try:
            self.stop_active_simulation()
            with open(file_path, "r", encoding="utf-8") as file:
                data = json.load(file)

            if "CommThreshold" in data:
                self.threshold_scale.set(float(data["CommThreshold"]))
            if "TargetEvalMode" in data:
                self.target_eval_mode.set(str(data["TargetEvalMode"]))
            self.set_entry_value(self.threshold1_entry, data.get("Threshold1"))
            self.set_entry_value(self.threshold2_entry, data.get("Threshold2"))
            self.set_entry_value(self.simulation_time_entry, data.get("SimulationTime"))
            self.set_entry_value(self.uav_speed_entry, data.get("UavSpeed"))

            algorithm_name = data.get("Algorithm")
            if algorithm_name:
                self.simulation_engine.set_algorithm(algorithm_name)
                self.populate_algorithms()

            self.simulation_engine.load_from_dict(data)
            self.attach_runtime_uav_dependencies(
                self.simulation_engine.uavs, self.real_drone_commands
            )
            self.draw_canvas()
            messagebox.showinfo("Success", f"Simulation data loaded from {file_path}.")
        except Exception as exc:
            messagebox.showerror("Error", f"Failed to load data: {exc}")

    def draw_communication_lines(self):
        uavs = self.simulation_engine.uavs
        ground = self.simulation_engine.ground
        comm_thr = self.simulation_engine.comm_thr

        if not uavs:
            return

        adj_matrix = MatrixOperation.adj_matrix(uavs, ground)
        components = MatrixOperation.find_connected_components(adj_matrix, comm_thr)
        risky_links = MatrixOperation.find_risky_links(components, adj_matrix, comm_thr)

        for i, uav1 in enumerate(uavs):
            for j, uav2 in enumerate(uavs):
                if i < j and adj_matrix[i, j] <= comm_thr * 0.9:
                    self.canvas.create_line(
                        uav1.pos.x,
                        uav1.pos.y,
                        uav2.pos.x,
                        uav2.pos.y,
                        fill="green",
                        width=3,
                    )

        if ground:
            ground_idx = len(uavs)
            for i, uav in enumerate(uavs):
                if adj_matrix[i, ground_idx] <= comm_thr * 0.9:
                    self.canvas.create_line(
                        uav.pos.x,
                        uav.pos.y,
                        ground.pos.x,
                        ground.pos.y,
                        fill="green",
                        width=3,
                    )

        for u, v in risky_links:
            uav1 = uavs[u] if u < len(uavs) else ground
            uav2 = uavs[v] if v < len(uavs) else ground
            self.canvas.create_line(
                uav1.pos.x,
                uav1.pos.y,
                uav2.pos.x,
                uav2.pos.y,
                fill="red",
                width=3,
            )

    def start_real_drone_session(self) -> None:
        if not self.simulation_engine.uavs:
            return
        first_uav = self.simulation_engine.uavs[0]
        if not isinstance(first_uav, co_Drone):
            return
        if first_uav.drone is None:
            raise CoDroneDependencyError(
                "codrone_edu is not installed. CoDrone mode cannot be started."
            )
        if self.real_drone_thread and self.real_drone_thread.is_alive():
            return
        self.real_drone_thread = RealDroneThread(first_uav, self.real_drone_commands)
        self.real_drone_thread.start()
        self.real_drone_commands.put({"type": "TAKEOFF"})

    def stop_real_drone_session(self) -> None:
        if self.real_drone_thread and self.real_drone_thread.is_alive():
            try:
                self.real_drone_thread.handle_command({"type": "LAND"})
            except Exception:
                pass
            self.real_drone_thread.stop()
            self.real_drone_thread.join(timeout=2)
        self.real_drone_thread = None

    def stop_active_simulation(self) -> None:
        self.stop_real_drone_session()
        self.simulation_engine.stop_simulation()
        self.StartButton.config(text="Start")

    @staticmethod
    def attach_runtime_uav_dependencies(uavs, command_queue) -> None:
        for uav in uavs:
            if isinstance(uav, co_Drone):
                uav.command_queue = command_queue

    @staticmethod
    def set_entry_value(entry, value) -> None:
        if value is None:
            return
        previous_state = entry.cget("state")
        if previous_state == "disabled":
            entry.config(state="normal")
        entry.delete(0, tk.END)
        entry.insert(0, str(value))
        if previous_state == "disabled":
            entry.config(state=previous_state)

    def start_simulation(self):
        if not self.simulation_engine.simulation_running:
            try:
                self.simulation_engine.start_simulation()
                self.start_real_drone_session()
            except (ValueError, TypeError, CoDroneDependencyError) as exc:
                messagebox.showerror("Error", str(exc))
                self.simulation_engine.stop_simulation()
                return
            self.StartButton.config(text="Stop")
        else:
            self.stop_real_drone_session()
            self.simulation_engine.stop_simulation()
            self.StartButton.config(text="Start")

    def run(self):
        self.mainloop()

    def update_loop(self):
        if self.simulation_engine.simulation_running:
            try:
                stop_reason = self.simulation_engine.move_uavs()
            except (ValueError, TypeError) as exc:
                self.simulation_engine.stop_simulation()
                self.StartButton.config(text="Start")
                messagebox.showerror("Error", str(exc))
                stop_reason = None

            if stop_reason == "all_goals_visited":
                self.stop_real_drone_session()
                self.StartButton.config(text="Start")
                messagebox.showinfo("Simulation Complete", "All goals have been visited.")
            elif stop_reason == "time_elapsed":
                self.stop_real_drone_session()
                self.StartButton.config(text="Start")
                messagebox.showinfo("Simulation Complete", "Simulation time has elapsed.")

        self.draw_canvas()
        self.after(100, self.update_loop)


def main() -> None:
    app = MainWindow()
    app.run()


if __name__ == "__main__":
    main()
