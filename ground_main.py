# main_window.py
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import queue

from simulation_engine import SimulationEngine
from functions import MatrixOperation
from uav import UAV
from goal import Goal
from ground import Ground
from vector import Vector

from co_drone import co_Drone
from real_drone_move_thread import RealDroneMoveThread
from generate_uav import GenerateUAV

class MainWindow(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Heterogeneous Unmanned Network Simulation Environment V01")
        self.geometry("1381x763")
        self.resizable(False, False)
        
        # Ana motor
        self.simulation_engine = SimulationEngine()
        self.simulation_running = False
        
        self.real_drone_thread = None
        
        self.initialize_components()
        
        # Link GUI elements to simulation engine
        self.simulation_engine.target_eval_mode = self.target_eval_mode
        self.simulation_engine.threshold1_entry = self.threshold1_entry
        self.simulation_engine.threshold2_entry = self.threshold2_entry
        self.simulation_engine.simulation_time_entry = self.simulation_time_entry

    def initialize_components(self):
        # Canvas
        self.canvas = tk.Canvas(self, width=960, height=760, bg="white")
        self.canvas.place(x=0, y=0)

        # GroupBox1
        self.groupBox1 = tk.Frame(self, borderwidth=2, relief="groove")
        self.groupBox1.place(x=968, y=0, width=399, height=749)

        # Reset, Load, Save Buttons
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

        # GroupBox7 - Add Goal
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

        self.GoalAdd = tk.Button(
            self.groupBox7, text="Add", command=self.add_goal)
        self.GoalAdd.place(x=240, y=5, width=114, height=39)

        # GroupBox6 - Add UAV
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

        self.UAVAdd = tk.Button(
            self.groupBox6, text="Add", command=self.add_uav)
        self.UAVAdd.place(x=240, y=5, width=114, height=39)

        # Restart and Start Buttons
        self.ReStartButton = tk.Button(
            self.groupBox1, text="Restart", font=("Microsoft Sans Serif", 15)
        )
        self.ReStartButton.place(x=82, y=690, width=75, height=35)

        self.StartButton = tk.Button(
            self.groupBox1,
            text="Start",
            font=("Microsoft Sans Serif", 15),
            command=self.start_simulation,
        )
        self.StartButton.place(x=7, y=690, width=75, height=35)

        # GroupBox5 - Generate Ground
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

        # GroupBox4 - Generate Goal
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

        # GroupBox3 - Generate UAV
        self.groupBox3 = tk.LabelFrame(
            self.groupBox1, text="Generate UAV", font=("Microsoft Sans Serif", 15)
        )
        self.groupBox3.place(x=7, y=252, width=385, height=81)

        tk.Label(self.groupBox3, text="UAV Number").place(x=7, y=5)
        self.textBox1 = tk.Entry(self.groupBox3)
        self.textBox1.place(x=134, y=5, width=98)

        self.use_co_drone_var = tk.BooleanVar(value=False)  # Varsayılan False
        self.co_drone_check = tk.Checkbutton(
            self.groupBox3,
            text="Use CoDrone",
            variable=self.use_co_drone_var
        )
        self.co_drone_check.place(x=10, y=25)

        self.UAVGenerate = tk.Button(
            self.groupBox3, text="Generate", command=self.generate_uavs
        )
        self.UAVGenerate.place(x=240, y=5, width=114, height=39)

        # GroupBox2 - Connection Threshold
        self.groupBox2 = tk.LabelFrame(
            self.groupBox1, text="Connection Threshold", font=("Microsoft Sans Serif", 15)
        )
        self.groupBox2.place(x=7, y=14, width=385, height=100)

        tk.Label(self.groupBox2, text="Threshold").place(x=10, y=20)

        # Kaydırma çubuğu
        self.threshold_scale = tk.Scale(
            self.groupBox2,
            from_=100.0,  # Minimum threshold
            to=300.0,   # Maximum threshold
            resolution=0.1,
            orient="horizontal",
            length=200,
            command=self.update_connection_threshold,
        )
        # Varsayılan threshold değeri
        self.threshold_scale.set(self.simulation_engine.comm_thr)
        self.threshold_scale.place(x=100, y=10)

        # Target Evaluation Selection
        self.groupBox8 = tk.LabelFrame(
            self.groupBox1, text="Target Evaluation", font=("Microsoft Sans Serif", 15))
        self.groupBox8.place(x=7, y=125, width=385, height=120)
        # Single Visit and Revisit Options
        self.target_eval_mode = tk.StringVar(value="single_visit")

        def update_threshold_entries(*args):
            if self.target_eval_mode.get() == "revisit":
                self.threshold1_entry.config(state="normal")
                self.threshold2_entry.config(state="normal")
                self.simulation_time_entry.config(state="normal")
            else:
                self.threshold1_entry.config(state="disabled")
                self.threshold2_entry.config(state="disabled")
                self.simulation_time_entry.config(state="disabled")

        # Bind the update function to changes in the target_eval_mode variable
        self.target_eval_mode.trace_add("write", update_threshold_entries)

        tk.Radiobutton(
            self.groupBox8,
            text="Single Visit",
            variable=self.target_eval_mode,
            value="single_visit"
        ).place(x=10, y=10)

        tk.Radiobutton(
            self.groupBox8,
            text="Revisit",
            variable=self.target_eval_mode,
            value="revisit"
        ).place(x=150, y=10)

        # Time Threshold Settings (for Revisit)
        tk.Label(self.groupBox8, text="Threshold 1:").place(x=10, y=50)
        self.threshold1_entry = tk.Entry(self.groupBox8)
        self.threshold1_entry.place(x=82, y=50, width=40)
        self.threshold1_entry.insert(0, "10")  # Default value

        tk.Label(self.groupBox8, text="Threshold 2:").place(x=122, y=50)
        self.threshold2_entry = tk.Entry(self.groupBox8)
        self.threshold2_entry.place(x=192, y=50, width=40)
        self.threshold2_entry.insert(0, "30")  # Default value

        # Time Settings (for Revisit)
        tk.Label(self.groupBox8, text="Simulation Time:").place(x=232, y=50)
        self.simulation_time_entry = tk.Entry(self.groupBox8)
        self.simulation_time_entry.place(x=330, y=50, width=40)
        self.simulation_time_entry.insert(0, "100")  # Default value

        # Initially disable the threshold entries if 'Single Visit' is selected
        if self.target_eval_mode.get() == "single_visit":
            self.threshold1_entry.config(state="disabled")
            self.threshold2_entry.config(state="disabled")
            self.simulation_time_entry.config(state="disabled")

    # ------------------------------------------------------
    #  FONKSİYONLAR
    # ------------------------------------------------------
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
            self.simulation_engine.generate_ground(x, y)
            self.draw_canvas()
            messagebox.showinfo("Success", f"Ground generated at ({x}, {y}).")
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input for X or Y.")

    def generate_goals(self):
        try:
            goal_count = int(self.textBox2.get())
            self.simulation_engine.generate_goals(
                goal_count, self.canvas.winfo_width(), self.canvas.winfo_height()
            )
            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"{goal_count} goals generated successfully!"
            )
        except ValueError:
            messagebox.showerror(
                "Error", "Invalid input! Please enter a numeric value."
            )

    def generate_uavs(self):
        try:
            uav_count = int(self.textBox1.get())
            if uav_count <= 0:
                messagebox.showerror("Error", "UAV number must be a positive integer.")
                return

            # Tüm UAV'leri önce normal şekilde GenerateUAV ile oluşturuyoruz
            # Rastgele veya single connected component mantığı
            uavs_temp = GenerateUAV.run(
                count=uav_count,
                comm_thr=self.simulation_engine.comm_thr,
                ground=self.simulation_engine.ground,
                canvas_width=self.canvas.winfo_width(),
                canvas_height=self.canvas.winfo_height(),
            )

            if self.use_co_drone_var.get():
                # "Use CoDrone" seçiliyse, ilk UAV'i co_Drone tipine dönüştürüyoruz
                if len(uavs_temp) < 1:
                    messagebox.showwarning("Warning","No UAVs generated to replace with coDrone.")
                else:
                    # Mevcut ilk normal UAV'i al
                    old_uav = uavs_temp[0]
                    old_pos = old_uav.pos
                    old_dir = old_uav.direction
                    old_no  = old_uav.uav_no
                    old_ground = old_uav.ground

                    # co_Drone nesnesi
                    co_uav = co_Drone(
                        pos=old_pos,
                        direction=old_dir,
                        uav_no=old_no,
                        ground=old_ground,
                    )

                    # Listenin ilk elemanını co_uav ile değiştir
                    uavs_temp[0] = co_uav

            # Tüm UAV'leri simulation_engine’e ekle
            self.simulation_engine.uavs.extend(uavs_temp)

            # UAV numaralarını güncelle
            for i, uav in enumerate(self.simulation_engine.uavs, start=1):
                uav.uav_no = i

            # Çiz
            self.draw_canvas()
            messagebox.showinfo("Success", f"{uav_count} UAVs generated successfully!")
        except ValueError:
            messagebox.showerror("Error", "Invalid input! Please enter a numeric value.")


    def generate_uavs_(self):
        try:
            uav_count = int(self.textBox1.get())
            if uav_count <= 0:
                messagebox.showerror(
                    "Error", "UAV number must be a positive integer."
                )
                return
            
            # Eğer işaretli ise ilk UAV co_Drone olsun
            if self.use_co_drone_var.get():
                # co_Drone örneğini manuel oluşturup listeye ekliyoruz                                
                co_uav = co_Drone(
                    pos=Vector(50, 50),
                    uav_no=1,
                    ground=None,
                    command_queue=self.real_drone_commands  # önemli!
                )
                self.simulation_engine.uavs.append(co_uav)

                # Kalan UAV'leri normal oluşturmak için (uav_count - 1) gönderiyoruz
                remaining_count = uav_count - 1
                if remaining_count > 0:
                    normal_uavs = GenerateUAV.run(
                        remaining_count, self.simulation_engine.comm_thr,
                        ground=self.simulation_engine.ground,
                        canvas_width=self.canvas.winfo_width(),
                        canvas_height=self.canvas.winfo_height()
                    )
                    self.simulation_engine.uavs.extend(normal_uavs)
                    # UAV numaralarını güncelleyelim
                    for i, uav in enumerate(self.simulation_engine.uavs, start=1):
                        uav.uav_no = i
            else:
                self.simulation_engine.generate_uavs(
                    uav_count, self.canvas.winfo_width(), self.canvas.winfo_height()
                )
            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"{uav_count} UAVs generated successfully!"
            )
        except ValueError:
            messagebox.showerror(
                "Error", "Invalid input! Please enter a numeric value."
            )

    def add_uav(self):
        try:
            x = int(self.textBox6.get())
            y = int(self.textBox5.get())
            self.simulation_engine.add_uav(x, y)
            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"UAV {len(self.simulation_engine.uavs)} added at ({x}, {y})."
            )
        except ValueError:
            messagebox.showerror("Error", "Invalid numeric input for X or Y.")

    def draw_canvas(self):
        self.canvas.delete("all")

        # Draw the ground
        if self.simulation_engine.ground:
            self.simulation_engine.ground.draw(self.canvas)

        # Draw the goals
        for goal in self.simulation_engine.goals:
            goal.draw(self.canvas)

        # Draw the UAVs
        for uav in self.simulation_engine.uavs:
            uav.draw(self.canvas)

        # Draw communication lines
        self.draw_communication_lines()
  
    def reset_simulation(self):   
        self.simulation_engine = SimulationEngine()
        self.simulation_running = False
        # Thread ve komut kuyruğunu da sıfırlıyoruz
        self.real_drone_commands = queue.Queue()
        self.real_drone_thread = None
        
        self.initialize_components()
        # Link GUI elements to simulation engine
        self.simulation_engine.target_eval_mode = self.target_eval_mode
        self.simulation_engine.threshold1_entry = self.threshold1_entry
        self.simulation_engine.threshold2_entry = self.threshold2_entry
        self.simulation_engine.simulation_time_entry = self.simulation_time_entry
        
        messagebox.showinfo("Reset", "Simulation has been reset.")

    def save_to_file(self):
        file_path = filedialog.asksaveasfilename(
            title="Save Simulation Data",
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
        )

        if not file_path:
            return

        data = {
            "Ground": (
                {"X": self.simulation_engine.ground.pos.x,
                    "Y": self.simulation_engine.ground.pos.y}
                if self.simulation_engine.ground
                else None
            ),
            "UAVs": [{"X": uav.pos.x, "Y": uav.pos.y} for uav in self.simulation_engine.uavs],
            "Goals": [{"X": goal.pos.x, "Y": goal.pos.y} for goal in self.simulation_engine.goals],
        }

        try:
            with open(file_path, "w") as file:
                json.dump(data, file, indent=4)
            messagebox.showinfo(
                "Success", f"Simulation data saved to {file_path}."
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save data: {e}")

    def load_from_file(self):
        file_path = filedialog.askopenfilename(
            title="Load Simulation Data",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
        )

        if not file_path:
            return

        try:
            with open(file_path, "r") as file:
                data = json.load(file)

            # Load Ground
            if "Ground" in data and data["Ground"]:
                ground_data = data["Ground"]
                self.simulation_engine.generate_ground(
                    ground_data["X"], ground_data["Y"])

            # Load UAVs
            self.simulation_engine.uavs = []
            if "UAVs" in data and isinstance(data["UAVs"], list):
                for i, uav_data in enumerate(data["UAVs"], start=1):
                    uav = UAV(pos=Vector(
                        uav_data["X"], uav_data["Y"]), color="red")
                    uav.uav_no = i
                    self.simulation_engine.uavs.append(uav)

            # Load Goals
            self.simulation_engine.goals = []
            if "Goals" in data and isinstance(data["Goals"], list):
                for i, goal_data in enumerate(data["Goals"], start=1):
                    goal = Goal(
                        pos=Vector(goal_data["X"], goal_data["Y"]),
                        goal_no=i,
                        color="blue",
                    )
                    self.simulation_engine.goals.append(goal)

            self.draw_canvas()
            messagebox.showinfo(
                "Success", f"Simulation data loaded from {file_path}."
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load data: {e}")

    def draw_communication_lines(self):
        uavs = self.simulation_engine.uavs
        ground = self.simulation_engine.ground
        comm_thr = self.simulation_engine.comm_thr

        if not uavs:
            return

        adj_matrix = MatrixOperation.adj_matrix(uavs, ground)
        components = MatrixOperation.find_connected_components(
            adj_matrix, comm_thr
        )
        risky_links = MatrixOperation.find_risky_links(
            components, adj_matrix, comm_thr
        )

        # Draw green communication lines
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

        # Connect UAVs to ground if within threshold
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

        # Draw red risky links
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

    def move_uavs(self):
        if not self.simulation_running:
            return

        self.simulation_engine.move_uavs()
        self.draw_canvas()

        # Check conditions to stop simulation based on mode
        if self.target_eval_mode.get() == "single_visit":
            # Stop if all goals have been visited
            if all(goal.state == "Visited" for goal in self.simulation_engine.goals):
                self.simulation_running = False
                self.simulation_engine.simulation_running = False
                self.StartButton.config(text="Start")
                messagebox.showinfo("Simulation Complete",
                                    "All goals have been visited.")
                self.stop_simulation()  # Drone iner, simülasyon biter
                return

        elif self.target_eval_mode.get() == "revisit":
            self.simulation_engine.target_eval_mode = self.target_eval_mode
            self.simulation_engine.threshold1_entry = self.threshold1_entry
            self.simulation_engine.threshold2_entry = self.threshold2_entry
            self.simulation_engine.simulation_time_entry = self.simulation_time_entry
            # Stop if simulation time exceeds threshold
            elapsed_time = time.time() - self.simulation_engine.start_time
            max_simulation_time = float(self.simulation_time_entry.get())
            if elapsed_time >= max_simulation_time:
                self.simulation_running = False
                self.simulation_engine.simulation_running = False
                self.StartButton.config(text="Start")
                messagebox.showinfo("Simulation Complete",
                                    "Simulation time has elapsed.")
                self.stop_simulation()  # Drone iner, simülasyon biter
                return

        self.after(1, self.move_uavs)

    def start_simulation(self):
        if not self.simulation_running:
            # Eğer ilk UAV co_Drone ise, bağlan ve kalk
            if self.simulation_engine.uavs and isinstance(self.simulation_engine.uavs[0], co_Drone):
                co_drone_uav = self.simulation_engine.uavs[0]       
                self.real_drone_thread = RealDroneMoveThread(
                    co_drone_uav=co_drone_uav,
                    update_interval=1.0,
                )
                self.real_drone_thread.start()                
            else:
                print("[DEBUG] No co_Drone found to start real_drone_thread!")
            time.sleep(1)  # Drone'un havalanmasını bekleme süresi
            self.simulation_running = True
            self.simulation_engine.simulation_running = True
            self.simulation_engine.start_time = time.time()  # Initialize start time
            self.StartButton.config(text="Stop")
            # UAV'leri hareket ettirmeye başla
            self.move_uavs()
        else:
            self.simulation_running = False
            self.simulation_engine.simulation_running = False
            self.StartButton.config(text="Start")
            # Eğer ilk UAV co_Drone ise, in ve bağlantıyı kes
                    # Durdur RealDroneDirectThread
            if self.real_drone_thread:
                self.real_drone_thread.stop()
                self.real_drone_thread.join()
                self.real_drone_thread = None

    
    def stop_simulation(self):
        self.simulation_running = False
        self.simulation_engine.simulation_running = False

        # Durdur RealDroneDirectThread
        if self.real_drone_thread:
            self.real_drone_thread.stop()
            self.real_drone_thread.join()
            self.real_drone_thread = None

    def run(self):
        self.mainloop()


if __name__ == "__main__":
    app = MainWindow()
    app.run()
