import time
import threading

from simulation_app.domain.vector import Vector

class RealDroneThread(threading.Thread):
    """
    This thread handles commands (e.g., takeoff, land, move, etc.) for the co_Drone object 
    through a queue and processes them in a non-blocking manner to manage real drone flights.
    """

    def __init__(self, co_drone_uav, command_queue, update_interval=0.1):
        """
        :param co_drone_uav: An object from the co_Drone class (UAV for real flight).
        :param command_queue: queue.Queue() - stores commands coming from the main program.
        :param update_interval: The interval at which the thread checks for new commands (in seconds).
        """
        super().__init__()
        self.co_drone_uav = co_drone_uav
        self.command_queue = command_queue
        self.update_interval = update_interval
        self.running = False

    def run(self):
        self.running = True
        # When the thread starts, initialize the drone connection (if not already connected).
        self.co_drone_uav.connect()
        print("[RealDroneThread] CoDrone connected.")

        while self.running:
            if not self.command_queue.empty():
                command = self.command_queue.get()
                self.handle_command(command)

            # Wait for the update_interval and then check the queue again
            time.sleep(self.update_interval)

        # When the thread stops, perform cleanup: shut down/land/disconnect the drone
        self.co_drone_uav.disconnect()
        print("[RealDroneThread] CoDrone disconnected.")

    def handle_command(self, command):
        """
        Parses incoming commands by their type and executes them on the co_drone_uav object.
        Example command formats:
         {"type": "TAKEOFF"}
         {"type": "LAND"}
         {"type": "MOVE_TO", "target_pos": Vector(x, y), "velocity": 0.5}
        """
        cmd_type = command.get("type")
        if cmd_type == "TAKEOFF":
            print("[RealDroneThread] TAKEOFF command.")
            self.co_drone_uav.takeoff()

        elif cmd_type == "LAND":
            print("[RealDroneThread] LAND command.")
            self.co_drone_uav.land()

        elif cmd_type == "MOVE_TO":
            target_pos = command.get("target_pos", Vector(0, 0))
            velocity = command.get("velocity", 0.5)
            print(f"[RealDroneThread] MOVE_TO command -> pos={target_pos}, vel={velocity}")
            self.co_drone_uav.move_to_real(target_pos, velocity)

        else:
            print(f"[RealDroneThread] Unknown command: {cmd_type}")

    def stop(self):
        """
        Called by the main program to stop the thread's execution.
        """
        self.running = False
