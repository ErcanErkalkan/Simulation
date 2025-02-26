import time
import os
import sys

class SimulationLogger:
    """
    A simple logger to track key events in the simulation.
    Logs network stability, UAV assignments, relay interventions, and goal progress.
    """

    LOG_FILE = "simulation_log.txt"

    def __init__(self):
        """Initialize logger and clear old logs."""
        self.start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.log_entries = []
        self.clear_log()
        self.log(f"Simulation started at {self.start_time}\n")

    def log(self, message: str):
        """Log a message with a timestamp, ensuring UTF-8 encoding."""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        log_entry = f"[{timestamp}] {message}"
        
        # ✅ Prevent encoding issues in Windows terminal
        safe_log_entry = log_entry.encode("utf-8", "ignore").decode("utf-8")

        # ✅ Print safely (UTF-8 + force flush)
        print(safe_log_entry, file=sys.stdout, flush=True)

        self.log_entries.append(safe_log_entry)
        self.save_to_file(safe_log_entry)

    def save_to_file(self, log_entry: str):
        """Save log entries to a file with UTF-8 encoding."""
        with open(self.LOG_FILE, "a", encoding="utf-8") as f:
            f.write(log_entry + "\n")

    def clear_log(self):
        """Clear previous log file before starting a new simulation."""
        if os.path.exists(self.LOG_FILE):
            os.remove(self.LOG_FILE)

    def summary(self):
        """Summarize the simulation at the end."""
        total_events = len(self.log_entries)
        self.log(f"\nSimulation Summary:\nTotal Events Logged: {total_events}")
