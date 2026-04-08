import threading
import time
from typing import List

from simulation_app.domain.uav import UAV

class UAVThread(threading.Thread):
    """
    Her bir UAV için iş parçacığı. 
    'Merkez' karar mekanizması SimulationEngine'de olsa da, 
    bu thread UAV'nin hareketlerini periyodik olarak çağırır.
    """
    def __init__(self, 
                 uav: UAV,
                 update_interval: float = 0.1):
        super().__init__()
        self.uav = uav
        self.update_interval = update_interval
        self.running = False

    def run(self):
        self.running = True
        print(f"[UAVThread] UAV{self.uav.uav_no} started.")
        while self.running:
            # Thread döngüsü:
            # Bu UAV'nin state'ine göre hareket et
            if self.uav.state in ["Leader", "Ground_Leader"] and self.uav.target:
                self.uav.move_to_target()
            elif self.uav.state == "Relay" and hasattr(self.uav, "target_position"):
                self.uav.move_to_position(self.uav.target_position)
            elif self.uav.state == "Slave" and self.uav.my_leader:
                self.uav.move_to_leader()
            # vs. (Free ise bekliyor)
            
            time.sleep(self.update_interval)
        print(f"[UAVThread] UAV{self.uav.uav_no} stopped.")

    def stop(self):
        self.running = False
