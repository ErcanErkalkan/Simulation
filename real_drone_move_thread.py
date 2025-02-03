import time
import threading
import math
from codrone_edu.drone import Drone
from vector import Vector

class RealDroneMoveThread(threading.Thread):
    """
    Continuously reads co_Drone UAV's pos/direction from the simulation 
    and commands the real drone to match that position+heading.
    """

    def __init__(self, co_drone_uav, update_interval=1.0):
        """
        :param co_drone_uav: co_Drone instance representing the real drone in the sim.
        :param update_interval: check/sync period in seconds.
        """
        super().__init__()
        self.co_drone_uav = co_drone_uav
        self.update_interval = update_interval
        self.running = False
        self.old_pos = Vector(co_drone_uav.pos.x, co_drone_uav.pos.y)

    def run(self):
        # 1) Connect + takeoff
        self.co_drone_uav.connect()
        print("[RealDroneMoveThread] Connected.")
        self.co_drone_uav.takeoff()
        self.running = True

        while self.running:
            # 2) Simülasyondaki sanal UAV'nin yeni konumunu al
            sim_pos = self.co_drone_uav.pos

            # 3) Hareket miktarını hesapla
            dx = sim_pos.x - self.old_pos.x  # X yönünde değişim (sağ-sol)
            dy = sim_pos.y - self.old_pos.y  # Y yönünde değişim (ileri-geri)

            # 4) Hareket eşiği (Çok küçük hareketleri engelle)
            #if abs(dx) < 10: dx = 0
            #if abs(dy) < 10: dy = 0

            # 5) Pitch ve Roll değerlerini normalize et
            pitch = max(-30, min(30, int(dy / 10)))  # İleri / geri hareket
            roll = max(-30, min(30, int(dx / 10)))   # Sağa / sola hareket

            print(f"[RealDroneMoveThread] dx={dx:.1f}, dy={dy:.1f} => pitch={pitch}, roll={roll}")

            # 6) Yeni hareketi uygula
            if pitch != 0 or roll != 0:
                self.co_drone_uav.drone.set_pitch(pitch)
                self.co_drone_uav.drone.set_roll(roll)
                print("[RealDroneMoveThread] => move(1) ...")
                self.co_drone_uav.drone.move(1)  # 1 saniye boyunca hareket ettir

            # 7) Güncel pozisyonu kaydet
            self.old_pos = Vector(sim_pos.x, sim_pos.y)

            # 8) Bekleme süresi
            time.sleep(self.update_interval)

        # 9) Durdurma sinyali gelince drone’u indir
        print("[RealDroneMoveThread] Stop => land + disconnect.")
        self.co_drone_uav.land()
        self.co_drone_uav.disconnect()

    def stop(self):
        """Thread’i durdur ve drone’u indir."""
        self.running = False
