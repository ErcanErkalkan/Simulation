import time
import threading
import math
from djitellopy import Tello
from vector import Vector

class RealDroneMoveThread(threading.Thread):
    """
    Continuously reads co_Drone UAV's pos/direction from the simulation 
    and commands the real drone (Tello) to match that position.
    """

    def __init__(self, tello_drone_uav, update_interval=1.5):
        """
        :param tello_drone_uav: co_Drone instance representing the real drone in the sim.
        :param update_interval: check/sync period in seconds.
        """
        super().__init__()
        self.tello_drone_uav = tello_drone_uav
        self.update_interval = update_interval
        self.running = False
        self.old_pos = Vector(tello_drone_uav.pos.x, tello_drone_uav.pos.y)

    def run(self):
        """Bağlantıyı başlatır ve dronun sanal simülasyona göre hareket etmesini sağlar."""
        # 1) Connect + takeoff
        self.tello_drone_uav.connect()
        print("[RealDroneMoveThread] Connected to Tello.")
        self.tello_drone_uav.takeoff()
        self.running = True

        while self.running:
            # 2) Simülasyondaki sanal UAV'nin yeni konumunu al
            sim_pos = self.tello_drone_uav.pos

            # 3) Hareket miktarını hesapla
            dx = sim_pos.x - self.old_pos.x  # X yönünde değişim (sağ-sol)
            dy = sim_pos.y - self.old_pos.y  # Y yönünde değişim (ileri-geri)

            # 4) Hareket eşiği (Çok küçük hareketleri engelle)
            if abs(dx) < 5: dx = 0
            if abs(dy) < 5: dy = 0

            # 5) Pitch ve Roll değerlerini normalize et (Tello için hız aralığı: -100 ile 100)
            pitch = max(-50, min(50, int(dy / 10)))  # İleri / geri hareket
            roll = max(-50, min(50, int(dx / 10)))   # Sağa / sola hareket

            print(f"[RealDroneMoveThread] dx={dx:.1f}, dy={dy:.1f} => pitch={pitch}, roll={roll}")

            # 6) Yeni hareketi uygula
            if pitch != 0 or roll != 0:
                self.tello_drone_uav.drone.send_rc_control(roll, pitch, 0, 0)  # Tello yön kontrolleri
                print("[RealDroneMoveThread] => move(1.5 sec) ...")
                time.sleep(1.5)  # 1.5 saniye hareket ettir

                # Dronu durdur
                self.tello_drone_uav.drone.send_rc_control(0, 0, 0, 0)

            # 7) Güncel pozisyonu kaydet
            self.old_pos = Vector(sim_pos.x, sim_pos.y)

            # 8) Bekleme süresi
            time.sleep(self.update_interval)

        # 9) Durdurma sinyali gelince drone’u indir
        print("[RealDroneMoveThread] Stop => land + disconnect.")
        self.tello_drone_uav.land()
        self.tello_drone_uav.disconnect()

    def stop(self):
        """Thread’i durdur ve drone’u indir."""
        self.running = False
