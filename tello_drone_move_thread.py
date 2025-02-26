import time
import threading
from djitellopy import Tello
from vector import Vector

class TelloDroneMoveThread(threading.Thread):
    """
    Continuously reads the simulated UAV's position and commands the real Tello drone
    to move accordingly.
    """
    def __init__(self, tello_drone_uav, update_interval=0.5):
        """
        :param tello_drone_uav: Tello_Drone instance representing the real drone in the simulation.
        :param update_interval: period (in seconds) to update the drone's movement.
        """
        super().__init__()
        self.tello_drone_uav = tello_drone_uav
        self.update_interval = update_interval
        self.running = False
        # Save the initial simulated position
        self.old_pos = Vector(tello_drone_uav.pos.x, tello_drone_uav.pos.y)

    def run(self):
        # 1) Connect and takeoff
        try:
            self.tello_drone_uav.connect()
            print("[TelloDroneMoveThread] Connected to Tello.")
            self.tello_drone_uav.takeoff()
            print("[TelloDroneMoveThread] Tello has taken off.")
        except Exception as e:
            print(f"[TelloDroneMoveThread] Error during connect/takeoff: {e}")
            return

        self.running = True

        while self.running:
            # 2) Get the current simulated position
            sim_pos = self.tello_drone_uav.pos

            # 3) Compute the movement delta
            dx = sim_pos.x - self.old_pos.x  # left/right difference
            dy = sim_pos.y - self.old_pos.y  # forward/backward difference

            # 4) Filter out very small movements
            if abs(dx) < 5:
                dx = 0
            if abs(dy) < 5:
                dy = 0

            # 5) Compute raw velocities using a scaling factor
            # Using a smaller divisor for increased sensitivity
            raw_pitch = int(dy / 5)  # forward/backward velocity
            raw_roll  = int(dx / 5)  # left/right velocity

            # 6) Apply a minimum speed if movement is detected
            min_speed = 10
            if raw_pitch != 0 and abs(raw_pitch) < min_speed:
                raw_pitch = min_speed if raw_pitch > 0 else -min_speed
            if raw_roll != 0 and abs(raw_roll) < min_speed:
                raw_roll = min_speed if raw_roll > 0 else -min_speed

            # 7) Clamp the velocities to Tello's allowed range (-50 to 50)
            pitch = max(-50, min(50, raw_pitch))
            roll  = max(-50, min(50, raw_roll))

            print(f"[TelloDroneMoveThread] dx={dx:.1f}, dy={dy:.1f} => pitch={pitch}, roll={roll}")

            # 8) Send RC command if there's movement; otherwise, send zeros.
            try:
                self.tello_drone_uav.drone.send_rc_control(roll, pitch, 0, 0)
                print(f"[TelloDroneMoveThread] RC command sent: roll={roll}, pitch={pitch}")
            except Exception as e:
                print(f"[TelloDroneMoveThread] Error sending RC command: {e}")

            # 9) Update the previous position
            self.old_pos = Vector(sim_pos.x, sim_pos.y)

            # 10) Wait for the next update
            time.sleep(self.update_interval)

        # 11) On stop signal, land and disconnect
        print("[TelloDroneMoveThread] Stop signal received. Landing and disconnecting...")
        try:
            self.tello_drone_uav.land()
            self.tello_drone_uav.disconnect()
            print("[TelloDroneMoveThread] Tello landed and disconnected.")
        except Exception as e:
            print(f"[TelloDroneMoveThread] Error during landing/disconnect: {e}")

    def stop(self):
        """Signal the thread to stop and initiate landing/disconnect."""
        self.running = False
