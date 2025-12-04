"""Unitree Go2W Experimental Control - Final Optimized Version.

LOGIC VERIFICATION:
- ZigZag Path: INCLUDED (Sine wave math).
- Braking Curves: INCLUDED (Linear decel at end).
- Resume Smoothing: INCLUDED (Quadratic ease-in).
- Rotation: TRAPEZOIDAL (Physics-based, no overshoot).
- Shutdown: SAFE (Stop -> Sit -> Damp).

Usage:
  python3 go2_gait2.py eth0 --speed 0.75 --path linear_forward --gaze stop_rotate_gaze
"""

import sys
import time
import math
import select
import termios
import tty
import threading
import argparse
import numpy as np

from rich.live import Live
from rich.table import Table
from rich.layout import Layout
from rich.panel import Panel
from rich.console import Console
from rich import box

sys.path.append('/home/unitree/unitree_sdk2_python')

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    _SDK_AVAILABLE = True
except ImportError:
    print("Warning: Unitree SDK not found. Running in simulation.")
    _SDK_AVAILABLE = False

# --- Configuration ---
SPEED_OPTIONS = [0.75, 1.25, 1.75, 2.25]
PATH_OPTIONS = ['linear_forward', 'forward_zigzag']
GAZE_OPTIONS = ['no_stop', 'stop_gaze_forward', 'stop_rotate_gaze']

class TrapezoidalProfile:
    """Generates a smooth velocity curve to hit a target distance exactly."""
    def __init__(self, total_distance, max_speed=0.6, ramp_time=0.8):
        self.distance = abs(total_distance)
        self.direction = 1.0 if total_distance > 0 else -1.0
        self.max_speed = max_speed
        self.ramp_time = ramp_time
        
        # Calculate kinematics
        accel_dist = max_speed * ramp_time 
        
        if accel_dist > self.distance:
            # Triangle profile (Short move)
            self.ramp_time = self.distance / max_speed
            self.max_speed = self.distance / self.ramp_time
            self.cruise_time = 0.0
        else:
            # Trapezoid profile
            self.cruise_dist = self.distance - accel_dist
            self.cruise_time = self.cruise_dist / max_speed
            
        self.total_time = (2 * self.ramp_time) + self.cruise_time

    def get_velocity(self, current_time, start_time):
        t = current_time - start_time
        if t < 0: return 0.0, False
        if t >= self.total_time: return 0.0, True
        
        speed = 0.0
        if t < self.ramp_time:
            # Ramp Up
            progress = t / self.ramp_time
            speed = self.max_speed * progress
        elif t < (self.ramp_time + self.cruise_time):
            # Cruise
            speed = self.max_speed
        elif t < self.total_time:
            # Ramp Down
            decel_t = t - (self.ramp_time + self.cruise_time)
            progress = 1.0 - (decel_t / self.ramp_time)
            speed = self.max_speed * progress
            
        return speed * self.direction, False


class Go2EthernetControl:
    def __init__(self, network_interface="eth0"):
        if _SDK_AVAILABLE:
            ChannelFactoryInitialize(0, network_interface)
            self.client = SportClient()
            self.client.SetTimeout(10.0)
            self.client.Init()
        
        self.position = [0.0, 0.0, 0.35]
        self.yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0

    def stand_up(self):
        if not _SDK_AVAILABLE: return
        self.client.StandUp()
        time.sleep(1)
        self.client.BalanceStand()

    def sit_down(self):
        if not _SDK_AVAILABLE: return
        # Try normal sit first
        self.client.StandDown()
        # Fallback to Damp mode (limp) just in case
        time.sleep(0.2)
        self.client.Damp()

    def stop(self):
        self.current_vx, self.current_vy, self.current_omega = 0, 0, 0
        if _SDK_AVAILABLE: self.client.StopMove()

    def set_velocity(self, vx, vy, omega):
        # Low-level smoothing
        alpha = 0.2
        self.current_vx += alpha * (vx - self.current_vx)
        self.current_vy += alpha * (vy - self.current_vy)
        self.current_omega += alpha * (omega - self.current_omega)

        dt = 0.02
        self.position[0] += self.current_vx * dt
        self.position[1] += self.current_vy * dt
        self.yaw += self.current_omega * dt

        if _SDK_AVAILABLE:
            vx = np.clip(vx, -3.0, 3.0)
            vy = np.clip(vy, -0.5, 0.5)
            omega = np.clip(omega, -1.5, 1.5)
            self.client.Move(vx, vy, omega)


class ExperimentalController:
    def __init__(self, speed, path_type, gaze_type):
        self.target_speed = speed
        self.path_type = path_type
        self.gaze_type = gaze_type
        
        self.path_length = 16.0
        self.braking_distance = 2.5
        
        self.gaze_stop_position = 8.0
        self.gaze_stop_duration = 2.0
        self.gaze_rotate_pause = 4.0
        
        self.zigzag_start = 5.0
        self.zigzag_end = 11.0
        self.zigzag_cycles = 2.0
        self.zigzag_lateral_fraction = 0.5
        
        self.gaze_behavior_completed = False
        self.gaze_state = 'moving'
        self.state_start_time = 0.0
        self.resume_start_time = None
        self.resume_duration = 2.0
        
        self.last_speed = 0.0
        self.speed_smoothing = 0.2
        self.rot_profile = None

    def reset_state(self):
        self.last_speed = 0.0
        self.gaze_state = 'moving'
        self.state_start_time = None
        self.gaze_behavior_completed = False
        self.resume_start_time = None
        self.rot_profile = None

    def get_velocity_commands(self, distance_traveled, current_time, robot_yaw):
        msg = "Moving"
        
        # 1. Path Completion
        if distance_traveled >= self.path_length:
            return 0.0, 0.0, 0.0, True, "Trial Complete"
        
        vx, vy, omega = 0.0, 0.0, 0.0
        override_motion = False
        
        # 2. Gaze Logic (Overrides motion)
        if self.gaze_type == 'stop_rotate_gaze':
            if not self.gaze_behavior_completed:
                # Trigger
                if self.gaze_state == 'moving' and distance_traveled >= self.gaze_stop_position:
                    self.gaze_state = 'stopped'
                    self.state_start_time = current_time
                    override_motion = True
                    
                # State Machine
                elif self.gaze_state != 'moving' and self.gaze_state != 'resuming':
                    override_motion = True
                    
                    if self.gaze_state == 'stopped':
                        msg = "Stopping (Pre-Turn)"
                        if (current_time - self.state_start_time) > 1.0:
                            self.gaze_state = 'rotating_out'
                            self.state_start_time = current_time
                            self.rot_profile = TrapezoidalProfile(1.5708, max_speed=0.6, ramp_time=0.8)

                    elif self.gaze_state == 'rotating_out':
                        omega, done = self.rot_profile.get_velocity(current_time, self.state_start_time)
                        msg = f"Rotating 90° (O: {omega:.2f})"
                        if done:
                            self.gaze_state = 'paused'
                            self.state_start_time = current_time

                    elif self.gaze_state == 'paused':
                        msg = "Gazing (Paused)"
                        omega = 0.0
                        if (current_time - self.state_start_time) > self.gaze_rotate_pause:
                            self.gaze_state = 'rotating_in'
                            self.state_start_time = current_time
                            self.rot_profile = TrapezoidalProfile(-1.5708, max_speed=0.6, ramp_time=0.8)

                    elif self.gaze_state == 'rotating_in':
                        omega, done = self.rot_profile.get_velocity(current_time, self.state_start_time)
                        msg = f"Rotating Back (O: {omega:.2f})"
                        if done:
                            self.gaze_state = 'resuming'
                            self.gaze_behavior_completed = True
                            self.resume_start_time = current_time
                            override_motion = False

        elif self.gaze_type == 'stop_gaze_forward':
            if not self.gaze_behavior_completed:
                if self.gaze_state == 'moving' and distance_traveled >= self.gaze_stop_position:
                    self.gaze_state = 'stopped'
                    self.state_start_time = current_time
                if self.gaze_state == 'stopped':
                    override_motion = True
                    msg = "Stopped (Gaze Fwd)"
                    if (current_time - self.state_start_time) > self.gaze_stop_duration:
                        self.gaze_state = 'resuming'
                        self.gaze_behavior_completed = True
                        self.resume_start_time = current_time
                        override_motion = False

        # 3. Speed Calculation
        target_v = self.target_speed
        
        # Resume Smoothing (Quadratic)
        if self.gaze_state == 'resuming':
            elapsed = current_time - self.resume_start_time
            if elapsed < self.resume_duration:
                prog = elapsed / self.resume_duration
                target_v = self.target_speed * (prog * prog)
                msg = "Resuming (Accel)"
            else:
                self.gaze_state = 'completed'

        # End-of-Path Braking
        dist_left = self.path_length - distance_traveled
        if dist_left < self.braking_distance:
            target_v *= (dist_left / self.braking_distance)

        # High-level smoothing
        self.last_speed = self.speed_smoothing * self.last_speed + (1 - self.speed_smoothing) * target_v
        
        # 4. Path Vector Generation
        if not override_motion:
            if self.path_type == 'forward_zigzag' and self.zigzag_start <= distance_traveled < self.zigzag_end:
                # ZigZag Math
                prog = (distance_traveled - self.zigzag_start) / (self.zigzag_end - self.zigzag_start)
                vy = self.last_speed * self.zigzag_lateral_fraction * math.sin(2 * math.pi * self.zigzag_cycles * prog)
                vx = math.sqrt(max(0, self.last_speed**2 - vy**2))
            else:
                # Linear
                vx = self.last_speed
                vy = 0.0
        else:
            vx = 0.0
            vy = 0.0
            # omega is already set by gaze logic

        return vx, vy, omega, False, msg


class KeyboardInput:
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()
        self.events = []
    def start(self): threading.Thread(target=self._listen, daemon=True).start()
    def stop(self): self.running = False
    def get_key(self):
        with self.lock: return self.events.pop(0) if self.events else None
    def _listen(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    k = sys.stdin.read(1)
                    with self.lock:
                        if k == ' ': self.events.append('SPACE')
                        elif k.lower() == 'r': self.events.append('R')
                        elif k == '\x1b' or k == '\x03': self.events.append('ESC')
        finally: termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('interface', type=str)
    parser.add_argument('--speed', type=float, default=0.75)
    parser.add_argument('--path', type=str, default='linear_forward')
    parser.add_argument('--gaze', type=str, default='no_stop')
    args = parser.parse_args()

    robot = Go2EthernetControl(args.interface)
    controller = ExperimentalController(args.speed, args.path, args.gaze)
    kb = KeyboardInput()
    
    console = Console()
    layout = Layout()
    layout.split(Layout(name="main"), Layout(name="footer", size=3))

    def update_ui(status, dist, speed, yaw, is_walking):
        table = Table(box=box.ROUNDED)
        table.add_column("Metric", style="cyan")
        table.add_column("Value", style="green")
        table.add_row("Status", f"[bold {'green' if is_walking else 'red'}]{status}[/]")
        table.add_row("Distance", f"{dist:.2f} m")
        table.add_row("Speed", f"{speed:.2f} m/s")
        table.add_row("Yaw (Est)", f"{math.degrees(yaw):.1f}°")
        layout["main"].update(Panel(table, title=f"Go2W: {args.gaze}"))
        layout["footer"].update(Panel("SPACE: Pause/Resume | R: Reset | ESC: Exit"))
        return layout

    try:
        robot.stand_up()
        kb.start()
        is_walking = False
        initial_pos = None
        status = "READY"
        
        with Live(layout, refresh_per_second=10, screen=True) as live:
            while True:
                current_time = time.time()
                key = kb.get_key()
                
                if key == 'ESC': break
                elif key == 'R':
                    robot.stop()
                    is_walking = False
                    initial_pos = None
                    robot.position = [0.0, 0.0, 0.35]
                    robot.yaw = 0.0
                    controller.reset_state()
                    status = "RESET"
                elif key == 'SPACE':
                    is_walking = not is_walking
                    if is_walking:
                        if initial_pos is None:
                            initial_pos = robot.position.copy()
                            controller.reset_state()
                            status = "STARTED"
                        else:
                            status = "RESUMED"
                    else:
                        robot.stop()
                        status = "PAUSED"

                dist = 0.0
                vx, vy, omega = 0.0, 0.0, 0.0
                
                if is_walking and initial_pos:
                    dist = robot.position[0] - initial_pos[0]
                    vx, vy, omega, complete, msg = controller.get_velocity_commands(
                        dist, current_time, robot.yaw
                    )
                    status = msg
                    if complete:
                        robot.stop()
                        is_walking = False
                        initial_pos = None
                        status = "COMPLETE"
                    else:
                        robot.set_velocity(vx, vy, omega)
                else:
                    robot.stop()

                speed_val = math.sqrt(robot.current_vx**2 + robot.current_vy**2)
                live.update(update_ui(status, dist, speed_val, robot.yaw, is_walking))
                time.sleep(0.02)

    except Exception as e:
        console.print_exception()
    finally:
        kb.stop()
        if robot:
            print("\n[SHUTDOWN] Stopping robot...")
            robot.stop()
            time.sleep(0.5)
            robot.sit_down()
            time.sleep(1.0)
            print("[SHUTDOWN] Robot safe.")
        print("Control ended.")

if __name__ == "__main__":
    main()