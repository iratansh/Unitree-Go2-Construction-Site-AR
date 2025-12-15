"""Unitree Go2W Experimental Control 
This script enables Ethernet control of the Unitree Go2W robot to perform
a predefined gait pattern with specific gaze behaviors. It includes advanced
control logic to handle momentum during zigzag maneuvers and precise rotation
commands to minimize overshoot and undershoot.
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

_STATE_AVAILABLE = False
if _SDK_AVAILABLE:
    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        _SPORT_STATE_AVAILABLE = False
        _LOW_STATE_AVAILABLE = False
        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
            _SPORT_STATE_AVAILABLE = True
        except ImportError:
            _SPORT_STATE_AVAILABLE = False
        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
            _LOW_STATE_AVAILABLE = True
        except ImportError:
            _LOW_STATE_AVAILABLE = False
        _STATE_AVAILABLE = _SPORT_STATE_AVAILABLE or _LOW_STATE_AVAILABLE
        if not _STATE_AVAILABLE:
            print("Warning: Unitree state message types not available. Yaw will be estimated.")
    except ImportError:
        print("Warning: Unitree channel subscriber not available. Yaw will be estimated.")
        _STATE_AVAILABLE = False

# --- Configuration ---
SPEED_OPTIONS = [0.75, 1.25, 1.75, 2.25]
PATH_OPTIONS = ['linear_forward', 'forward_zigzag']
GAZE_OPTIONS = ['no_stop', 'stop_gaze_forward', 'stop_rotate_gaze']

class Go2EthernetControl:
    def __init__(self, network_interface="eth0"):
        if _SDK_AVAILABLE:
            ChannelFactoryInitialize(0, network_interface)
            self.client = SportClient()
            self.client.SetTimeout(10.0)
            self.client.Init()
                
        self.position = [0.0, 0.0, 0.35]
        self.yaw = 0.0
        self.measured_velocity = [0.0, 0.0, 0.0]
        self.measured_yaw_rate = 0.0
        self.yaw_received = False
        self.pose_received = False
        self._state_lock = threading.Lock()
        self._sport_state_subscriber = None
        self._low_state_subscriber = None

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0

        if _SDK_AVAILABLE and _STATE_AVAILABLE:
            self._init_state_subscriptions()

    def _init_state_subscriptions(self):
        if _SPORT_STATE_AVAILABLE:
            try:
                self._sport_state_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
                self._sport_state_subscriber.Init(self._on_sport_state, queueLen=1)
            except Exception as exc:
                print(f"Warning: Failed to subscribe to rt/sportmodestate: {exc}")
                self._sport_state_subscriber = None

        if _LOW_STATE_AVAILABLE:
            try:
                self._low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
                self._low_state_subscriber.Init(self._on_low_state, queueLen=1)
            except Exception as exc:
                print(f"Warning: Failed to subscribe to rt/lowstate: {exc}")
                self._low_state_subscriber = None

    def _on_sport_state(self, msg):
        try:
            yaw = float(msg.imu_state.rpy[2])
        except Exception:
            yaw = None

        try:
            yaw_rate = float(msg.yaw_speed)
        except Exception:
            try:
                yaw_rate = float(msg.imu_state.gyroscope[2])
            except Exception:
                yaw_rate = None

        try:
            position = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
        except Exception:
            position = None

        try:
            velocity = [float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]
        except Exception:
            velocity = None

        with self._state_lock:
            if yaw is not None:
                self.yaw = yaw
                self.yaw_received = True
            if yaw_rate is not None:
                self.measured_yaw_rate = yaw_rate
            if position is not None:
                self.position = position
                self.pose_received = True
            if velocity is not None:
                self.measured_velocity = velocity

    def _on_low_state(self, msg):
        try:
            yaw = float(msg.imu_state.rpy[2])
        except Exception:
            yaw = None

        try:
            yaw_rate = float(msg.imu_state.gyroscope[2])
        except Exception:
            yaw_rate = None

        with self._state_lock:
            if yaw is not None:
                self.yaw = yaw
                self.yaw_received = True
            if yaw_rate is not None:
                self.measured_yaw_rate = yaw_rate

    def stand_up(self):
        if not _SDK_AVAILABLE: return
        self.client.StandUp()
        time.sleep(1)
        self.client.BalanceStand()

    def sit_down(self):
        if not _SDK_AVAILABLE: return
        self.client.StandDown()
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

        # Simulation / fallback integration (only integrate what we aren't receiving)
        dt = 0.02
        if not (_SDK_AVAILABLE and _STATE_AVAILABLE and self.pose_received):
            self.position[0] += self.current_vx * dt
            self.position[1] += self.current_vy * dt
        if not (_SDK_AVAILABLE and _STATE_AVAILABLE and self.yaw_received):
            self.yaw += self.current_omega * dt

        if _SDK_AVAILABLE:
            vx = np.clip(vx, -3.0, 3.0)
            vy = np.clip(vy, -0.5, 0.5)
            omega = np.clip(omega, -1.5, 1.5)
            self.client.Move(vx, vy, omega)

    def get_measured_yaw_rate(self):
        with self._state_lock:
            if _SDK_AVAILABLE and _STATE_AVAILABLE and self.yaw_received:
                return float(self.measured_yaw_rate)
        return float(self.current_omega)

    def get_measured_speed(self):
        with self._state_lock:
            if _SDK_AVAILABLE and _STATE_AVAILABLE and self.pose_received:
                return float(math.sqrt(self.measured_velocity[0] ** 2 + self.measured_velocity[1] ** 2))
        return float(math.sqrt(self.current_vx ** 2 + self.current_vy ** 2))


class ExperimentalController:
    def __init__(self, speed, path_type, gaze_type):
        self.base_speed = speed # Store original speed request
        self.target_speed = speed
        self.path_type = path_type
        self.gaze_type = gaze_type
        
        self.path_length = 16.0
        self.braking_distance = 2.5
        
        self.gaze_stop_position = 8.0
        self.gaze_stop_duration = 2.0
        self.gaze_rotate_pause = 4.0
        
        # Pre-brake zone for zigzag mode (slow down before stop to reduce momentum)
        self.zigzag_prebrake_distance = 1.5  # Start slowing 1.5m before stop
        self.zigzag_prebrake_speed = 0.3     # Slow to this speed before stopping
        
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
        
        # Yaw tracking
        self.original_yaw = None  
        self.target_yaw = None    
        self.yaw_tolerance = 0.03 
        self.yaw_rate_tolerance = 0.10
        self.rotation_settling_time = 1.0 
        self.settling_start_time = None  
        self.achieved_gaze_yaw = None  # Actual yaw after first rotation (for symmetric return)
        self.gaze_rotate_angle = math.pi / 2.0
        
        self.heading_correction_gain = 1.5  

    def reset_state(self):
        self.last_speed = 0.0
        self.target_speed = self.base_speed
        self.gaze_state = 'moving'
        self.state_start_time = None
        self.gaze_behavior_completed = False
        self.resume_start_time = None
        self.original_yaw = None
        self.target_yaw = None
        self.settling_start_time = None
        self.achieved_gaze_yaw = None

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]""" 
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _get_rotation_command(self, current_yaw, target_yaw):
        """
        Calculates rotation with SYMMETRIC TUNING for both directions.
        Slower rotation speed to minimize wheel slip on Go2W.
        """
        error = self._normalize_angle(target_yaw - current_yaw)
        abs_error = abs(error)
        
        # Tolerance check
        if abs_error < self.yaw_tolerance:
            return 0.0, True
        
        # --- TUNING FOR REDUCED WHEEL SLIP ---
        kp = 0.8             # Softer response to reduce slip
        max_omega = 0.35     # Slower rotation to minimize wheel slip
        min_holding_power = 0.15  # Lower minimum to prevent jerky starts
        
        # --- PROGRESSIVE GAIN ---
        if abs_error < 0.2:  # ~11 deg - very close, crawl
            limit = 0.4      
        elif abs_error < 0.5:  # ~28 deg
            limit = 0.6      
        else:
            limit = 1.0

        omega = kp * error
        
        # Dynamic Clamp
        active_max = max_omega * limit
        omega = np.clip(omega, -active_max, active_max)

        # --- STICTION KICKER ---
        if abs(omega) < min_holding_power:
            omega = min_holding_power * (1.0 if error > 0 else -1.0)
        
        return omega, False

    def get_velocity_commands(self, distance_traveled, current_time, robot_yaw, robot_yaw_rate=0.0):
        msg = "Moving"
        
        # Capture initial heading for basic path correction
        if self.original_yaw is None and distance_traveled < 0.1:
            self.original_yaw = robot_yaw
        
        # 1. Path Completion
        if distance_traveled >= self.path_length:
            return 0.0, 0.0, 0.0, True, "Trial Complete"
        
        vx, vy, omega = 0.0, 0.0, 0.0
        override_motion = False
        
        # 2. Gaze Logic
        if self.gaze_type == 'stop_rotate_gaze':
            if not self.gaze_behavior_completed:
                
                # TRIGGER STOP
                if self.gaze_state == 'moving' and distance_traveled >= self.gaze_stop_position:
                    self.gaze_state = 'stopping_momentum'
                    self.state_start_time = current_time
                    override_motion = False # Let smoother handle stop
                    
                # STATE MACHINE
                elif self.gaze_state != 'moving' and self.gaze_state != 'resuming':
                    override_motion = True
                    
                    if self.gaze_state == 'stopping_momentum':
                        msg = "Braking ZigZag"
                        self.target_speed = 0.0 # Force brake
                        # Wait 0.8s for momentum to die down
                        if (current_time - self.state_start_time) > 0.8:
                            self.gaze_state = 'stopped'
                            self.state_start_time = current_time

                    elif self.gaze_state == 'stopped':
                        msg = "Stabilizing"
                        # Wait for chassis to settle completely
                        if (current_time - self.state_start_time) > 1.0:
                            self.gaze_state = 'rotating_out'
                            self.state_start_time = current_time
                            
                            # Capture rotation reference
                            self.original_yaw = robot_yaw 
                            self.target_yaw = self._normalize_angle(self.original_yaw + self.gaze_rotate_angle)
                            self.achieved_gaze_yaw = None

                    elif self.gaze_state == 'rotating_out':
                        omega, at_target = self._get_rotation_command(robot_yaw, self.target_yaw)
                        msg = (
                            f"Rotating Out (err: {math.degrees(self._normalize_angle(self.target_yaw - robot_yaw)):.1f}°)"
                            f" | yaw_rate: {math.degrees(robot_yaw_rate):.1f}°/s"
                        )

                        rotation_stable = at_target and abs(robot_yaw_rate) < self.yaw_rate_tolerance
                        if rotation_stable:
                            if self.settling_start_time is None:
                                self.settling_start_time = current_time
                            elif (current_time - self.settling_start_time) > self.rotation_settling_time:
                                self.achieved_gaze_yaw = robot_yaw
                                # Hold whatever yaw we actually achieved to prevent drift during pause
                                self.target_yaw = self.achieved_gaze_yaw
                                self.gaze_state = 'paused'
                                self.state_start_time = current_time
                                self.settling_start_time = None
                                omega = 0.0
                        else:
                            self.settling_start_time = None

                    elif self.gaze_state == 'paused':
                        omega, _ = self._get_rotation_command(robot_yaw, self.target_yaw)
                        msg = "Gazing (Hold Yaw)"
                        if (current_time - self.state_start_time) > self.gaze_rotate_pause:
                            self.gaze_state = 'rotating_in'
                            self.state_start_time = current_time
                            # Return to original heading (symmetric: negative of rotate-out)
                            self.target_yaw = self.original_yaw
                            self.settling_start_time = None

                    elif self.gaze_state == 'rotating_in':
                        omega, at_target = self._get_rotation_command(robot_yaw, self.target_yaw)
                        msg = (
                            f"Rotating In (err: {math.degrees(self._normalize_angle(self.target_yaw - robot_yaw)):.1f}°)"
                            f" | yaw_rate: {math.degrees(robot_yaw_rate):.1f}°/s"
                        )

                        rotation_stable = at_target and abs(robot_yaw_rate) < self.yaw_rate_tolerance
                        if rotation_stable:
                            if self.settling_start_time is None:
                                self.settling_start_time = current_time
                            if (current_time - self.settling_start_time) > 0.5:
                                self.gaze_state = 'resuming'
                                self.gaze_behavior_completed = True
                                self.resume_start_time = current_time
                                self.settling_start_time = None
                                override_motion = False
                                omega = 0.0
                        else:
                            self.settling_start_time = None

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
        
        # Resume Logic - Restore Speed
        if self.gaze_state == 'resuming':
            elapsed = current_time - self.resume_start_time
            if elapsed < self.resume_duration:
                prog = elapsed / self.resume_duration
                # Quadratic ease-in to Base Speed
                target_v = self.base_speed * (prog * prog)
                msg = "Resuming (Accel)"
            else:
                self.gaze_state = 'completed'
                self.target_speed = self.base_speed # Fully restored

        # End-of-Path Braking
        dist_left = self.path_length - distance_traveled
        if dist_left < self.braking_distance:
            # Scale whatever the current target is (braking takes priority)
            target_v = min(target_v, self.base_speed * (dist_left / self.braking_distance))

        # --- ZIGZAG PRE-BRAKE: Slow down before gaze stop to reduce momentum ---
        # This makes zigzag behave like forward mode for consistent rotation
        if (self.path_type == 'forward_zigzag' and 
            self.gaze_type == 'stop_rotate_gaze' and 
            not self.gaze_behavior_completed):
            dist_to_stop = self.gaze_stop_position - distance_traveled
            if 0 < dist_to_stop < self.zigzag_prebrake_distance:
                # Linearly ramp down to prebrake speed
                brake_progress = 1.0 - (dist_to_stop / self.zigzag_prebrake_distance)
                prebrake_target = self.base_speed - (self.base_speed - self.zigzag_prebrake_speed) * brake_progress
                target_v = min(target_v, prebrake_target)
                msg = f"Pre-braking ({dist_to_stop:.1f}m to stop)"

        # High-level smoothing
        self.last_speed = self.speed_smoothing * self.last_speed + (1 - self.speed_smoothing) * target_v
        
        # 4. Path Vector Generation
        if not override_motion:
            if self.path_type == 'forward_zigzag' and self.zigzag_start <= distance_traveled < self.zigzag_end:
                prog = (distance_traveled - self.zigzag_start) / (self.zigzag_end - self.zigzag_start)
                vy = self.last_speed * self.zigzag_lateral_fraction * math.sin(2 * math.pi * self.zigzag_cycles * prog)
                vx = math.sqrt(max(0, self.last_speed**2 - vy**2))
                
                # Heading Correction (Stronger now)
                if self.original_yaw is not None:
                    yaw_error = self._normalize_angle(self.original_yaw - robot_yaw)
                    omega = self.heading_correction_gain * yaw_error
                    omega = np.clip(omega, -0.6, 0.6) 
            else:
                # Linear motion
                vx = self.last_speed
                vy = 0.0
                if self.gaze_behavior_completed and self.original_yaw is not None:
                    yaw_error = self._normalize_angle(self.original_yaw - robot_yaw)
                    if abs(yaw_error) > self.yaw_tolerance:
                        omega = self.heading_correction_gain * yaw_error
                        omega = np.clip(omega, -0.6, 0.6)
        else:
            vx = 0.0
            vy = 0.0
            # omega set by gaze logic

        return vx, vy, omega, False, msg


class KeyboardInput:
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()
        self.events = []
        self.old_settings = None
        self.fd = sys.stdin.fileno()
        
    def start(self):
        self.old_settings = termios.tcgetattr(self.fd)
        threading.Thread(target=self._listen, daemon=True).start()
        
    def stop(self):
        self.running = False
        if self.old_settings:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except:
                pass
                
    def get_key(self):
        with self.lock: return self.events.pop(0) if self.events else None
        
    def _listen(self):
        try:
            tty.setcbreak(self.fd)
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    k = sys.stdin.read(1)
                    with self.lock:
                        if k == ' ': self.events.append('SPACE')
                        elif k.lower() == 'r': self.events.append('R')
                        elif k == '\x1b' or k == '\x03': self.events.append('ESC')
        except:
            pass

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
        table.add_row("Yaw", f"{math.degrees(yaw):.1f}°")
        layout["main"].update(Panel(table, title=f"Go2W: {args.gaze}"))
        layout["footer"].update(Panel("SPACE: Pause/Resume | R: Reset | ESC: Exit"))
        return layout

    try:
        robot.stand_up()
        kb.start()
        is_walking = False
        pending_start = False
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
                    if not (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.pose_received):
                        robot.position = [0.0, 0.0, 0.35]
                    if not (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.yaw_received):
                        robot.yaw = 0.0
                    controller.reset_state()
                    status = "RESET"
                elif key == 'SPACE':
                    if is_walking:
                        pending_start = False
                        is_walking = False
                        robot.stop()
                        status = "PAUSED"
                    else:
                        if pending_start:
                            pending_start = False
                            status = "READY"
                        elif _SDK_AVAILABLE and _STATE_AVAILABLE and not robot.yaw_received:
                            pending_start = True
                            robot.stop()
                            status = "WAITING FOR STATE"
                        else:
                            is_walking = True
                            if initial_pos is None:
                                initial_pos = robot.position.copy()
                                controller.reset_state()
                                status = "STARTED"
                            else:
                                status = "RESUMED"

                if pending_start and (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.yaw_received):
                    pending_start = False
                    is_walking = True
                    if initial_pos is None:
                        initial_pos = robot.position.copy()
                        controller.reset_state()
                        status = "STARTED"
                    else:
                        status = "RESUMED"

                dist = 0.0
                vx, vy, omega = 0.0, 0.0, 0.0
                
                if is_walking and initial_pos:
                    dist = robot.position[0] - initial_pos[0]
                    vx, vy, omega, complete, msg = controller.get_velocity_commands(
                        dist, current_time, robot.yaw, robot.get_measured_yaw_rate()
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

                speed_val = robot.get_measured_speed()
                live.update(update_ui(status, dist, speed_val, robot.yaw, is_walking))
                time.sleep(0.02)

    except Exception as e:
        console.print_exception()
    finally:
        kb.stop()
        try:
            import os
            os.system('stty sane 2>/dev/null')
        except:
            pass
        
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