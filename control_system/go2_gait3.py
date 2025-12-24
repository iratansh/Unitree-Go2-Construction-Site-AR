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

# Wheel radius (meters) used for wheel odometry.
# Derived from the Go2W URDF wheel mesh (control_system/URDF/go2w_description/dae/*wheel.dae),
# where the outer radius is ~0.086m (diameter ~17.2cm).
WHEEL_RADIUS_M = 0.086
WHEEL_ODOM_MIN_MEAN_ABS_DQ = 0.5  # rad/s; ignore noise when nearly stopped
WHEEL_ODOM_STABLE_SAMPLES = 10  # samples to lock wheel motor indices
WHEEL_SIGN_LEARN_SAMPLES = 15
WHEEL_SIGN_LEARN_CMD_VX_MIN = 0.2
WHEEL_SIGN_LEARN_CMD_VY_MAX = 0.1
WHEEL_SIGN_LEARN_CMD_OMEGA_MAX = 0.6


def _extract_xyz(value):
    if value is None:
        return None
    try:
        return [float(value[0]), float(value[1]), float(value[2])]
    except Exception:
        pass
    try:
        return [float(value.x), float(value.y), float(value.z)]
    except Exception:
        return None


class Go2EthernetControl:
    def __init__(
        self,
        network_interface="eth0",
        wheel_radius_m=WHEEL_RADIUS_M,
        wheel_odom_scale=1.0,
        wheel_motor_indices=None,
    ):
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
        self.velocity_received = False
        self.wheel_odom_received = False
        self.wheel_speed_mps = 0.0
        self._wheel_radius_m = float(wheel_radius_m)
        self._wheel_odom_scale = float(wheel_odom_scale)
        self._wheel_motor_indices_user = (
            [int(i) for i in wheel_motor_indices] if wheel_motor_indices is not None else None
        )
        self._wheel_motor_indices = None
        self._wheel_indices_candidate = None
        self._wheel_indices_candidate_hits = 0
        self._wheel_sign_indices = None
        self._wheel_sign_scores = None
        self._wheel_signs = None
        self._wheel_sign_learn_samples = 0
        self._motor_state_count = None
        self._wheel_detect_mode = None
        self._state_lock = threading.Lock()
        self._sport_state_subscriber = None
        self._low_state_subscriber = None

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        self._last_command_time = None

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

        position = (
            _extract_xyz(getattr(msg, "position", None))
            or _extract_xyz(getattr(msg, "pos", None))
            or _extract_xyz(getattr(msg, "world_position", None))
            or _extract_xyz(getattr(msg, "world_pos", None))
        )
        velocity = (
            _extract_xyz(getattr(msg, "velocity", None))
            or _extract_xyz(getattr(msg, "vel", None))
            or _extract_xyz(getattr(msg, "v", None))
        )

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
                self.velocity_received = True

    def _on_low_state(self, msg):
        try:
            yaw = float(msg.imu_state.rpy[2])
        except Exception:
            yaw = None

        try:
            yaw_rate = float(msg.imu_state.gyroscope[2])
        except Exception:
            yaw_rate = None

        wheel_speed_mps = None
        try:
            motor_states = getattr(msg, "motor_state", None)
            if motor_states is None:
                motor_states = getattr(msg, "motorState", None)
            if motor_states is not None:
                dq_values = []
                for state in motor_states:
                    dq = getattr(state, "dq", None)
                    if dq is None:
                        dq = getattr(state, "qd", None)
                    dq_values.append(float(dq) if dq is not None else 0.0)

                if len(dq_values) >= 4:
                    motor_count = int(len(dq_values))
                    self._motor_state_count = motor_count

                    # If motor ordering matches the Go2W URDF, the wheel joints are the 4th joint in each leg group:
                    # - 16 motors: [3, 7, 11, 15]
                    # - 17 motors with a dummy/blank at index 0: [4, 8, 12, 16]
                    # Users can override with --wheel-motor-indices.
                    if self._wheel_motor_indices_user is not None:
                        self._wheel_detect_mode = "user"
                    elif motor_count == 16:
                        self._wheel_motor_indices = [3, 7, 11, 15]
                        self._wheel_detect_mode = "fixed16"
                    elif motor_count == 17:
                        self._wheel_motor_indices = [4, 8, 12, 16]
                        self._wheel_detect_mode = "fixed17"

                    abs_dq = np.abs(np.asarray(dq_values, dtype=float))
                    top4 = np.argpartition(abs_dq, -4)[-4:]
                    top4_candidate = tuple(sorted(int(i) for i in top4))
                    mean_abs_top4 = float(abs_dq[list(top4_candidate)].mean())

                    # Prefer known Go2W wheel joint patterns when plausible:
                    # - 16 joints with 0-based indexing: [3, 7, 11, 15]
                    # - 17 joints with a dummy at index 0: [4, 8, 12, 16]
                    preferred_candidates = []
                    if len(dq_values) >= 16:
                        preferred_candidates.append((3, 7, 11, 15))
                    if len(dq_values) >= 17:
                        preferred_candidates.append((4, 8, 12, 16))

                    best_preferred = None
                    best_preferred_mean_abs = -1.0
                    for cand in preferred_candidates:
                        try:
                            cand_mean = float(abs_dq[list(cand)].mean())
                        except Exception:
                            continue
                        if cand_mean > best_preferred_mean_abs:
                            best_preferred_mean_abs = cand_mean
                            best_preferred = cand

                    if (
                        best_preferred is not None
                        and best_preferred_mean_abs >= float(WHEEL_ODOM_MIN_MEAN_ABS_DQ)
                        and best_preferred_mean_abs >= (0.6 * mean_abs_top4)
                    ):
                        candidate = tuple(sorted(int(i) for i in best_preferred))
                        mean_abs = float(best_preferred_mean_abs)
                    else:
                        candidate = top4_candidate
                        mean_abs = float(mean_abs_top4)

                    cmd_vx = float(self.current_vx)
                    cmd_vy = float(self.current_vy)
                    cmd_omega = float(self.current_omega)
                    learn_motion_ok = (
                        abs(cmd_vx) >= float(WHEEL_SIGN_LEARN_CMD_VX_MIN)
                        and abs(cmd_vy) <= float(WHEEL_SIGN_LEARN_CMD_VY_MAX)
                        and abs(cmd_omega) <= float(WHEEL_SIGN_LEARN_CMD_OMEGA_MAX)
                    )
                    # Only lock auto-detected indices during mostly-straight forward motion
                    # to avoid accidentally locking onto leg joints during postural adjustments.
                    if (
                        self._wheel_motor_indices_user is None
                        and self._wheel_motor_indices is None
                        and learn_motion_ok
                        and mean_abs >= float(WHEEL_ODOM_MIN_MEAN_ABS_DQ)
                    ):
                        if self._wheel_indices_candidate == candidate:
                            self._wheel_indices_candidate_hits += 1
                        else:
                            self._wheel_indices_candidate = candidate
                            self._wheel_indices_candidate_hits = 1

                        if self._wheel_motor_indices is None and self._wheel_indices_candidate_hits >= int(WHEEL_ODOM_STABLE_SAMPLES):
                            self._wheel_motor_indices = list(candidate)
                            self._wheel_detect_mode = "auto"

                    wheel_locked = False
                    wheel_indices = None
                    if self._wheel_motor_indices_user is not None:
                        user_indices = [int(i) for i in self._wheel_motor_indices_user]
                        if all(0 <= int(i) < len(dq_values) for i in user_indices):
                            wheel_indices = user_indices
                            wheel_locked = True
                    if wheel_indices is None and self._wheel_motor_indices is not None:
                        wheel_indices = list(self._wheel_motor_indices)
                        wheel_locked = True
                    if wheel_indices is None:
                        wheel_indices = list(candidate)
                    wheel_dq = np.asarray([dq_values[i] for i in wheel_indices], dtype=float)
                    abs_wheel_dq = np.abs(wheel_dq)

                    # When wheel indices aren't locked yet, use the fastest wheels to reduce
                    # sensitivity to occasional mis-identification. Once locked, use all wheels
                    # for a less biased speed estimate.
                    radius_m = float(self._wheel_radius_m)
                    odom_scale = float(self._wheel_odom_scale)
                    cmd_vx = float(self.current_vx)
                    cmd_vy = float(self.current_vy)
                    cmd_omega = float(self.current_omega)

                    wheel_indices_tuple = tuple(int(i) for i in wheel_indices)
                    if self._wheel_sign_indices != wheel_indices_tuple:
                        self._wheel_sign_indices = wheel_indices_tuple
                        self._wheel_sign_scores = np.zeros(len(wheel_indices), dtype=float)
                        self._wheel_signs = None
                        self._wheel_sign_learn_samples = 0

                    if (
                        abs(cmd_vx) >= float(WHEEL_SIGN_LEARN_CMD_VX_MIN)
                        and abs(cmd_vy) <= float(WHEEL_SIGN_LEARN_CMD_VY_MAX)
                        and abs(cmd_omega) <= float(WHEEL_SIGN_LEARN_CMD_OMEGA_MAX)
                        and self._wheel_sign_scores is not None
                    ):
                        self._wheel_sign_learn_samples += 1
                        self._wheel_sign_scores += wheel_dq * cmd_vx
                        if self._wheel_sign_learn_samples >= int(WHEEL_SIGN_LEARN_SAMPLES):
                            self._wheel_signs = np.where(self._wheel_sign_scores >= 0.0, 1.0, -1.0)

                    if self._wheel_signs is not None and len(self._wheel_signs) == len(wheel_dq):
                        # Convert wheel angular rates into a forward-drive linear speed estimate by
                        # learning constant sign flips per wheel. This cancels most yaw/rotation-only
                        # wheel motion (important for zigzag / heading corrections).
                        aligned_dq = wheel_dq * self._wheel_signs
                        forward_dq = float(aligned_dq.mean())
                        wheel_speed_mps = float(odom_scale * radius_m * abs(forward_dq))
                    elif wheel_locked:
                        wheel_speed_mps = float(odom_scale * radius_m * float(abs_wheel_dq.mean()))
                    else:
                        wheel_speed_mps = float(odom_scale * radius_m * float(np.sort(abs_wheel_dq)[-2:].mean()))
        except Exception:
            wheel_speed_mps = None

        with self._state_lock:
            if yaw is not None:
                self.yaw = yaw
                self.yaw_received = True
            if yaw_rate is not None:
                self.measured_yaw_rate = yaw_rate
            if wheel_speed_mps is not None:
                self.wheel_speed_mps = float(wheel_speed_mps)
                self.wheel_odom_received = True

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

        cmd_vx = float(self.current_vx)
        cmd_vy = float(self.current_vy)
        cmd_omega = float(self.current_omega)

        # Simulation / fallback integration (only integrate what we aren't receiving)
        now = time.time()
        if self._last_command_time is None:
            dt = 0.02
        else:
            dt = float(now - self._last_command_time)
            dt = float(np.clip(dt, 0.0, 0.1))
        self._last_command_time = now
        if not (_SDK_AVAILABLE and _STATE_AVAILABLE and self.pose_received):
            # Treat Move(vx, vy) as body-frame velocities and rotate into a world-frame estimate.
            yaw = float(self.yaw)
            vx_world = cmd_vx * math.cos(yaw) - cmd_vy * math.sin(yaw)
            vy_world = cmd_vx * math.sin(yaw) + cmd_vy * math.cos(yaw)
            self.position[0] += vx_world * dt
            self.position[1] += vy_world * dt
        if not (_SDK_AVAILABLE and _STATE_AVAILABLE and self.yaw_received):
            self.yaw += cmd_omega * dt

        if _SDK_AVAILABLE:
            cmd_vx = float(np.clip(cmd_vx, -3.0, 3.0))
            cmd_vy = float(np.clip(cmd_vy, -0.5, 0.5))
            cmd_omega = float(np.clip(cmd_omega, -1.5, 1.5))
            self.client.Move(cmd_vx, cmd_vy, cmd_omega)

    def get_measured_yaw_rate(self):
        with self._state_lock:
            if _SDK_AVAILABLE and _STATE_AVAILABLE and self.yaw_received:
                return float(self.measured_yaw_rate)
        return float(self.current_omega)

    def get_measured_speed(self):
        with self._state_lock:
            if _SDK_AVAILABLE and _STATE_AVAILABLE and self.velocity_received:
                return float(math.sqrt(self.measured_velocity[0] ** 2 + self.measured_velocity[1] ** 2))
            if _SDK_AVAILABLE and _STATE_AVAILABLE and self.wheel_odom_received:
                return float(abs(self.wheel_speed_mps))
        return float(math.sqrt(self.current_vx ** 2 + self.current_vy ** 2))


class ExperimentalController:
    def __init__(self, speed, path_type, gaze_type, participant_side="right", path_length=14.0):
        self.base_speed = speed # Store original speed request
        self.target_speed = speed
        self.path_type = path_type
        self.gaze_type = gaze_type
        
        self.path_length = float(path_length)
        self.braking_distance = 2.5

        self.halfway_distance = self.path_length / 2.0
        
        self.gaze_stop_position = self.halfway_distance
        self.gaze_stop_duration = 4.0
        self.gaze_rotate_pause = 4.0
        # Braking profile used to reach the stop marker consistently across speeds.
        # The commanded speed is limited to v <= sqrt(2 * a * d) as we approach the stop,
        # which corresponds to a roughly constant deceleration "a" (in m/s^2).
        self.gaze_stop_deceleration = 1.2
        
        # Pre-brake zone for zigzag mode (slow down before stop to reduce momentum)
        self.zigzag_prebrake_distance = 1.5  # Start slowing 1.5m before stop
        self.zigzag_prebrake_speed = 0.3     # Slow to this speed before stopping
        
        # Start the zigzag earlier so the lateral motion is visible sooner.
        self.zigzag_start = 3.5
        # Total zigzag window length (meters along the forward path).
        self.zigzag_distance = 7.0
        self.zigzag_end = self.zigzag_start + self.zigzag_distance
        # Maximum lateral deviation from the original path while zigzagging.
        self.zigzag_max_lateral = 2.0
        # Fixed, asymmetric pattern expressed in "steps".
        # Added one extra zigzag "motion" (one additional lateral segment) vs go2_gait2.
        # Make the final oscillation a bit larger (and less "tiny") than the initial go2_gait3 draft.
        self.zigzag_step_pattern = [-3, 8, -6, 3, -2]
        # Lateral control tuning (vy is also clipped in set_velocity()).
        # Increase max lateral speed to make the zigzag motion more pronounced.
        self.zigzag_max_vy = 0.5
        # Gain in the distance-domain: dy/ds += k * (y_des - y).
        # Using a per-meter gain makes tracking consistent across speed modes.
        self.zigzag_lateral_kp = 1.0
        # Gain in the distance-domain: dy/ds = -k * y (re-center after zigzag).
        self.zigzag_recenter_kp = 0.6
        self.zigzag_recenter_deadband = 0.05
        # Participant position relative to the original path at the halfway point.
        # Used to ensure the zigzag direction at halfway is away from the participant.
        self.participant_side = participant_side  # {"left","right","none"}
        # Build a single, speed-independent zigzag profile so the path geometry stays
        # consistent across all speed modes.
        #
        # To make the lateral motion more apparent, size the profile using the slowest
        # configured speed (instead of the fastest). At higher speeds, vy may saturate,
        # but the robot will still produce the steepest lateral motion it can.
        self.zigzag_profile_speed = float(min(SPEED_OPTIONS))
        # Reserve some lateral-velocity headroom so feedback can still correct drift.
        self.zigzag_profile_vy_utilization = 0.9
        self._zigzag_profile = self._build_zigzag_profile()
        
        self.gaze_behavior_completed = False
        self.gaze_state = 'moving'
        self.state_start_time = 0.0
        self.resume_start_time = None
        self.resume_duration = 2.0
        
        self.last_speed = 0.0
        self.speed_smoothing = 0.2
        
        # Yaw tracking
        self.path_yaw = None  # Path heading reference (captured at start; never overwritten)
        self.gaze_reference_yaw = None  # Heading reference captured at gaze-stop (for rotate out/in)
        self.target_yaw = None    
        self.yaw_tolerance = 0.03 
        self.yaw_rate_tolerance = 0.10
        self.rotation_settling_time = 1.0 
        self.settling_start_time = None  
        self.achieved_gaze_yaw = None  # Actual yaw after first rotation (for symmetric return)
        self.gaze_rotate_angle = math.pi / 2.0
        
        self.heading_correction_gain = 1.5  

    def _build_zigzag_profile(self):
        """Build a piecewise-linear lateral-offset profile over the zigzag window."""
        pattern = [int(s) for s in self.zigzag_step_pattern if int(s) != 0]
        if not pattern:
            return [(self.zigzag_start, 0.0), (self.zigzag_end, 0.0)]

        net_steps = sum(pattern)
        if net_steps != 0:
            pattern.append(-net_steps)

        cumulative_steps = [0]
        running = 0
        for delta in pattern:
            running += int(delta)
            cumulative_steps.append(running)

        max_abs_steps = max(abs(v) for v in cumulative_steps) or 1
        total_abs_steps = sum(abs(int(v)) for v in pattern) or 1

        meters_along_path_per_step = float(self.zigzag_distance) / float(total_abs_steps)
        meters_per_step_by_offset = (float(self.zigzag_max_lateral) * 0.95) / float(max_abs_steps)
        meters_per_step_by_vy = (
            meters_along_path_per_step
            * (float(self.zigzag_max_vy) * float(self.zigzag_profile_vy_utilization))
            / max(1e-6, float(self.zigzag_profile_speed))
        )
        meters_per_step = min(meters_per_step_by_offset, meters_per_step_by_vy)

        offsets = [float(v) * meters_per_step for v in cumulative_steps]
        offsets = [
            float(np.clip(v, -float(self.zigzag_max_lateral), float(self.zigzag_max_lateral))) for v in offsets
        ]

        distances = [float(self.zigzag_start)]
        s = float(self.zigzag_start)
        for delta in pattern:
            s += abs(float(delta)) * meters_along_path_per_step
            distances.append(float(s))
        distances[-1] = float(self.zigzag_end)

        profile = list(zip(distances, offsets))

        # Ensure that at the halfway point the zigzag is moving away from the participant.
        # Convention: positive lateral is "left" of the original path.
        if (
            self.participant_side in {"left", "right"}
            and float(self.zigzag_start) < float(self.halfway_distance) < float(self.zigzag_end)
        ):
            desired_slope_sign = 1.0 if self.participant_side == "right" else -1.0
            dy_ds_half = 0.0
            for idx in range(len(profile) - 1):
                s0, y0 = profile[idx]
                s1, y1 = profile[idx + 1]
                if s0 <= float(self.halfway_distance) < s1:
                    ds = float(s1 - s0)
                    dy_ds_half = float((y1 - y0) / ds) if ds > 1e-9 else 0.0
                    break

            if dy_ds_half != 0.0:
                sign_mult = desired_slope_sign * (1.0 if dy_ds_half > 0.0 else -1.0)
            else:
                sign_mult = desired_slope_sign

            if sign_mult != 1.0:
                profile = [(float(s), float(y) * sign_mult) for s, y in profile]

        return profile

    def _desired_zigzag_lateral(self, distance_traveled):
        """Return (y_des, dy/ds) for the current zigzag segment."""
        if distance_traveled <= self.zigzag_start or distance_traveled >= self.zigzag_end:
            return 0.0, 0.0

        profile = self._zigzag_profile or [(self.zigzag_start, 0.0), (self.zigzag_end, 0.0)]
        for idx in range(len(profile) - 1):
            s0, y0 = profile[idx]
            s1, y1 = profile[idx + 1]
            if s0 <= distance_traveled < s1:
                ds = float(s1 - s0)
                if ds <= 1e-9:
                    return float(y1), 0.0
                t = float((distance_traveled - s0) / ds)
                y_des = float(y0 + t * (y1 - y0))
                slope = float((y1 - y0) / ds)
                return y_des, slope
        return float(profile[-1][1]), 0.0

    def reset_state(self):
        self.last_speed = 0.0
        self.target_speed = self.base_speed
        self.gaze_state = 'moving'
        self.state_start_time = None
        self.gaze_behavior_completed = False
        self.resume_start_time = None
        self.path_yaw = None
        self.gaze_reference_yaw = None
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

    def _should_apply_heading_correction(self):
        if self.path_yaw is None:
            return False
        if self.gaze_type in {"stop_gaze_forward", "stop_rotate_gaze"}:
            return self.gaze_state in {"moving", "resuming", "completed"}
        return True

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

    def get_velocity_commands(self, distance_traveled, lateral_offset, current_time, robot_yaw, robot_yaw_rate=0.0):
        msg = "Moving"
        
        # Capture initial heading for basic path correction
        if self.path_yaw is None and distance_traveled < 0.1:
            self.path_yaw = robot_yaw
        
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
                            self.gaze_reference_yaw = robot_yaw
                            self.target_yaw = self._normalize_angle(self.gaze_reference_yaw + self.gaze_rotate_angle)
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
                            if self.path_yaw is not None:
                                self.target_yaw = self.path_yaw
                            else:
                                self.target_yaw = self.gaze_reference_yaw
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

        # --- GAZE STOP BRAKING: reach stop marker consistently across speeds ---
        if (
            self.gaze_type in {"stop_gaze_forward", "stop_rotate_gaze"}
            and not self.gaze_behavior_completed
            and self.gaze_state == "moving"
        ):
            dist_to_stop = self.gaze_stop_position - distance_traveled
            if dist_to_stop > 0.0:
                speed_cap = math.sqrt(max(0.0, 2.0 * self.gaze_stop_deceleration * dist_to_stop))
                if speed_cap < target_v:
                    target_v = speed_cap
                    msg = f"Braking to Stop ({dist_to_stop:.2f}m)"

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
                y_des, dy_ds = self._desired_zigzag_lateral(distance_traveled)
                desired_slope = float(dy_ds) + float(self.zigzag_lateral_kp) * (float(y_des) - float(lateral_offset))
                denom = math.hypot(1.0, desired_slope)
                vx = float(self.last_speed) / denom if denom > 1e-9 else float(self.last_speed)
                vy = desired_slope * vx

                max_vy = min(float(self.zigzag_max_vy), float(self.last_speed))
                vy = float(np.clip(vy, -max_vy, max_vy))

                if abs(float(lateral_offset)) > float(self.zigzag_max_lateral):
                    vy = -math.copysign(max_vy, float(lateral_offset))

                vx = math.sqrt(max(0.0, self.last_speed**2 - vy**2))
                msg = f"Zigzag (lat: {lateral_offset:+.2f}m → {y_des:+.2f}m)"
            elif self.path_type == 'forward_zigzag' and distance_traveled >= self.zigzag_end:
                # After the zigzag window, gently re-center on the original trajectory if needed.
                vx = self.last_speed
                vy = 0.0
                if abs(float(lateral_offset)) > self.zigzag_recenter_deadband:
                    desired_slope = -float(self.zigzag_recenter_kp) * float(lateral_offset)
                    denom = math.hypot(1.0, desired_slope)
                    vx = float(self.last_speed) / denom if denom > 1e-9 else float(self.last_speed)
                    vy = desired_slope * vx
                    max_vy = min(float(self.zigzag_max_vy), float(self.last_speed))
                    vy = float(np.clip(vy, -max_vy, max_vy))
                    vx = math.sqrt(max(0.0, self.last_speed**2 - vy**2))
                    msg = f"Re-centering (lat: {lateral_offset:+.2f}m)"
            else:
                # Linear motion
                vx = self.last_speed
                vy = 0.0

            if self._should_apply_heading_correction():
                yaw_error = self._normalize_angle(float(self.path_yaw) - float(robot_yaw))
                if abs(yaw_error) > float(self.yaw_tolerance):
                    omega = float(np.clip(self.heading_correction_gain * yaw_error, -0.6, 0.6))
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
                        elif k.lower() == 'c': self.events.append('C')
                        elif k == '\x1b' or k == '\x03': self.events.append('ESC')
        except:
            pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('interface', type=str)
    parser.add_argument('--speed', type=float, default=0.75)
    parser.add_argument('--path', type=str, default='linear_forward')
    parser.add_argument('--gaze', type=str, default='no_stop')
    parser.add_argument(
        '--path-length',
        type=float,
        default=14.0,
        help="Path length in meters (forward-progress distance used for completion).",
    )
    parser.add_argument(
        '--distance-scale',
        type=float,
        default=1.0,
        help=(
            "Scale applied to the distance/lateral estimate before control logic. "
            "Use to calibrate when odometry units are off (e.g., try 14/5.5≈2.55 if 14m reads as 5.5m)."
        ),
    )
    parser.add_argument(
        '--distance-source',
        type=str,
        default='auto',
        choices=['auto', 'state_pos', 'state_vel', 'wheel_odom', 'cmd_int'],
        help=(
            "Which signal to use for distance estimation. "
            "'auto' prefers STATE_POS → STATE_VEL → WHEEL_ODOM → CMD_INT."
        ),
    )
    parser.add_argument(
        '--calib-distance',
        type=float,
        default=None,
        help="Known distance (m) used when pressing 'C' to calibrate distance-scale.",
    )
    parser.add_argument(
        '--wheel-radius',
        type=float,
        default=WHEEL_RADIUS_M,
        help="Wheel radius in meters used for wheel odometry (only matters when Dist Src is WHEEL_ODOM).",
    )
    parser.add_argument(
        '--wheel-odom-scale',
        type=float,
        default=1.0,
        help="Scale factor applied to wheel odometry (only matters when Dist Src is WHEEL_ODOM).",
    )
    parser.add_argument(
        '--wheel-motor-indices',
        type=str,
        default=None,
        help=(
            "Comma-separated 0-based indices for the 4 wheel motors in LowState.motor_state "
            "(e.g., '3,7,11,15'). Overrides auto-detection."
        ),
    )
    parser.add_argument(
        '--participant-side',
        type=str,
        default='right',
        choices=['left', 'right', 'none'],
        help=(
            "Which side of the path the participant stands on at the halfway point, defined in the robot travel frame "
            "(looking from start→end / increasing distance). Used to bias zigzag direction away at 7m. "
            "If you're standing at halfway facing the approaching robot, your left/right are mirrored."
        ),
    )
    args = parser.parse_args()

    wheel_motor_indices = None
    if args.wheel_motor_indices:
        try:
            parts = [p.strip() for p in args.wheel_motor_indices.replace(";", ",").split(",") if p.strip()]
            wheel_motor_indices = [int(p) for p in parts]
            if len(wheel_motor_indices) != 4:
                raise ValueError("Need exactly 4 indices.")
        except Exception as exc:
            raise SystemExit(f"Invalid --wheel-motor-indices: {exc}")

    robot = Go2EthernetControl(
        args.interface,
        wheel_radius_m=args.wheel_radius,
        wheel_odom_scale=args.wheel_odom_scale,
        wheel_motor_indices=wheel_motor_indices,
    )
    controller = ExperimentalController(
        args.speed,
        args.path,
        args.gaze,
        args.participant_side,
        path_length=args.path_length,
    )
    kb = KeyboardInput()
    
    console = Console()
    layout = Layout()
    layout.split(Layout(name="main"), Layout(name="footer", size=3))

    def update_ui(status, dist, lateral, speed, yaw, is_walking, dist_source, dist_scale):
        table = Table(box=box.ROUNDED)
        table.add_column("Metric", style="cyan")
        table.add_column("Value", style="green")
        table.add_row("Status", f"[bold {'green' if is_walking else 'red'}]{status}[/]")
        table.add_row("Distance", f"{dist:.2f} m")
        table.add_row("Lateral", f"{lateral:+.2f} m")
        table.add_row("Speed", f"{speed:.2f} m/s")
        table.add_row("Yaw", f"{math.degrees(yaw):.1f}°")
        table.add_row("Dist Src", str(dist_source))
        table.add_row("Dist Scale", f"{dist_scale:.3f}")
        table.add_row(
            "Odom",
            f"state:{int(_STATE_AVAILABLE)} pose:{int(robot.pose_received)} vel:{int(robot.velocity_received)} wheel:{int(robot.wheel_odom_received)}",
        )
        if robot.wheel_odom_received:
            table.add_row("Wheel v", f"{robot.wheel_speed_mps:.2f} m/s")
        if robot._motor_state_count is not None:
            table.add_row("Motor N", str(int(robot._motor_state_count)))
        wheel_idx = robot._wheel_motor_indices_user if robot._wheel_motor_indices_user is not None else robot._wheel_motor_indices
        if wheel_idx is not None:
            table.add_row("Wheel idx", ",".join(str(int(i)) for i in wheel_idx))
            if len(wheel_idx) == 4 and (max(wheel_idx) - min(wheel_idx)) <= 3:
                table.add_row("Wheel WARN", "idx clustered; try --wheel-motor-indices 3,7,11,15")
        if robot._wheel_detect_mode is not None:
            table.add_row("Wheel mode", str(robot._wheel_detect_mode))
        if robot._wheel_signs is not None:
            table.add_row("Wheel sign", ",".join("+" if float(s) >= 0.0 else "-" for s in robot._wheel_signs))
        table.add_row(
            "Cmd (vx,vy,w)",
            f"({robot.current_vx:+.2f},{robot.current_vy:+.2f},{robot.current_omega:+.2f})",
        )
        layout["main"].update(Panel(table, title=f"Go2W: {args.gaze}"))
        layout["footer"].update(Panel("SPACE: Pause/Resume | R: Reset | ESC: Exit"))
        return layout

    try:
        robot.stand_up()
        kb.start()
        is_walking = False
        pending_start = False
        initial_pos = None
        initial_yaw = None
        progress_time = None
        distance_along_path = 0.0
        lateral_estimate = 0.0
        distance_source = "N/A"
        status = "READY"
        distance_scale = float(args.distance_scale)
        dist_display = 0.0
        lateral_display = 0.0
        last_forward_sign = 1.0
        
        with Live(layout, refresh_per_second=10, screen=True) as live:
            while True:
                current_time = time.time()
                key = kb.get_key()
                
                if key == 'ESC': break
                elif key == 'R':
                    robot.stop()
                    is_walking = False
                    initial_pos = None
                    initial_yaw = None
                    progress_time = None
                    distance_along_path = 0.0
                    lateral_estimate = 0.0
                    distance_source = "N/A"
                    dist_display = 0.0
                    lateral_display = 0.0
                    last_forward_sign = 1.0
                    distance_scale = float(args.distance_scale)
                    if not (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.pose_received):
                        robot.position = [0.0, 0.0, 0.35]
                    if not (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.yaw_received):
                        robot.yaw = 0.0
                    controller.reset_state()
                    status = "RESET"
                elif key == 'C':
                    if args.calib_distance is None:
                        status = "CALIB: set --calib-distance"
                    elif float(dist_display) <= 0.05:
                        status = "CALIB: need Distance > 0"
                    else:
                        ratio = float(args.calib_distance) / float(dist_display)
                        distance_scale *= ratio
                        dist_display *= ratio
                        lateral_display *= ratio
                        status = f"CALIB: distance_scale={distance_scale:.3f}"
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
                                initial_yaw = float(robot.yaw)
                                progress_time = current_time
                                distance_along_path = 0.0
                                lateral_estimate = 0.0
                                distance_source = "N/A"
                                dist_display = 0.0
                                lateral_display = 0.0
                                last_forward_sign = 1.0
                                controller.reset_state()
                                status = "STARTED"
                            else:
                                status = "RESUMED"

                if pending_start and (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.yaw_received):
                    pending_start = False
                    is_walking = True
                    if initial_pos is None:
                        initial_pos = robot.position.copy()
                        initial_yaw = float(robot.yaw)
                        progress_time = current_time
                        distance_along_path = 0.0
                        lateral_estimate = 0.0
                        distance_source = "N/A"
                        dist_display = 0.0
                        lateral_display = 0.0
                        last_forward_sign = 1.0
                        controller.reset_state()
                        status = "STARTED"
                    else:
                        status = "RESUMED"

                dist = dist_display
                lateral = lateral_display
                vx, vy, omega = 0.0, 0.0, 0.0
                
                if is_walking and initial_pos:
                    if initial_yaw is None:
                        initial_yaw = float(robot.yaw)
                    if progress_time is None:
                        progress_time = current_time
                    dt = float(current_time - progress_time)
                    dt = float(np.clip(dt, 0.0, 0.1))
                    progress_time = current_time

                    # Prefer true odometry (state position). Otherwise integrate state velocity
                    # (or commanded velocity as a last resort) in the path frame.
                    use_mode = str(args.distance_source)
                    allow_pos = use_mode in {"auto", "state_pos"}
                    allow_vel = use_mode in {"auto", "state_vel"}
                    allow_wheel = use_mode in {"auto", "wheel_odom"}
                    allow_cmd = use_mode in {"auto", "cmd_int"}

                    if allow_pos and (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.pose_received):
                        dx = robot.position[0] - initial_pos[0]
                        dy = robot.position[1] - initial_pos[1]
                        dist = dx * math.cos(initial_yaw) + dy * math.sin(initial_yaw)
                        lateral = -dx * math.sin(initial_yaw) + dy * math.cos(initial_yaw)
                        distance_along_path = float(dist)
                        lateral_estimate = float(lateral)
                        distance_source = "STATE_POS"
                    else:
                        yaw = float(robot.yaw)
                        if allow_vel and (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.velocity_received):
                            # Unitree SDKs have historically reported velocity in either the body frame or
                            # a fixed/world frame depending on the message type/version. Use a simple
                            # alignment heuristic against the commanded motion to pick the better match.
                            vx0 = float(robot.measured_velocity[0])
                            vy0 = float(robot.measured_velocity[1])

                            # Candidate A: velocity is already world-frame.
                            vx_world_a, vy_world_a = vx0, vy0

                            # Candidate B: velocity is body-frame → rotate into world.
                            vx_world_b = vx0 * math.cos(yaw) - vy0 * math.sin(yaw)
                            vy_world_b = vx0 * math.sin(yaw) + vy0 * math.cos(yaw)

                            cmd_vx_body = float(robot.current_vx)
                            cmd_vy_body = float(robot.current_vy)
                            cmd_vx_world = cmd_vx_body * math.cos(yaw) - cmd_vy_body * math.sin(yaw)
                            cmd_vy_world = cmd_vx_body * math.sin(yaw) + cmd_vy_body * math.cos(yaw)

                            score_a = vx_world_a * cmd_vx_world + vy_world_a * cmd_vy_world
                            score_b = vx_world_b * cmd_vx_world + vy_world_b * cmd_vy_world
                            if score_a >= score_b:
                                vx_world, vy_world = vx_world_a, vy_world_a
                                distance_source = "STATE_VEL(W)"
                            else:
                                vx_world, vy_world = vx_world_b, vy_world_b
                                distance_source = "STATE_VEL(B)"
                        elif allow_wheel and (_SDK_AVAILABLE and _STATE_AVAILABLE and robot.wheel_odom_received):
                            # If only LowState is available (no base pose/velocity), use wheel-derived
                            # forward speed for distance. Keep lateral estimate driven by commanded vy.
                            cmd_vx_body = float(robot.current_vx)
                            cmd_vy_body = float(robot.current_vy)
                            cmd_speed = float(math.hypot(cmd_vx_body, cmd_vy_body))

                            wheel_speed = float(abs(robot.wheel_speed_mps))
                            # Track the last commanded forward direction so odometry still works while braking/coasting.
                            if abs(cmd_vx_body) > 0.05:
                                last_forward_sign = 1.0 if cmd_vx_body >= 0.0 else -1.0

                            # Basic sanity checks: if we command motion but wheel speed is ~0, fall back to CMD_INT.
                            odom_valid = True
                            if cmd_speed > 0.2 and wheel_speed < 0.03:
                                odom_valid = False
                            if wheel_speed > 6.0:
                                odom_valid = False

                            if odom_valid:
                                vx_body = last_forward_sign * wheel_speed
                                vy_body = cmd_vy_body
                                vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
                                vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
                                distance_source = "WHEEL_ODOM"
                            else:
                                vx_body = cmd_vx_body
                                vy_body = cmd_vy_body
                                vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
                                vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
                                distance_source = "CMD_INT"
                        elif allow_cmd:
                            vx_body = float(robot.current_vx)
                            vy_body = float(robot.current_vy)
                            vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
                            vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
                            distance_source = "CMD_INT"
                        else:
                            vx_body = float(robot.current_vx)
                            vy_body = float(robot.current_vy)
                            vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
                            vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
                            distance_source = "CMD_INT"

                        v_along = vx_world * math.cos(initial_yaw) + vy_world * math.sin(initial_yaw)
                        v_lateral = -vx_world * math.sin(initial_yaw) + vy_world * math.cos(initial_yaw)

                        distance_along_path = float(max(0.0, distance_along_path + v_along * dt))
                        lateral_estimate = float(lateral_estimate + v_lateral * dt)
                        dist = distance_along_path
                        lateral = lateral_estimate
                    dist *= float(distance_scale)
                    lateral *= float(distance_scale)
                    dist_display = float(dist)
                    lateral_display = float(lateral)
                    vx, vy, omega, complete, msg = controller.get_velocity_commands(
                        dist, lateral, current_time, robot.yaw, robot.get_measured_yaw_rate()
                    )
                    status = msg
                    if complete:
                        robot.stop()
                        is_walking = False
                        initial_pos = None
                        initial_yaw = None
                        progress_time = None
                        distance_along_path = 0.0
                        lateral_estimate = 0.0
                        status = "COMPLETE"
                    else:
                        robot.set_velocity(vx, vy, omega)
                else:
                    robot.stop()
                    progress_time = None

                speed_val = robot.get_measured_speed()
                live.update(update_ui(status, dist_display, lateral_display, speed_val, robot.yaw, is_walking, distance_source, distance_scale))
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
