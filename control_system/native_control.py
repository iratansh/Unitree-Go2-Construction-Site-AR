#!/usr/bin/env python3
"""
Go2 Native Control - Using Official Unitree SDK
Lowest latency possible - direct API calls
"""

import sys
import os
import time
import math
import select
import termios
import tty

# Add SDK to path
sys.path.append('/home/unitree/unitree_sdk2_python')

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class Go2NativeControl:
    def __init__(self):
        self.client = SportClient()
        self.client.SetTimeout(3.0)
        self.client.Init()

        print("🤖 Go2 Native SDK Control Initialized")

        # Variables for path following
        self.position = 0.0  # Distance traveled
        self.target_distance = 15.0  # Target distance in meters
        self.speed = 0.5  # m/s
        self.direction = 1  # 1 = forward, -1 = backward

    def stand_up(self):
        """Stand up the robot"""
        print("🤖 Standing up...")
        code = self.client.StandUp()
        if code == 0:
            print("✅ Stand up successful")
        else:
            print(f"❌ Stand up failed: {code}")
        return code == 0

    def sit_down(self):
        """Sit down the robot"""
        print("🤖 Sitting down...")
        code = self.client.StandDown()
        if code == 0:
            print("✅ Sit down successful")
        else:
            print(f"❌ Sit down failed: {code}")
        return code == 0

    def move(self, vx, vy, vyaw):
        """Send velocity command"""
        code = self.client.Move(vx, vy, vyaw)
        return code == 0

    def stop(self):
        """Stop all movement"""
        code = self.client.StopMove()
        return code == 0

    def get_state(self):
        """Get robot state"""
        keys = ["pose", "velocity", "imu"]
        code, data = self.client.GetState(keys)
        if code == 0:
            return data
        return None


def get_key_press():
    """Non-blocking key detection"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    if len(sys.argv) < 2:
        script = os.path.basename(sys.argv[0])
        print(f"Usage: python3 {script} <network_interface>")
        print("Available interfaces: lo, eth0, l4tbr0, rndis0, usb0")
        print(f"Try: python3 {script} eth0")
        sys.exit(-1)

    print("WARNING: Ensure no obstacles around the robot!")
    input("Press Enter to continue...")

    # Initialize the channel factory with network interface
    ChannelFactoryInitialize(0, sys.argv[1])

    print("="*60)
    print("         🤖 GO2 NATIVE SDK CONTROL")
    print("="*60)

    # Initialize robot
    robot = Go2NativeControl()

    # Stand up
    if not robot.stand_up():
        print("❌ Failed to stand up. Exiting.")
        return

    time.sleep(3)  # Wait for robot to stand

    print("\n🎮 Controls:")
    print("   SPACE  : Start/Stop automatic path")
    print("   WASD   : Manual control")
    print("   QE     : Rotate left/right")
    print("   R      : Stand up")
    print("   F      : Sit down")
    print("   ESC    : Exit")
    print("="*60)

    # Set terminal to raw mode
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    try:
        auto_mode = False
        vx = vy = vyaw = 0.0

        while True:
            key = get_key_press()

            if key:
                if key == ' ':  # Toggle auto mode
                    auto_mode = not auto_mode
                    if auto_mode:
                        print(f"\n🚀 Auto mode ON - Going {robot.target_distance}m forward")
                        robot.position = 0.0
                    else:
                        print("\n⏸️ Auto mode OFF")
                        robot.stop()

                elif key == 'w':
                    vx = robot.speed
                    auto_mode = False
                elif key == 's':
                    vx = -robot.speed
                    auto_mode = False
                elif key == 'a':
                    vy = robot.speed
                    auto_mode = False
                elif key == 'd':
                    vy = -robot.speed
                    auto_mode = False
                elif key == 'q':
                    vyaw = 0.5
                    auto_mode = False
                elif key == 'e':
                    vyaw = -0.5
                    auto_mode = False
                elif key == 'r':
                    robot.stand_up()
                elif key == 'f':
                    robot.sit_down()
                elif key == '\x1b':  # ESC
                    break
                else:
                    vx = vy = vyaw = 0.0

                if not auto_mode and key in 'wsadqe':
                    robot.move(vx, vy, vyaw)
                    print(f"\rManual: vx={vx:.1f} vy={vy:.1f} ω={vyaw:.1f}", end='', flush=True)
                    time.sleep(0.1)
                    robot.stop()  # Stop after brief movement

            # Auto mode path following
            if auto_mode:
                if robot.position < robot.target_distance:
                    robot.move(robot.speed * robot.direction, 0, 0)
                    # estimate distance based on loop rate (0.05s sleep below)
                    robot.position += robot.speed * 0.05
                    remaining = robot.target_distance - robot.position
                    print(f"\r🏃 Auto: {robot.position:.1f}m / {robot.target_distance:.1f}m (remaining: {remaining:.1f}m)", end='', flush=True)
                else:
                    robot.stop()
                    auto_mode = False
                    print("\n🎯 Reached target distance!")

            time.sleep(0.05)  # 20Hz control loop

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        robot.stop()
        time.sleep(0.5)
        robot.sit_down()
        print("\n🏁 Session ended")


if __name__ == "__main__":
    main()