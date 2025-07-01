#!/usr/bin/env python3
"""
Go2 Robot Control CLI for Behavioral Study
Controls the Unitree Go2 robot in Unity via UDP communication
"""

import socket
import json
import threading
import time
import argparse
from enum import Enum
from typing import Dict, Any
import sys
import select

class MovementMode(Enum):
    FORWARD = "forward"
    RIGHTWARD = "rightward"

class Go2Controller:
    def __init__(self, unity_ip="127.0.0.1", send_port=12345, receive_port=12346):
        self.unity_ip = unity_ip
        self.send_port = send_port
        self.receive_port = receive_port
        
        # Robot state
        self.speed = 0.5  # m/s
        self.mode = MovementMode.FORWARD
        self.gaze_angle = 0.0  # degrees
        self.is_moving = False
        
        # Status from Unity
        self.robot_status = {}
        
        # Networking
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receive_socket.bind(("", self.receive_port))
        
        # Start receive thread
        self.receiving = True
        self.receive_thread = threading.Thread(target=self._receive_status)
        self.receive_thread.daemon = True
        self.receive_thread.start()
    
    def _receive_status(self):
        """Receive status updates from Unity"""
        while self.receiving:
            try:
                data, addr = self.receive_socket.recvfrom(1024)
                self.robot_status = json.loads(data.decode())
            except Exception as e:
                if self.receiving:
                    print(f"Receive error: {e}")
    
    def send_command(self, command: Dict[str, Any]):
        """Send command to Unity"""
        try:
            data = json.dumps(command).encode()
            self.send_socket.sendto(data, (self.unity_ip, self.send_port))
        except Exception as e:
            print(f"Send error: {e}")
    
    def start_movement(self):
        """Start robot movement"""
        self.is_moving = True
        command = {
            "speed": self.speed,
            "mode": self.mode.value,
            "gazeAngle": self.gaze_angle,
            "start": True,
            "stop": False,
            "reset": False
        }
        self.send_command(command)
        print("✓ Movement started")
    
    def stop_movement(self):
        """Stop robot movement"""
        self.is_moving = False
        command = {
            "speed": 0,
            "mode": self.mode.value,
            "gazeAngle": self.gaze_angle,
            "start": False,
            "stop": True,
            "reset": False
        }
        self.send_command(command)
        print("✓ Movement stopped")
    
    def reset_position(self):
        """Reset robot to start position"""
        self.is_moving = False
        command = {
            "speed": 0,
            "mode": self.mode.value,
            "gazeAngle": self.gaze_angle,
            "start": False,
            "stop": False,
            "reset": True
        }
        self.send_command(command)
        print("✓ Position reset")
    
    def set_speed(self, speed: float):
        """Set robot speed (m/s)"""
        self.speed = max(0.1, min(2.0, speed))  # Clamp between 0.1 and 2.0 m/s
        if self.is_moving:
            self.update_movement()
        print(f"✓ Speed set to {self.speed:.2f} m/s")
    
    def set_mode(self, mode: MovementMode):
        """Set movement mode"""
        self.mode = mode
        if self.is_moving:
            self.update_movement()
        print(f"✓ Mode set to {mode.value}")
    
    def set_gaze_angle(self, angle: float):
        """Set gaze angle relative to movement direction"""
        self.gaze_angle = max(-90, min(90, angle))  # Clamp between -90 and 90 degrees
        if self.is_moving:
            self.update_movement()
        print(f"✓ Gaze angle set to {self.gaze_angle:.1f}°")
    
    def update_movement(self):
        """Update movement parameters while moving"""
        command = {
            "speed": self.speed,
            "mode": self.mode.value,
            "gazeAngle": self.gaze_angle,
            "start": False,
            "stop": False,
            "reset": False
        }
        self.send_command(command)
    
    def print_status(self):
        """Print current robot status"""
        print("\n--- Robot Status ---")
        print(f"Speed: {self.speed:.2f} m/s")
        print(f"Mode: {self.mode.value}")
        print(f"Gaze Angle: {self.gaze_angle:.1f}°")
        print(f"Moving: {self.is_moving}")
        
        if self.robot_status:
            print(f"Position: {self.robot_status.get('position', 'N/A')}")
            print(f"Distance Traveled: {self.robot_status.get('distanceTraveled', 0):.2f} m")
            print(f"Orientation: {self.robot_status.get('orientation', 0):.1f}°")
        print("-------------------\n")
    
    def cleanup(self):
        """Clean up resources"""
        self.receiving = False
        self.send_socket.close()
        self.receive_socket.close()

def print_help():
    """Print available commands"""
    print("\n=== Go2 Robot Control Commands ===")
    print("start          - Start robot movement")
    print("stop           - Stop robot movement")
    print("reset          - Reset robot to start position")
    print("speed <value>  - Set speed (0.1-2.0 m/s)")
    print("mode <f/r>     - Set mode (f=forward, r=rightward)")
    print("gaze <angle>   - Set gaze angle (-90 to 90 degrees)")
    print("status         - Show current status")
    print("run <preset>   - Run preset configuration")
    print("help           - Show this help")
    print("quit           - Exit program")
    print("==================================\n")

def run_preset(controller, preset_name):
    """Run predefined movement presets for the study"""
    presets = {
        "baseline": {
            "description": "Baseline forward movement",
            "speed": 0.5,
            "mode": MovementMode.FORWARD,
            "gaze": 0
        },
        "slow_forward": {
            "description": "Slow forward movement",
            "speed": 0.3,
            "mode": MovementMode.FORWARD,
            "gaze": 0
        },
        "fast_forward": {
            "description": "Fast forward movement",
            "speed": 1.0,
            "mode": MovementMode.FORWARD,
            "gaze": 0
        },
        "rightward_facing": {
            "description": "Rightward movement facing participant",
            "speed": 0.5,
            "mode": MovementMode.RIGHTWARD,
            "gaze": 0
        },
        "rightward_glancing": {
            "description": "Rightward movement with occasional glances",
            "speed": 0.5,
            "mode": MovementMode.RIGHTWARD,
            "gaze": 30
        },
        "approach_direct": {
            "description": "Direct approach with eye contact",
            "speed": 0.4,
            "mode": MovementMode.FORWARD,
            "gaze": 0
        },
        "approach_averted": {
            "description": "Approach with averted gaze",
            "speed": 0.4,
            "mode": MovementMode.FORWARD,
            "gaze": 45
        }
    }
    
    if preset_name not in presets:
        print(f"Unknown preset: {preset_name}")
        print(f"Available presets: {', '.join(presets.keys())}")
        return
    
    preset = presets[preset_name]
    print(f"\nRunning preset: {preset['description']}")
    
    controller.reset_position()
    time.sleep(0.5)
    
    controller.set_speed(preset['speed'])
    controller.set_mode(preset['mode'])
    controller.set_gaze_angle(preset['gaze'])
    
    time.sleep(0.5)
    controller.start_movement()

def main():
    parser = argparse.ArgumentParser(description="Go2 Robot Control CLI")
    parser.add_argument("--ip", default="127.0.0.1", help="Unity IP address")
    parser.add_argument("--send-port", type=int, default=12345, help="Port to send commands")
    parser.add_argument("--receive-port", type=int, default=12346, help="Port to receive status")
    
    args = parser.parse_args()
    
    controller = Go2Controller(args.ip, args.send_port, args.receive_port)
    
    print("Go2 Robot Control CLI")
    print(f"Connected to Unity at {args.ip}:{args.send_port}")
    print_help()
    
    try:
        while True:
            try:
                command = input("go2> ").strip().lower().split()
                
                if not command:
                    continue
                
                if command[0] == "quit":
                    break
                elif command[0] == "help":
                    print_help()
                elif command[0] == "start":
                    controller.start_movement()
                elif command[0] == "stop":
                    controller.stop_movement()
                elif command[0] == "reset":
                    controller.reset_position()
                elif command[0] == "status":
                    controller.print_status()
                elif command[0] == "speed" and len(command) > 1:
                    try:
                        speed = float(command[1])
                        controller.set_speed(speed)
                    except ValueError:
                        print("Invalid speed value")
                elif command[0] == "mode" and len(command) > 1:
                    if command[1] in ['f', 'forward']:
                        controller.set_mode(MovementMode.FORWARD)
                    elif command[1] in ['r', 'rightward']:
                        controller.set_mode(MovementMode.RIGHTWARD)
                    else:
                        print("Invalid mode. Use 'f' for forward or 'r' for rightward")
                elif command[0] == "gaze" and len(command) > 1:
                    try:
                        angle = float(command[1])
                        controller.set_gaze_angle(angle)
                    except ValueError:
                        print("Invalid angle value")
                elif command[0] == "run" and len(command) > 1:
                    run_preset(controller, command[1])
                else:
                    print(f"Unknown command: {' '.join(command)}")
                
            except KeyboardInterrupt:
                print("\nUse 'quit' to exit")
                continue
    
    finally:
        print("\nShutting down...")
        controller.stop_movement()
        controller.cleanup()

if __name__ == "__main__":
    main()