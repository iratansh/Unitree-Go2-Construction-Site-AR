#!/usr/bin/env python3
"""
Behavioral Study Protocol Manager
Manages experimental trials for studying worker responses to robot dogs
"""

import json
import time
import datetime
import csv
from typing import List, Dict, Any
from dataclasses import dataclass, asdict
import os

@dataclass
class StudyParticipant:
    """Participant information"""
    id: str
    age: int
    experience_years: int
    prior_robot_exposure: str  # "none", "minimal", "moderate", "extensive"
    
@dataclass
class MarkerPosition:
    """Marker position in the study area"""
    id: str
    distance_from_path: float  # meters
    position_along_path: float  # meters (0-10)
    
@dataclass
class TrialCondition:
    """Experimental condition for a trial"""
    trial_id: int
    marker_id: str
    robot_speed: float  # m/s
    movement_mode: str  # "forward" or "rightward"
    gaze_angle: float  # degrees
    description: str

@dataclass
class TrialResult:
    """Results from a single trial"""
    participant_id: str
    trial_id: int
    condition: TrialCondition
    start_time: str
    end_time: str
    duration: float
    notes: str = ""
    
class StudyProtocol:
    """Manages the experimental protocol"""
    
    def __init__(self, study_name: str):
        self.study_name = study_name
        self.participants: List[StudyParticipant] = []
        self.markers: List[MarkerPosition] = []
        self.conditions: List[TrialCondition] = []
        self.results: List[TrialResult] = []
        
        # Create study directory
        self.study_dir = f"study_data/{study_name}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(self.study_dir, exist_ok=True)
        
        # Initialize default markers
        self._setup_default_markers()
        
        # Initialize default conditions
        self._setup_default_conditions()
    
    def _setup_default_markers(self):
        """Setup default marker positions"""
        # Markers at different distances from path
        distances = [1.0, 2.0, 3.0, 5.0]  # meters from path
        positions = [2.5, 5.0, 7.5]  # positions along 10m path
        
        marker_id = 1
        for dist in distances:
            for pos in positions:
                self.markers.append(MarkerPosition(
                    id=f"M{marker_id}",
                    distance_from_path=dist,
                    position_along_path=pos
                ))
                marker_id += 1
    
    def _setup_default_conditions(self):
        """Setup default experimental conditions"""
        conditions_config = [
            # Baseline conditions
            {"speed": 0.5, "mode": "forward", "gaze": 0, "desc": "Baseline forward"},
            {"speed": 0.3, "mode": "forward", "gaze": 0, "desc": "Slow forward"},
            {"speed": 1.0, "mode": "forward", "gaze": 0, "desc": "Fast forward"},
            
            # Gaze variations
            {"speed": 0.5, "mode": "forward", "gaze": 30, "desc": "Forward with side glance"},
            {"speed": 0.5, "mode": "forward", "gaze": -30, "desc": "Forward looking away"},
            
            # Rightward movement
            {"speed": 0.5, "mode": "rightward", "gaze": 0, "desc": "Rightward facing participant"},
            {"speed": 0.5, "mode": "rightward", "gaze": 45, "desc": "Rightward partial gaze"},
            {"speed": 0.3, "mode": "rightward", "gaze": 0, "desc": "Slow rightward facing"},
            
            # Approach simulations
            {"speed": 0.4, "mode": "forward", "gaze": 0, "desc": "Direct approach"},
            {"speed": 0.4, "mode": "forward", "gaze": 45, "desc": "Approach with averted gaze"},
        ]
        
        for i, config in enumerate(conditions_config, 1):
            self.conditions.append(TrialCondition(
                trial_id=i,
                marker_id="",  # To be set per trial
                robot_speed=config["speed"],
                movement_mode=config["mode"],
                gaze_angle=config["gaze"],
                description=config["desc"]
            ))
    
    def add_participant(self, participant: StudyParticipant):
        """Add a participant to the study"""
        self.participants.append(participant)
        print(f"Added participant: {participant.id}")
    
    def get_trial_sequence(self, participant_id: str, randomize: bool = True) -> List[TrialCondition]:
        """Generate trial sequence for a participant"""
        import random
        
        # Create trials for each marker-condition combination
        trials = []
        trial_counter = 1
        
        for marker in self.markers:
            for condition in self.conditions:
                trial = TrialCondition(
                    trial_id=trial_counter,
                    marker_id=marker.id,
                    robot_speed=condition.robot_speed,
                    movement_mode=condition.movement_mode,
                    gaze_angle=condition.gaze_angle,
                    description=f"{condition.description} @ {marker.id}"
                )
                trials.append(trial)
                trial_counter += 1
        
        if randomize:
            random.shuffle(trials)
        
        # Save trial sequence
        self._save_trial_sequence(participant_id, trials)
        
        return trials
    
    def _save_trial_sequence(self, participant_id: str, trials: List[TrialCondition]):
        """Save trial sequence to file"""
        filename = os.path.join(self.study_dir, f"trial_sequence_{participant_id}.json")
        with open(filename, 'w') as f:
            json.dump([asdict(t) for t in trials], f, indent=2)
    
    def record_trial_result(self, result: TrialResult):
        """Record the result of a trial"""
        self.results.append(result)
        self._save_results()
    
    def _save_results(self):
        """Save all results to CSV"""
        filename = os.path.join(self.study_dir, "trial_results.csv")
        
        with open(filename, 'w', newline='') as f:
            if self.results:
                fieldnames = [
                    'participant_id', 'trial_id', 'marker_id', 'robot_speed',
                    'movement_mode', 'gaze_angle', 'description',
                    'start_time', 'end_time', 'duration', 'notes'
                ]
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                
                for result in self.results:
                    row = {
                        'participant_id': result.participant_id,
                        'trial_id': result.trial_id,
                        'marker_id': result.condition.marker_id,
                        'robot_speed': result.condition.robot_speed,
                        'movement_mode': result.condition.movement_mode,
                        'gaze_angle': result.condition.gaze_angle,
                        'description': result.condition.description,
                        'start_time': result.start_time,
                        'end_time': result.end_time,
                        'duration': result.duration,
                        'notes': result.notes
                    }
                    writer.writerow(row)
    
    def save_study_config(self):
        """Save study configuration"""
        config = {
            'study_name': self.study_name,
            'markers': [asdict(m) for m in self.markers],
            'conditions': [asdict(c) for c in self.conditions],
            'participants': [asdict(p) for p in self.participants]
        }
        
        filename = os.path.join(self.study_dir, "study_config.json")
        with open(filename, 'w') as f:
            json.dump(config, f, indent=2)
    
    def print_marker_layout(self):
        """Print marker layout for setup"""
        print("\n=== Marker Layout ===")
        print("Distance from path (m) | Position along path (m)")
        print("-" * 45)
        
        for marker in self.markers:
            print(f"{marker.id}: {marker.distance_from_path:>6.1f}m | {marker.position_along_path:>6.1f}m")
        print("==================\n")
    
    def get_marker_info(self, marker_id: str) -> MarkerPosition:
        """Get information about a specific marker"""
        for marker in self.markers:
            if marker.id == marker_id:
                return marker
        return None


class TrialRunner:
    """Manages the execution of trials"""
    
    def __init__(self, protocol: StudyProtocol, controller):
        self.protocol = protocol
        self.controller = controller
        self.current_participant = None
        self.current_trials = []
        self.current_trial_index = 0
        
    def setup_participant(self, participant: StudyParticipant):
        """Setup for a new participant"""
        self.current_participant = participant
        self.protocol.add_participant(participant)
        self.current_trials = self.protocol.get_trial_sequence(participant.id)
        self.current_trial_index = 0
        
        print(f"\nParticipant {participant.id} setup complete")
        print(f"Total trials: {len(self.current_trials)}")
        
    def run_next_trial(self):
        """Run the next trial in the sequence"""
        if self.current_trial_index >= len(self.current_trials):
            print("All trials completed!")
            return False
        
        trial = self.current_trials[self.current_trial_index]
        marker = self.protocol.get_marker_info(trial.marker_id)
        
        print(f"\n=== Trial {self.current_trial_index + 1}/{len(self.current_trials)} ===")
        print(f"Condition: {trial.description}")
        print(f"Marker: {trial.marker_id} (Distance: {marker.distance_from_path}m, Position: {marker.position_along_path}m)")
        print(f"Speed: {trial.robot_speed} m/s")
        print(f"Mode: {trial.movement_mode}")
        print(f"Gaze: {trial.gaze_angle}°")
        
        input("\nPress Enter when participant is ready at marker position...")
        
        # Configure robot
        self.controller.reset_position()
        time.sleep(0.5)
        
        self.controller.set_speed(trial.robot_speed)
        self.controller.set_mode(trial.movement_mode)
        self.controller.set_gaze_angle(trial.gaze_angle)
        
        # Start trial
        start_time = datetime.datetime.now()
        print("\nStarting robot movement...")
        self.controller.start_movement()
        
        # Wait for completion
        input("Press Enter when trial is complete...")
        end_time = datetime.datetime.now()
        
        # Stop robot
        self.controller.stop_movement()
        
        # Record result
        duration = (end_time - start_time).total_seconds()
        notes = input("Any notes for this trial? (or press Enter to skip): ")
        
        result = TrialResult(
            participant_id=self.current_participant.id,
            trial_id=trial.trial_id,
            condition=trial,
            start_time=start_time.isoformat(),
            end_time=end_time.isoformat(),
            duration=duration,
            notes=notes
        )
        
        self.protocol.record_trial_result(result)
        self.current_trial_index += 1
        
        print(f"Trial completed. Duration: {duration:.1f}s")
        return True
    
    def skip_trial(self):
        """Skip the current trial"""
        if self.current_trial_index < len(self.current_trials):
            self.current_trial_index += 1
            print("Trial skipped")
    
    def print_progress(self):
        """Print current progress"""
        if self.current_participant:
            print(f"\nParticipant: {self.current_participant.id}")
            print(f"Progress: {self.current_trial_index}/{len(self.current_trials)} trials")
            if self.current_trial_index < len(self.current_trials):
                next_trial = self.current_trials[self.current_trial_index]
                print(f"Next: {next_trial.description}")


# Example usage script
if __name__ == "__main__":
    from go2_controller import Go2Controller  # Import the controller from the CLI script
    
    # Create study protocol
    study = StudyProtocol("construction_worker_response")
    
    # Print marker layout for physical setup
    study.print_marker_layout()
    
    # Example: Add a participant
    participant = StudyParticipant(
        id="P001",
        age=35,
        experience_years=10,
        prior_robot_exposure="minimal"
    )
    
    # Create controller (assuming it's imported)
    controller = Go2Controller()
    
    # Create trial runner
    runner = TrialRunner(study, controller)
    
    # Setup participant
    runner.setup_participant(participant)
    
    # Run trials
    print("\nStarting experimental trials...")
    print("Commands: 'next' - run next trial, 'skip' - skip trial, 'progress' - show progress, 'quit' - exit")
    
    while True:
        command = input("\nstudy> ").strip().lower()
        
        if command == "quit":
            break
        elif command == "next":
            if not runner.run_next_trial():
                print("Study complete!")
                break
        elif command == "skip":
            runner.skip_trial()
        elif command == "progress":
            runner.print_progress()
        else:
            print("Unknown command")
    
    # Save final configuration
    study.save_study_config()
    print(f"\nStudy data saved to: {study.study_dir}")