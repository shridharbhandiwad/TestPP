#!/usr/bin/env python3
"""
Automotive Perception Function Logic Emulator

This script emulates the C++ automotive perception post-processing functions
to determine which functions would execute their main logic based on input values.
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, simpledialog
import math
import json
from dataclasses import dataclass, field
from typing import List, Dict, Any
import os

@dataclass
class ObjectState:
    """Represents the object state with position and velocity"""
    x: float = 0.0  # longitudinal position
    y: float = 0.0  # lateral position
    vx: float = 0.0  # longitudinal velocity
    vy: float = 0.0  # lateral velocity

@dataclass
class SensorFilterFusHelper:
    """Sensor fusion helper data"""
    total_num_radar_updates: int = 0
    total_num_video_updates: int = 0
    total_num_front_left_corner_updates: int = 0
    total_num_front_right_corner_updates: int = 0
    total_num_front_center_location_radar_updates: int = 0
    updates_since_last_video_update: int = 0
    updates_since_last_radar_update: int = 0
    updates_since_last_front_center_video_update: int = 0
    updates_since_last_front_center_location_radar_update: int = 0
    updates_since_last_front_left_corner_update: int = 0
    updates_since_last_front_right_corner_update: int = 0
    updates_since_last_update: int = 0
    is_good_quality_fused_object: bool = False
    is_trustworthy_object: bool = False

@dataclass
class ObjectData:
    """Complete object data structure"""
    # Basic object attributes
    state: ObjectState = field(default_factory=ObjectState)
    is_object_vru: bool = False
    rcs: float = 0.0
    num_cycles_existing: int = 0
    filter_type: str = "LA"  # LA, WNJ, etc.
    
    # Probabilities
    prob_has_been_observed_moving: float = 0.0
    prob_is_currently_moving: float = 0.0
    
    # Sensor fusion data
    sensor_filter_fus_helper: SensorFilterFusHelper = field(default_factory=SensorFilterFusHelper)
    
    # Innovation data
    avg_dx_innovation: float = 0.0
    radar_based_innovation: List[float] = field(default_factory=lambda: [0.0, 0.0])
    video_based_innovation: List[float] = field(default_factory=lambda: [0.0, 0.0])
    radar_raw_alpha_innovation: float = 0.0
    video_raw_alpha_innovation: float = 0.0
    
    # Elevation
    elevation: float = 0.0
    elevation_is_valid: bool = False
    
    # Micro doppler
    number_micro_doppler_cycles: int = 0
    expected_vr_high_enough_for_mu_doppler_counter: int = 0
    
    # Various counters
    split_counter: int = 0
    stopping_split_counter: int = 0
    stationary_locations_only_counter: int = 0
    non_plausible_location_cnt: int = 0
    bad_sensor_based_inno_count: int = 0
    vy_inconsistent: int = 0
    object_orientation_unreliable_count: int = 0
    num_cycles_no_orientation_update: int = 0
    total_num_cycles_with_oncoming_locations: int = 0
    num_consecutive_cycles_without_oncoming_locations: int = 0
    transferred_from_sep_cycle: int = 0
    
    # Object type and classification
    most_probable_conditional_type: str = "PEDESTRIAN"  # PEDESTRIAN, CAR, TRUCK, 2WHEELER, etc.
    p_non_obstacle_rcs_only_classifier: float = 0.0
    
    # Dimensions
    length: float = 1.8
    width: float = 0.6
    
    # Angular data
    yaw_angle: float = 0.0
    facing_angle: float = 0.0
    
    # Video related
    w_exist_of_associated_video_object: float = 0.0
    video_inv_ttc: float = 0.0
    recently_used_video_measurement_handle_valid: bool = False
    recently_used_video_measurement_handle: int = 0
    object_id_10bit: int = 1
    created_by_video_with_high_vy: bool = False
    num_cycles_since_last_video_update_with_angular_velocity: int = 0
    
    # Suppression flags
    is_suppressed_until_next_video_update: bool = False
    is_suppressed_due_to_video_otc_post_processing: bool = False
    is_updated_with_stat_loc_with_high_mdoppler_with_outgoing_vr: bool = False
    is_orientation_implausible_compared_2_vid: bool = False
    
    # Other accumulated values
    vy_unreliable_accumulated: float = 0.0

@dataclass
class EgoVehicleData:
    """Ego vehicle motion data"""
    velocity_x: float = 0.0  # m/s
    acceleration_y: float = 0.0  # m/s²
    yaw_rate: float = 0.0  # rad/s

@dataclass
class Parameters:
    """Configuration parameters"""
    is_micro_doppler_check_enabled: bool = True
    min_vru_micro_doppler_cycles: int = 1
    is_micro_doppler_check_on_crossing_vru_applied: bool = True
    is_micro_doppler_check_on_stationary_vru_applied: bool = True
    innovation_check_dx_threshold: float = 50.0
    innovation_check_dy_threshold: float = 50.0
    implausible_vy_thresh_la_hypo: float = 8.0
    split_detection_cnt_max_val: int = 3
    implausible_rcs_thresh: float = -9.5
    max_longitudinal_distance_for_rcs_countermeasure: float = 20.0
    elevation_check_dx_limits: List[float] = field(default_factory=lambda: [0.0, 100.0])
    elevation_check_dz_thresholds: List[float] = field(default_factory=lambda: [2.0, 3.0])

class AutomotivePerceptionEmulator:
    """Main emulation class for automotive perception functions"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Automotive Perception Function Emulator")
        self.root.geometry("1200x800")
        
        # Data structures
        self.obj_data = ObjectData()
        self.ego_data = EgoVehicleData()
        self.params = Parameters()
        self.abs_vel_over_ground = [0.0, 0.0]
        self.is_mpc3_used = False
        self.dep_obj_probably_video_ghost = False
        
        # Add interactive mode flag
        self.interactive_mode = tk.BooleanVar(value=False)
        
        # GUI components
        self.obj_entries = {}
        self.sensor_entries = {}
        self.ego_entries = {}
        self.results_text = None
        
        self.setup_gui()
        
    def setup_gui(self):
        """Setup the main GUI"""
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create notebook for tabs
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Create tabs
        self.create_object_tab()
        self.create_sensor_tab()
        self.create_ego_tab()
        self.create_results_tab()
        
        # Create control buttons
        self.create_control_buttons()
        
    def create_object_tab(self):
        """Create object data input tab"""
        self.obj_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.obj_frame, text="Object Data")
        
        # Object state frame
        state_frame = ttk.LabelFrame(self.obj_frame, text="Object State")
        state_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Position
        tk.Label(state_frame, text="Position X (m):").grid(row=0, column=0, sticky="w")
        self.obj_entries['pos_x'] = tk.Entry(state_frame, width=10)
        self.obj_entries['pos_x'].grid(row=0, column=1, padx=5)
        self.obj_entries['pos_x'].insert(0, "25.0")
        
        tk.Label(state_frame, text="Position Y (m):").grid(row=0, column=2, sticky="w")
        self.obj_entries['pos_y'] = tk.Entry(state_frame, width=10)
        self.obj_entries['pos_y'].grid(row=0, column=3, padx=5)
        self.obj_entries['pos_y'].insert(0, "1.5")
        
        # Velocity
        tk.Label(state_frame, text="Velocity X (m/s):").grid(row=1, column=0, sticky="w")
        self.obj_entries['vel_x'] = tk.Entry(state_frame, width=10)
        self.obj_entries['vel_x'].grid(row=1, column=1, padx=5)
        self.obj_entries['vel_x'].insert(0, "0.5")
        
        tk.Label(state_frame, text="Velocity Y (m/s):").grid(row=1, column=2, sticky="w")
        self.obj_entries['vel_y'] = tk.Entry(state_frame, width=10)
        self.obj_entries['vel_y'].grid(row=1, column=3, padx=5)
        self.obj_entries['vel_y'].insert(0, "3.0")
        
        # Object properties frame
        props_frame = ttk.LabelFrame(self.obj_frame, text="Object Properties")
        props_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # VRU flag
        tk.Label(props_frame, text="Is VRU:").grid(row=0, column=0, sticky="w")
        self.obj_entries['is_vru'] = tk.BooleanVar(value=True)
        tk.Checkbutton(props_frame, variable=self.obj_entries['is_vru']).grid(row=0, column=1)
        
        # RCS
        tk.Label(props_frame, text="RCS (dBm²):").grid(row=0, column=2, sticky="w")
        self.obj_entries['rcs'] = tk.Entry(props_frame, width=10)
        self.obj_entries['rcs'].grid(row=0, column=3, padx=5)
        self.obj_entries['rcs'].insert(0, "-8.0")
        
        # Number of cycles
        tk.Label(props_frame, text="Cycles Existing:").grid(row=1, column=0, sticky="w")
        self.obj_entries['num_cycles'] = tk.Entry(props_frame, width=10)
        self.obj_entries['num_cycles'].grid(row=1, column=1, padx=5)
        self.obj_entries['num_cycles'].insert(0, "8")
        
        # Filter type
        tk.Label(props_frame, text="Filter Type:").grid(row=1, column=2, sticky="w")
        self.obj_entries['filter_type'] = ttk.Combobox(props_frame, values=["LA", "WNJ", "KF"], width=8)
        self.obj_entries['filter_type'].grid(row=1, column=3, padx=5)
        self.obj_entries['filter_type'].set("LA")
        
        # Probabilities frame
        prob_frame = ttk.LabelFrame(self.obj_frame, text="Probabilities")
        prob_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(prob_frame, text="Prob Has Been Moving:").grid(row=0, column=0, sticky="w")
        self.obj_entries['prob_has_been_moving'] = tk.Entry(prob_frame, width=10)
        self.obj_entries['prob_has_been_moving'].grid(row=0, column=1, padx=5)
        self.obj_entries['prob_has_been_moving'].insert(0, "0.3")
        
        tk.Label(prob_frame, text="Prob Currently Moving:").grid(row=0, column=2, sticky="w")
        self.obj_entries['prob_currently_moving'] = tk.Entry(prob_frame, width=10)
        self.obj_entries['prob_currently_moving'].grid(row=0, column=3, padx=5)
        self.obj_entries['prob_currently_moving'].insert(0, "0.2")
        
        # Elevation frame
        elev_frame = ttk.LabelFrame(self.obj_frame, text="Elevation")
        elev_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(elev_frame, text="Elevation (m):").grid(row=0, column=0, sticky="w")
        self.obj_entries['elevation'] = tk.Entry(elev_frame, width=10)
        self.obj_entries['elevation'].grid(row=0, column=1, padx=5)
        self.obj_entries['elevation'].insert(0, "0.0")
        
        tk.Label(elev_frame, text="Elevation Valid:").grid(row=0, column=2, sticky="w")
        self.obj_entries['elevation_valid'] = tk.BooleanVar(value=True)
        tk.Checkbutton(elev_frame, variable=self.obj_entries['elevation_valid']).grid(row=0, column=3)
        
        # Innovation frame
        innov_frame = ttk.LabelFrame(self.obj_frame, text="Innovation")
        innov_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(innov_frame, text="Avg DX Innovation:").grid(row=0, column=0, sticky="w")
        self.obj_entries['avg_dx_innovation'] = tk.Entry(innov_frame, width=10)
        self.obj_entries['avg_dx_innovation'].grid(row=0, column=1, padx=5)
        self.obj_entries['avg_dx_innovation'].insert(0, "0.5")
        
        tk.Label(innov_frame, text="Radar Innov DR:").grid(row=0, column=2, sticky="w")
        self.obj_entries['radar_innovation_dr'] = tk.Entry(innov_frame, width=10)
        self.obj_entries['radar_innovation_dr'].grid(row=0, column=3, padx=5)
        self.obj_entries['radar_innovation_dr'].insert(0, "0.0")
        
        tk.Label(innov_frame, text="Radar Innov Alpha:").grid(row=1, column=0, sticky="w")
        self.obj_entries['radar_innovation_alpha'] = tk.Entry(innov_frame, width=10)
        self.obj_entries['radar_innovation_alpha'].grid(row=1, column=1, padx=5)
        self.obj_entries['radar_innovation_alpha'].insert(0, "0.0")
        
        tk.Label(innov_frame, text="Video Innov DR:").grid(row=1, column=2, sticky="w")
        self.obj_entries['video_innovation_dr'] = tk.Entry(innov_frame, width=10)
        self.obj_entries['video_innovation_dr'].grid(row=1, column=3, padx=5)
        self.obj_entries['video_innovation_dr'].insert(0, "0.0")
        
        tk.Label(innov_frame, text="Video Innov Alpha:").grid(row=2, column=0, sticky="w")
        self.obj_entries['video_innovation_alpha'] = tk.Entry(innov_frame, width=10)
        self.obj_entries['video_innovation_alpha'].grid(row=2, column=1, padx=5)
        self.obj_entries['video_innovation_alpha'].insert(0, "0.0")
        
        # Micro doppler frame
        micro_frame = ttk.LabelFrame(self.obj_frame, text="Micro Doppler")
        micro_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(micro_frame, text="Micro Doppler Cycles:").grid(row=0, column=0, sticky="w")
        self.obj_entries['micro_doppler_cycles'] = tk.Entry(micro_frame, width=10)
        self.obj_entries['micro_doppler_cycles'].grid(row=0, column=1, padx=5)
        self.obj_entries['micro_doppler_cycles'].insert(0, "0")
        
        tk.Label(micro_frame, text="Expected VR Counter:").grid(row=0, column=2, sticky="w")
        self.obj_entries['expected_vr_counter'] = tk.Entry(micro_frame, width=10)
        self.obj_entries['expected_vr_counter'].grid(row=0, column=3, padx=5)
        self.obj_entries['expected_vr_counter'].insert(0, "5")
        
    def create_sensor_tab(self):
        """Create sensor data input tab"""
        self.sensor_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.sensor_frame, text="Sensor Data")
        
        # Sensor updates frame
        updates_frame = ttk.LabelFrame(self.sensor_frame, text="Sensor Updates")
        updates_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(updates_frame, text="Total Radar Updates:").grid(row=0, column=0, sticky="w")
        self.sensor_entries['total_radar_updates'] = tk.Entry(updates_frame, width=10)
        self.sensor_entries['total_radar_updates'].grid(row=0, column=1, padx=5)
        self.sensor_entries['total_radar_updates'].insert(0, "5")
        
        tk.Label(updates_frame, text="Total Video Updates:").grid(row=0, column=2, sticky="w")
        self.sensor_entries['total_video_updates'] = tk.Entry(updates_frame, width=10)
        self.sensor_entries['total_video_updates'].grid(row=0, column=3, padx=5)
        self.sensor_entries['total_video_updates'].insert(0, "3")
        
        tk.Label(updates_frame, text="FC Location Radar Updates:").grid(row=1, column=0, sticky="w")
        self.sensor_entries['fc_location_radar_updates'] = tk.Entry(updates_frame, width=10)
        self.sensor_entries['fc_location_radar_updates'].grid(row=1, column=1, padx=5)
        self.sensor_entries['fc_location_radar_updates'].insert(0, "5")
        
        tk.Label(updates_frame, text="FL Corner Updates:").grid(row=1, column=2, sticky="w")
        self.sensor_entries['fl_corner_updates'] = tk.Entry(updates_frame, width=10)
        self.sensor_entries['fl_corner_updates'].grid(row=1, column=3, padx=5)
        self.sensor_entries['fl_corner_updates'].insert(0, "0")
        
        tk.Label(updates_frame, text="FR Corner Updates:").grid(row=2, column=0, sticky="w")
        self.sensor_entries['fr_corner_updates'] = tk.Entry(updates_frame, width=10)
        self.sensor_entries['fr_corner_updates'].grid(row=2, column=1, padx=5)
        self.sensor_entries['fr_corner_updates'].insert(0, "0")
        
        tk.Label(updates_frame, text="Since Last Video:").grid(row=2, column=2, sticky="w")
        self.sensor_entries['since_last_video'] = tk.Entry(updates_frame, width=10)
        self.sensor_entries['since_last_video'].grid(row=2, column=3, padx=5)
        self.sensor_entries['since_last_video'].insert(0, "2")
        
        tk.Label(updates_frame, text="Since Last Radar:").grid(row=3, column=0, sticky="w")
        self.sensor_entries['since_last_radar'] = tk.Entry(updates_frame, width=10)
        self.sensor_entries['since_last_radar'].grid(row=3, column=1, padx=5)
        self.sensor_entries['since_last_radar'].insert(0, "0")
        
        # Quality flags frame
        quality_frame = ttk.LabelFrame(self.sensor_frame, text="Quality Flags")
        quality_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(quality_frame, text="Good Quality Fused:").grid(row=0, column=0, sticky="w")
        self.sensor_entries['good_quality_fused'] = tk.BooleanVar(value=True)
        tk.Checkbutton(quality_frame, variable=self.sensor_entries['good_quality_fused']).grid(row=0, column=1)
        
        tk.Label(quality_frame, text="Trustworthy Object:").grid(row=0, column=2, sticky="w")
        self.sensor_entries['trustworthy_object'] = tk.BooleanVar(value=True)
        tk.Checkbutton(quality_frame, variable=self.sensor_entries['trustworthy_object']).grid(row=0, column=3)
        
    def create_ego_tab(self):
        """Create ego vehicle data input tab"""
        self.ego_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.ego_frame, text="Ego Vehicle")
        
        # Ego motion frame
        motion_frame = ttk.LabelFrame(self.ego_frame, text="Ego Motion")
        motion_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(motion_frame, text="Velocity X (m/s):").grid(row=0, column=0, sticky="w")
        self.ego_entries['velocity_x'] = tk.Entry(motion_frame, width=10)
        self.ego_entries['velocity_x'].grid(row=0, column=1, padx=5)
        self.ego_entries['velocity_x'].insert(0, "13.9")
        
        tk.Label(motion_frame, text="Acceleration Y (m/s²):").grid(row=0, column=2, sticky="w")
        self.ego_entries['acceleration_y'] = tk.Entry(motion_frame, width=10)
        self.ego_entries['acceleration_y'].grid(row=0, column=3, padx=5)
        self.ego_entries['acceleration_y'].insert(0, "0.0")
        
        tk.Label(motion_frame, text="Yaw Rate (rad/s):").grid(row=1, column=0, sticky="w")
        self.ego_entries['yaw_rate'] = tk.Entry(motion_frame, width=10)
        self.ego_entries['yaw_rate'].grid(row=1, column=1, padx=5)
        self.ego_entries['yaw_rate'].insert(0, "0.0")
        
        # Absolute velocity frame
        abs_vel_frame = ttk.LabelFrame(self.ego_frame, text="Absolute Velocity Over Ground")
        abs_vel_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(abs_vel_frame, text="Abs Vel X (m/s):").grid(row=0, column=0, sticky="w")
        self.ego_entries['abs_vel_x'] = tk.Entry(abs_vel_frame, width=10)
        self.ego_entries['abs_vel_x'].grid(row=0, column=1, padx=5)
        self.ego_entries['abs_vel_x'].insert(0, "0.5")
        
        tk.Label(abs_vel_frame, text="Abs Vel Y (m/s):").grid(row=0, column=2, sticky="w")
        self.ego_entries['abs_vel_y'] = tk.Entry(abs_vel_frame, width=10)
        self.ego_entries['abs_vel_y'].grid(row=0, column=3, padx=5)
        self.ego_entries['abs_vel_y'].insert(0, "3.0")
        
        # Additional flags frame
        flags_frame = ttk.LabelFrame(self.ego_frame, text="Additional Flags")
        flags_frame.pack(fill=tk.X, padx=5, pady=5)
        
        tk.Label(flags_frame, text="Is MPC3 Used:").grid(row=0, column=0, sticky="w")
        self.ego_entries['is_mpc3_used'] = tk.BooleanVar(value=False)
        tk.Checkbutton(flags_frame, variable=self.ego_entries['is_mpc3_used']).grid(row=0, column=1)
        
    def create_results_tab(self):
        """Create results display tab"""
        self.results_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.results_frame, text="Results")
        
        # Results text area
        self.results_text = scrolledtext.ScrolledText(self.results_frame, height=25, width=80)
        self.results_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
    def create_control_buttons(self):
        """Create control buttons"""
        control_frame = ttk.Frame(self.root)
        control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Interactive mode checkbox
        tk.Checkbutton(control_frame, text="Interactive Input Mode", variable=self.interactive_mode).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(control_frame, text="Evaluate Functions", command=self.evaluate_functions).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Load Example", command=self.load_example).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Save Config", command=self.save_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Load Config", command=self.load_config).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Clear Results", command=self.clear_results).pack(side=tk.LEFT, padx=5)
    
    def get_interactive_input(self, field_name, description, field_type, default_value=None):
        """Get interactive input for a specific field"""
        prompt = f"Enter {description}:"
        if default_value is not None:
            prompt += f" (default: {default_value})"
        
        if field_type == "bool":
            result = messagebox.askyesno("Interactive Input", prompt)
            return result
        elif field_type == "int":
            while True:
                try:
                    value = simpledialog.askstring("Interactive Input", prompt)
                    if value is None:  # User cancelled
                        return default_value if default_value is not None else 0
                    if value == "" and default_value is not None:
                        return default_value
                    return int(value)
                except ValueError:
                    messagebox.showerror("Error", "Please enter a valid integer.")
        elif field_type == "float":
            while True:
                try:
                    value = simpledialog.askstring("Interactive Input", prompt)
                    if value is None:  # User cancelled
                        return default_value if default_value is not None else 0.0
                    if value == "" and default_value is not None:
                        return default_value
                    return float(value)
                except ValueError:
                    messagebox.showerror("Error", "Please enter a valid number.")
        elif field_type == "str":
            value = simpledialog.askstring("Interactive Input", prompt)
            if value is None or value == "":
                return default_value if default_value is not None else ""
            return value
        
    def get_input_values_interactive(self):
        """Get all input values through interactive prompts"""
        try:
            # Object state
            self.obj_data.state.x = self.get_interactive_input("pos_x", "Object Position X (m)", "float", 25.0)
            self.obj_data.state.y = self.get_interactive_input("pos_y", "Object Position Y (m)", "float", 1.5)
            self.obj_data.state.vx = self.get_interactive_input("vel_x", "Object Velocity X (m/s)", "float", 0.5)
            self.obj_data.state.vy = self.get_interactive_input("vel_y", "Object Velocity Y (m/s)", "float", 3.0)
            
            # Object properties
            self.obj_data.is_object_vru = self.get_interactive_input("is_vru", "Is Object VRU?", "bool", True)
            self.obj_data.rcs = self.get_interactive_input("rcs", "RCS (dBm²)", "float", -8.0)
            self.obj_data.num_cycles_existing = self.get_interactive_input("num_cycles", "Number of Cycles Existing", "int", 8)
            self.obj_data.filter_type = self.get_interactive_input("filter_type", "Filter Type (LA/WNJ/KF)", "str", "LA")
            
            # Probabilities
            self.obj_data.prob_has_been_observed_moving = self.get_interactive_input("prob_has_been_moving", "Probability Has Been Observed Moving (0-1)", "float", 0.3)
            self.obj_data.prob_is_currently_moving = self.get_interactive_input("prob_currently_moving", "Probability Is Currently Moving (0-1)", "float", 0.2)
            
            # Elevation
            self.obj_data.elevation = self.get_interactive_input("elevation", "Elevation (m)", "float", 0.0)
            self.obj_data.elevation_is_valid = self.get_interactive_input("elevation_valid", "Is Elevation Valid?", "bool", True)
            
            # Innovation
            self.obj_data.avg_dx_innovation = self.get_interactive_input("avg_dx_innovation", "Average DX Innovation", "float", 0.5)
            self.obj_data.radar_based_innovation[0] = self.get_interactive_input("radar_innovation_dr", "Radar Innovation DR", "float", 0.0)
            self.obj_data.radar_based_innovation[1] = self.get_interactive_input("radar_innovation_alpha", "Radar Innovation Alpha", "float", 0.0)
            self.obj_data.video_based_innovation[0] = self.get_interactive_input("video_innovation_dr", "Video Innovation DR", "float", 0.0)
            self.obj_data.video_based_innovation[1] = self.get_interactive_input("video_innovation_alpha", "Video Innovation Alpha", "float", 0.0)
            
            # Micro doppler
            self.obj_data.number_micro_doppler_cycles = self.get_interactive_input("micro_doppler_cycles", "Number of Micro Doppler Cycles", "int", 0)
            self.obj_data.expected_vr_high_enough_for_mu_doppler_counter = self.get_interactive_input("expected_vr_counter", "Expected VR High Enough Counter", "int", 5)
            
            # Sensor data
            self.obj_data.sensor_filter_fus_helper.total_num_radar_updates = self.get_interactive_input("total_radar_updates", "Total Number of Radar Updates", "int", 5)
            self.obj_data.sensor_filter_fus_helper.total_num_video_updates = self.get_interactive_input("total_video_updates", "Total Number of Video Updates", "int", 3)
            self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates = self.get_interactive_input("fc_location_radar_updates", "Front Center Location Radar Updates", "int", 5)
            self.obj_data.sensor_filter_fus_helper.total_num_front_left_corner_updates = self.get_interactive_input("fl_corner_updates", "Front Left Corner Updates", "int", 0)
            self.obj_data.sensor_filter_fus_helper.total_num_front_right_corner_updates = self.get_interactive_input("fr_corner_updates", "Front Right Corner Updates", "int", 0)
            self.obj_data.sensor_filter_fus_helper.updates_since_last_video_update = self.get_interactive_input("since_last_video", "Updates Since Last Video", "int", 2)
            self.obj_data.sensor_filter_fus_helper.updates_since_last_radar_update = self.get_interactive_input("since_last_radar", "Updates Since Last Radar", "int", 0)
            self.obj_data.sensor_filter_fus_helper.is_good_quality_fused_object = self.get_interactive_input("good_quality_fused", "Is Good Quality Fused Object?", "bool", True)
            self.obj_data.sensor_filter_fus_helper.is_trustworthy_object = self.get_interactive_input("trustworthy_object", "Is Trustworthy Object?", "bool", True)
            
            # Ego data
            self.ego_data.velocity_x = self.get_interactive_input("velocity_x", "Ego Velocity X (m/s)", "float", 13.9)
            self.ego_data.acceleration_y = self.get_interactive_input("acceleration_y", "Ego Acceleration Y (m/s²)", "float", 0.0)
            self.ego_data.yaw_rate = self.get_interactive_input("yaw_rate", "Ego Yaw Rate (rad/s)", "float", 0.0)
            
            # Absolute velocity over ground
            self.abs_vel_over_ground[0] = self.get_interactive_input("abs_vel_x", "Absolute Velocity X (m/s)", "float", 0.5)
            self.abs_vel_over_ground[1] = self.get_interactive_input("abs_vel_y", "Absolute Velocity Y (m/s)", "float", 3.0)
            
            # Additional flags
            self.is_mpc3_used = self.get_interactive_input("is_mpc3_used", "Is MPC3 Used?", "bool", False)
            
            # Set some default values for fields not commonly used
            self.obj_data.total_num_cycles_with_oncoming_locations = 0
            self.obj_data.vy_unreliable_accumulated = 0.0
            self.obj_data.video_inv_ttc = 0.0
            self.obj_data.is_suppressed_until_next_video_update = False
            self.obj_data.is_suppressed_due_to_video_otc_post_processing = False
            self.obj_data.is_updated_with_stat_loc_with_high_mdoppler_with_outgoing_vr = False
            
            return True
        except Exception as e:
            messagebox.showerror("Input Error", f"Error getting interactive input: {e}")
            return False
        
    def get_input_values(self):
        """Extract all input values from GUI or interactive input"""
        if self.interactive_mode.get():
            return self.get_input_values_interactive()
        else:
            return self.get_input_values_from_gui()
    
    def get_input_values_from_gui(self):
        """Extract all input values from GUI"""
        try:
            # Object state
            self.obj_data.state.x = float(self.obj_entries['pos_x'].get())
            self.obj_data.state.y = float(self.obj_entries['pos_y'].get())
            self.obj_data.state.vx = float(self.obj_entries['vel_x'].get())
            self.obj_data.state.vy = float(self.obj_entries['vel_y'].get())
            
            # Object properties
            self.obj_data.is_object_vru = self.obj_entries['is_vru'].get()
            self.obj_data.rcs = float(self.obj_entries['rcs'].get())
            self.obj_data.num_cycles_existing = int(self.obj_entries['num_cycles'].get())
            self.obj_data.filter_type = self.obj_entries['filter_type'].get()
            
            # Probabilities
            self.obj_data.prob_has_been_observed_moving = float(self.obj_entries['prob_has_been_moving'].get())
            self.obj_data.prob_is_currently_moving = float(self.obj_entries['prob_currently_moving'].get())
            
            # Elevation
            self.obj_data.elevation = float(self.obj_entries['elevation'].get())
            self.obj_data.elevation_is_valid = self.obj_entries['elevation_valid'].get()
            
            # Innovation
            self.obj_data.avg_dx_innovation = float(self.obj_entries['avg_dx_innovation'].get())
            self.obj_data.radar_based_innovation[0] = float(self.obj_entries['radar_innovation_dr'].get())
            self.obj_data.radar_based_innovation[1] = float(self.obj_entries['radar_innovation_alpha'].get())
            self.obj_data.video_based_innovation[0] = float(self.obj_entries['video_innovation_dr'].get())
            self.obj_data.video_based_innovation[1] = float(self.obj_entries['video_innovation_alpha'].get())
            
            # Micro doppler
            self.obj_data.number_micro_doppler_cycles = int(self.obj_entries['micro_doppler_cycles'].get())
            self.obj_data.expected_vr_high_enough_for_mu_doppler_counter = int(self.obj_entries['expected_vr_counter'].get())
            
            # Sensor data
            self.obj_data.sensor_filter_fus_helper.total_num_radar_updates = int(self.sensor_entries['total_radar_updates'].get())
            self.obj_data.sensor_filter_fus_helper.total_num_video_updates = int(self.sensor_entries['total_video_updates'].get())
            self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates = int(self.sensor_entries['fc_location_radar_updates'].get())
            self.obj_data.sensor_filter_fus_helper.total_num_front_left_corner_updates = int(self.sensor_entries['fl_corner_updates'].get())
            self.obj_data.sensor_filter_fus_helper.total_num_front_right_corner_updates = int(self.sensor_entries['fr_corner_updates'].get())
            self.obj_data.sensor_filter_fus_helper.updates_since_last_video_update = int(self.sensor_entries['since_last_video'].get())
            self.obj_data.sensor_filter_fus_helper.updates_since_last_radar_update = int(self.sensor_entries['since_last_radar'].get())
            self.obj_data.sensor_filter_fus_helper.is_good_quality_fused_object = self.sensor_entries['good_quality_fused'].get()
            self.obj_data.sensor_filter_fus_helper.is_trustworthy_object = self.sensor_entries['trustworthy_object'].get()
            
            # Ego data
            self.ego_data.velocity_x = float(self.ego_entries['velocity_x'].get())
            self.ego_data.acceleration_y = float(self.ego_entries['acceleration_y'].get())
            self.ego_data.yaw_rate = float(self.ego_entries['yaw_rate'].get())
            
            # Absolute velocity over ground
            self.abs_vel_over_ground[0] = float(self.ego_entries['abs_vel_x'].get())
            self.abs_vel_over_ground[1] = float(self.ego_entries['abs_vel_y'].get())
            
            # Additional flags
            self.is_mpc3_used = self.ego_entries['is_mpc3_used'].get()
            
            # Set some default values for fields not exposed in GUI
            self.obj_data.total_num_cycles_with_oncoming_locations = 0
            self.obj_data.vy_unreliable_accumulated = 0.0
            self.obj_data.video_inv_ttc = 0.0
            self.obj_data.is_suppressed_until_next_video_update = False
            self.obj_data.is_suppressed_due_to_video_otc_post_processing = False
            self.obj_data.is_updated_with_stat_loc_with_high_mdoppler_with_outgoing_vr = False
            
            return True
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid input value: {e}")
            return False
            
    def is_negative(self, value):
        """Helper function to check if value is negative"""
        return value < 0.0
        
    def is_moving_towards_ego_lane(self, dy_obj, vy_obj_rel, vy_obj_over_ground):
        """Function 3: isMovingTowardsEgoLane"""
        return self.is_negative(dy_obj * vy_obj_rel) or self.is_negative(dy_obj * vy_obj_over_ground)
        
    def is_dep_obj_probably_video_ghost(self):
        """Function 4: isDepObjProbablyVideoGhost"""
        # Complex logic for video ghost detection
        is_initial_radar_update_phase = (
            self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates > 0 and
            self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates < 3 and
            self.obj_data.sensor_filter_fus_helper.updates_since_last_radar_update == 0
        )
        
        is_tracked_by_video = self.obj_data.sensor_filter_fus_helper.total_num_video_updates > 3
        
        has_not_been_updated_by_corner_radar = (
            self.obj_data.sensor_filter_fus_helper.total_num_front_left_corner_updates == 0 and
            self.obj_data.sensor_filter_fus_helper.total_num_front_right_corner_updates == 0
        )
        
        is_almost_video_only = is_tracked_by_video and has_not_been_updated_by_corner_radar and is_initial_radar_update_phase
        
        has_no_micro_doppler = (
            self.obj_data.number_micro_doppler_cycles == 0 and
            self.obj_data.expected_vr_high_enough_for_mu_doppler_counter > 0
        )
        
        is_very_low_rcs = self.obj_data.rcs < -15.0
        
        return is_almost_video_only and has_no_micro_doppler and is_very_low_rcs
        
    def evaluate_functions(self):
        """Evaluate all functions and display results"""
        if not self.get_input_values():
            return
            
        self.clear_results()
        results = []
        
        # Display input mode
        input_mode = "Interactive Input Mode" if self.interactive_mode.get() else "GUI Input Mode"
        self.results_text.insert(tk.END, f"=== AUTOMOTIVE PERCEPTION FUNCTION EVALUATION RESULTS ({input_mode}) ===\n\n")
        
        # Pre-calculate common values
        self.dep_obj_probably_video_ghost = self.is_dep_obj_probably_video_ghost()
        
        # Function 1: applySuppressionUntilNextVideoUpdateCheck
        if self.obj_data.is_suppressed_until_next_video_update:
            results.append("✓ applySuppressionUntilNextVideoUpdateCheck - Object is suppressed until next video update")
        else:
            results.append("✗ applySuppressionUntilNextVideoUpdateCheck - Object is not suppressed")
            
        # Function 2: applyPostProcessVideoOtcCheck
        if self.obj_data.is_suppressed_due_to_video_otc_post_processing:
            results.append("✓ applyPostProcessVideoOtcCheck - Object is suppressed due to video OTC post processing")
        else:
            results.append("✗ applyPostProcessVideoOtcCheck - Object is not suppressed due to video OTC")
            
        # Function 3: isMovingTowardsEgoLane
        if self.is_moving_towards_ego_lane(self.obj_data.state.y, self.obj_data.state.vy, self.abs_vel_over_ground[1]):
            results.append("✓ isMovingTowardsEgoLane - Object is moving towards ego lane")
        else:
            results.append("✗ isMovingTowardsEgoLane - Object is not moving towards ego lane")
            
        # Function 4: isDepObjProbablyVideoGhost
        if self.dep_obj_probably_video_ghost:
            results.append("✓ isDepObjProbablyVideoGhost - Object is probably a video ghost")
        else:
            results.append("✗ isDepObjProbablyVideoGhost - Object is probably not a video ghost")
            
        # Function 6: applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck
        condition_6 = (
            self.obj_data.is_object_vru and
            self.abs_vel_over_ground[0] < 0.2 and
            self.abs_vel_over_ground[1] > 1.0 and
            abs(self.obj_data.state.y) < 0.5 and
            self.obj_data.is_updated_with_stat_loc_with_high_mdoppler_with_outgoing_vr
        )
        if condition_6:
            results.append("✓ applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck - VRU with stat loc and high micro doppler")
        else:
            results.append("✗ applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck - Conditions not met")
            
        # Function 7: applyIsMeasuredRatioCheckForFastWnj
        if (self.obj_data.filter_type == "WNJ" and 
            abs(self.abs_vel_over_ground[1]) > 4.6 and 
            self.obj_data.num_cycles_existing < 255):
            
            if self.obj_data.num_cycles_existing > 1:
                ratio = self.obj_data.sensor_filter_fus_helper.total_num_radar_updates / (self.obj_data.num_cycles_existing + 1)
                if (ratio < 0.7 and self.obj_data.sensor_filter_fus_helper.total_num_video_updates <= 5):
                    results.append("✓ applyIsMeasuredRatioCheckForFastWnj - Fast WNJ with insufficient measurement ratio")
                else:
                    results.append("✗ applyIsMeasuredRatioCheckForFastWnj - Measurement ratio is sufficient")
            else:
                results.append("✗ applyIsMeasuredRatioCheckForFastWnj - Object too young")
        else:
            results.append("✗ applyIsMeasuredRatioCheckForFastWnj - Not a fast crossing WNJ object")
            
        # Function 9: applyNonCrossingObjectCheck
        appears_crossing = self.abs_vel_over_ground[1] > 0.5
        is_prob_moving_low = (self.obj_data.prob_is_currently_moving < 0.1 and 
                             self.obj_data.prob_has_been_observed_moving < 0.1)
        has_been_updated_by_radar = self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates > 0
        has_no_micro_doppler = self.obj_data.number_micro_doppler_cycles == 0
        has_no_oncoming_locations = self.obj_data.total_num_cycles_with_oncoming_locations == 0
        is_not_perceived_as_moving_by_radar = has_been_updated_by_radar and has_no_micro_doppler and has_no_oncoming_locations
        
        appears_crossing_but_probably_not = appears_crossing and is_prob_moving_low and is_not_perceived_as_moving_by_radar
        
        if appears_crossing_but_probably_not:
            results.append("✓ applyNonCrossingObjectCheck - Object appears crossing but is probably not moving")
        else:
            results.append("✗ applyNonCrossingObjectCheck - Object crossing behavior is consistent")
            
        # Function 12: applyMicroDopplerCheck
        if (self.params.is_micro_doppler_check_enabled and 
            self.obj_data.is_object_vru and
            self.obj_data.expected_vr_high_enough_for_mu_doppler_counter >= 2 and
            self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates > 0 and
            self.obj_data.sensor_filter_fus_helper.total_num_front_left_corner_updates < 1 and
            self.obj_data.sensor_filter_fus_helper.total_num_front_right_corner_updates < 1 and
            self.obj_data.number_micro_doppler_cycles < self.params.min_vru_micro_doppler_cycles):
            
            # Check for crossing VRU conditions
            object_age_threshold = 12
            is_object_old = (self.obj_data.num_cycles_existing > object_age_threshold and
                           self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates > 8)
            upper_abs_vy_threshold = 3.2 if is_object_old else 99.0
            
            is_crossing_vru = (self.abs_vel_over_ground[1] > 0.5 and
                             self.abs_vel_over_ground[1] < upper_abs_vy_threshold and
                             self.abs_vel_over_ground[0] < 4.0)
            
            are_crossing_vru_conditions_satisfied = is_crossing_vru and self.params.is_micro_doppler_check_on_crossing_vru_applied
            
            # Check for stationary VRU conditions
            is_stationary_vru = (self.abs_vel_over_ground[0] < 0.5 and self.abs_vel_over_ground[1] < 0.5)
            are_stationary_vru_conditions_satisfied = is_stationary_vru and self.params.is_micro_doppler_check_on_stationary_vru_applied
            
            if are_crossing_vru_conditions_satisfied or are_stationary_vru_conditions_satisfied:
                results.append("✓ applyMicroDopplerCheck - VRU missing expected micro-doppler signatures")
            else:
                results.append("✗ applyMicroDopplerCheck - VRU conditions not met for micro-doppler check")
        else:
            results.append("✗ applyMicroDopplerCheck - Micro-doppler check conditions not met")
            
        # Function 13: applyRadarOnlyRcsAndDrInnovationLimit
        is_front_center_radar_only = (self.obj_data.sensor_filter_fus_helper.total_num_video_updates == 0 and
                                     self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates > 0)
        is_dr_innovation_exceeded = abs(self.obj_data.avg_dx_innovation) > 1.2
        is_rcs_too_low = self.obj_data.rcs < -15.0
        
        if is_front_center_radar_only and is_dr_innovation_exceeded and is_rcs_too_low:
            results.append("✓ applyRadarOnlyRcsAndDrInnovationLimit - Radar-only object with high innovation and low RCS")
        else:
            results.append("✗ applyRadarOnlyRcsAndDrInnovationLimit - Conditions not met")
            
        # Function 14: applyElevationCheck
        obj_dx = self.obj_data.state.x
        is_stationary_velocity_threshold = 1.0
        is_object_stationary = (self.abs_vel_over_ground[0] < is_stationary_velocity_threshold and
                              self.abs_vel_over_ground[1] < is_stationary_velocity_threshold)
        has_been_updated_by_video_recently = self.obj_data.sensor_filter_fus_helper.updates_since_last_video_update < 10
        is_stationary_video_confirmed_object = is_object_stationary and has_been_updated_by_video_recently
        
        if (obj_dx > 0 and self.obj_data.elevation_is_valid and 
            (is_stationary_video_confirmed_object or self.obj_data.is_object_vru)):
            # Simple linear interpolation for allowed elevation threshold
            allowed_dz_threshold = 2.0 + (obj_dx / 100.0) * (3.0 - 2.0)  # Linear interpolation
            is_dz_inappropriate = self.obj_data.elevation > allowed_dz_threshold
            
            if is_dz_inappropriate:
                results.append("✓ applyElevationCheck - Object elevation is inappropriate (too high)")
            else:
                results.append("✗ applyElevationCheck - Object elevation is appropriate")
        else:
            results.append("✗ applyElevationCheck - Elevation check preconditions not met")
            
        # Function 21: applyInnovationCheck
        innovation_relevant = (abs(self.obj_data.state.x) < self.params.innovation_check_dx_threshold and
                             abs(self.obj_data.state.y) < self.params.innovation_check_dy_threshold)
        
        # Simplified dx innovation threshold calculation
        dx_innovation_threshold = 1.6  # Default
        if abs(self.obj_data.state.x) < 20.0 and self.obj_data.is_object_vru:
            dx_innovation_threshold = 1.5
        elif self.obj_data.rcs < -5.0 and self.obj_data.vy_unreliable_accumulated > 1.9:
            dx_innovation_threshold = 1.1
        elif self.obj_data.rcs < -15.0:
            dx_innovation_threshold = 1.5
        
        abs_avg_innovation_dx = abs(self.obj_data.avg_dx_innovation)
        
        if innovation_relevant and abs_avg_innovation_dx > dx_innovation_threshold:
            results.append("✓ applyInnovationCheck - Object has high dx innovation in relevant range")
        else:
            results.append("✗ applyInnovationCheck - Innovation is within acceptable limits")
            
        # Function 23: applyImplausibleVyVruCheck
        turning_ego_yaw_rate_threshold = 10.0 * (math.pi / 180.0)  # 10 degrees in radians
        is_ego_turning = abs(self.ego_data.yaw_rate) > turning_ego_yaw_rate_threshold
        
        if (self.obj_data.is_object_vru and 
            self.abs_vel_over_ground[1] > self.params.implausible_vy_thresh_la_hypo and
            self.obj_data.filter_type == "LA" and 
            not is_ego_turning):
            results.append("✓ applyImplausibleVyVruCheck - VRU with implausible VY velocity")
        else:
            results.append("✗ applyImplausibleVyVruCheck - VRU VY velocity is plausible")
            
        # Function 25: applyImplausibleVideoTtcForVru
        if (self.obj_data.is_object_vru and
            self.obj_data.sensor_filter_fus_helper.updates_since_last_video_update < 1 and
            self.obj_data.video_inv_ttc == float('inf')):  # Representing max float
            results.append("✓ applyImplausibleVideoTtcForVru - VRU with implausible video TTC")
        else:
            results.append("✗ applyImplausibleVideoTtcForVru - Video TTC is plausible")
            
        # Function 29: applyRadarOnlyNLDCheck
        if self.obj_data.sensor_filter_fus_helper.total_num_video_updates == 0:
            # Ego driving straight check
            acceleration_threshold = 0.15
            angle_dt_threshold = 0.012
            ego_driving_straight_radius = 2500.0
            
            is_ego_driving_straight = True
            if self.ego_data.yaw_rate != 0:
                ego_radius = self.ego_data.velocity_x / self.ego_data.yaw_rate
                is_ego_driving_straight = (abs(ego_radius) > ego_driving_straight_radius or
                                         (abs(self.ego_data.acceleration_y) < acceleration_threshold and 
                                          self.ego_data.yaw_rate < angle_dt_threshold))
            
            # Object in relevant area check
            is_object_in_relevant_area = ((abs(self.obj_data.state.y) <= 1.25 and self.obj_data.state.x < 120.0) or
                                        (abs(self.obj_data.state.y) <= 6.0 and self.obj_data.state.x < 10.0))
            
            # Object age check
            is_object_old_enough = self.obj_data.num_cycles_existing >= 3
            
            # Object measurement check
            is_object_measured_sufficiently = (self.obj_data.sensor_filter_fus_helper.total_num_radar_updates >= 
                                             self.obj_data.num_cycles_existing or 
                                             self.obj_data.num_cycles_existing >= 30)
            
            is_radar_only_nld_candidate = (not is_ego_driving_straight or 
                                         not is_object_in_relevant_area or 
                                         not is_object_old_enough or 
                                         not is_object_measured_sufficiently)
            
            # High lateral velocity check
            is_object_close_with_high_lateral_velocity = (abs(self.obj_data.state.vy) > 3.0 and
                                                        self.obj_data.state.x < 8.0 and
                                                        abs(self.obj_data.state.y) < 4.0)
            
            if is_radar_only_nld_candidate or is_object_close_with_high_lateral_velocity:
                results.append("✓ applyRadarOnlyNLDCheck - Radar-only object is NLD candidate")
            else:
                results.append("✗ applyRadarOnlyNLDCheck - Radar-only object is not NLD candidate")
        else:
            results.append("✗ applyRadarOnlyNLDCheck - Object is not radar-only")
            
        # Function 30: applyRadarOnlyStationaryCheck
        if self.obj_data.sensor_filter_fus_helper.total_num_video_updates == 0:
            if self.abs_vel_over_ground[0] < 0.3 and self.abs_vel_over_ground[1] < 0.3:
                results.append("✓ applyRadarOnlyStationaryCheck - Radar-only stationary object")
            else:
                results.append("✗ applyRadarOnlyStationaryCheck - Radar-only object is not stationary")
        else:
            results.append("✗ applyRadarOnlyStationaryCheck - Object is not radar-only")
        
        # Display results
        self.results_text.insert(tk.END, f"Object Type: {'VRU' if self.obj_data.is_object_vru else 'Non-VRU'}\n")
        self.results_text.insert(tk.END, f"Position: ({self.obj_data.state.x:.2f}, {self.obj_data.state.y:.2f}) m\n")
        self.results_text.insert(tk.END, f"Velocity: ({self.obj_data.state.vx:.2f}, {self.obj_data.state.vy:.2f}) m/s\n")
        self.results_text.insert(tk.END, f"Abs Vel Over Ground: ({self.abs_vel_over_ground[0]:.2f}, {self.abs_vel_over_ground[1]:.2f}) m/s\n")
        self.results_text.insert(tk.END, f"RCS: {self.obj_data.rcs:.2f} dBm²\n")
        self.results_text.insert(tk.END, f"Age: {self.obj_data.num_cycles_existing} cycles\n\n")
        
        active_functions = [r for r in results if r.startswith("✓")]
        inactive_functions = [r for r in results if r.startswith("✗")]
        
        self.results_text.insert(tk.END, f"ACTIVE FUNCTIONS ({len(active_functions)}):\n")
        self.results_text.insert(tk.END, "=" * 50 + "\n")
        for result in active_functions:
            self.results_text.insert(tk.END, result + "\n")
            
        self.results_text.insert(tk.END, f"\nINACTIVE FUNCTIONS ({len(inactive_functions)}):\n")
        self.results_text.insert(tk.END, "=" * 50 + "\n")
        for result in inactive_functions:
            self.results_text.insert(tk.END, result + "\n")
            
        self.results_text.insert(tk.END, f"\nSUMMARY: {len(active_functions)} out of {len(results)} functions would execute their main logic.\n")
        
    def load_example(self):
        """Load a predefined example scenario"""
        # Fast crossing pedestrian example
        self.obj_entries['pos_x'].delete(0, tk.END)
        self.obj_entries['pos_x'].insert(0, "25.0")
        self.obj_entries['pos_y'].delete(0, tk.END)
        self.obj_entries['pos_y'].insert(0, "1.5")
        self.obj_entries['vel_x'].delete(0, tk.END)
        self.obj_entries['vel_x'].insert(0, "0.5")
        self.obj_entries['vel_y'].delete(0, tk.END)
        self.obj_entries['vel_y'].insert(0, "3.0")
        
        self.obj_entries['is_vru'].set(True)
        self.obj_entries['rcs'].delete(0, tk.END)
        self.obj_entries['rcs'].insert(0, "-8.0")
        self.obj_entries['num_cycles'].delete(0, tk.END)
        self.obj_entries['num_cycles'].insert(0, "8")
        
        self.obj_entries['prob_has_been_moving'].delete(0, tk.END)
        self.obj_entries['prob_has_been_moving'].insert(0, "0.3")
        self.obj_entries['prob_currently_moving'].delete(0, tk.END)
        self.obj_entries['prob_currently_moving'].insert(0, "0.2")
        
        self.obj_entries['micro_doppler_cycles'].delete(0, tk.END)
        self.obj_entries['micro_doppler_cycles'].insert(0, "0")
        self.obj_entries['expected_vr_counter'].delete(0, tk.END)
        self.obj_entries['expected_vr_counter'].insert(0, "5")
        
        self.ego_entries['abs_vel_x'].delete(0, tk.END)
        self.ego_entries['abs_vel_x'].insert(0, "0.5")
        self.ego_entries['abs_vel_y'].delete(0, tk.END)
        self.ego_entries['abs_vel_y'].insert(0, "3.0")
        
        messagebox.showinfo("Example Loaded", "Fast crossing pedestrian example loaded!")
        
    def save_config(self):
        """Save current configuration to file"""
        if not self.get_input_values():
            return
            
        config = {
            'object_data': {
                'state': {
                    'x': self.obj_data.state.x,
                    'y': self.obj_data.state.y,
                    'vx': self.obj_data.state.vx,
                    'vy': self.obj_data.state.vy
                },
                'is_object_vru': self.obj_data.is_object_vru,
                'rcs': self.obj_data.rcs,
                'num_cycles_existing': self.obj_data.num_cycles_existing,
                'filter_type': self.obj_data.filter_type,
                'prob_has_been_observed_moving': self.obj_data.prob_has_been_observed_moving,
                'prob_is_currently_moving': self.obj_data.prob_is_currently_moving,
                'elevation': self.obj_data.elevation,
                'elevation_is_valid': self.obj_data.elevation_is_valid,
                'avg_dx_innovation': self.obj_data.avg_dx_innovation,
                'number_micro_doppler_cycles': self.obj_data.number_micro_doppler_cycles,
                'expected_vr_high_enough_for_mu_doppler_counter': self.obj_data.expected_vr_high_enough_for_mu_doppler_counter
            },
            'ego_data': {
                'velocity_x': self.ego_data.velocity_x,
                'acceleration_y': self.ego_data.acceleration_y,
                'yaw_rate': self.ego_data.yaw_rate
            },
            'abs_vel_over_ground': self.abs_vel_over_ground,
            'is_mpc3_used': self.is_mpc3_used
        }
        
        try:
            with open('perception_config.json', 'w') as f:
                json.dump(config, f, indent=2)
            messagebox.showinfo("Success", "Configuration saved to perception_config.json")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save configuration: {e}")
            
    def load_config(self):
        """Load configuration from file"""
        try:
            with open('perception_config.json', 'r') as f:
                config = json.load(f)
            
            # Load object data
            obj_data = config['object_data']
            self.obj_entries['pos_x'].delete(0, tk.END)
            self.obj_entries['pos_x'].insert(0, str(obj_data['state']['x']))
            self.obj_entries['pos_y'].delete(0, tk.END)
            self.obj_entries['pos_y'].insert(0, str(obj_data['state']['y']))
            self.obj_entries['vel_x'].delete(0, tk.END)
            self.obj_entries['vel_x'].insert(0, str(obj_data['state']['vx']))
            self.obj_entries['vel_y'].delete(0, tk.END)
            self.obj_entries['vel_y'].insert(0, str(obj_data['state']['vy']))
            
            self.obj_entries['is_vru'].set(obj_data['is_object_vru'])
            self.obj_entries['rcs'].delete(0, tk.END)
            self.obj_entries['rcs'].insert(0, str(obj_data['rcs']))
            # ... load other fields
            
            messagebox.showinfo("Success", "Configuration loaded from perception_config.json")
        except FileNotFoundError:
            messagebox.showerror("Error", "Configuration file not found")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load configuration: {e}")
            
    def clear_results(self):
        """Clear the results display"""
        self.results_text.delete(1.0, tk.END)
        
    def run(self):
        """Run the application"""
        self.root.mainloop()

if __name__ == "__main__":
    app = AutomotivePerceptionEmulator()
    app.run()