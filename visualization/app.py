import sys
import os
import numpy as np

# Ensure we can import from the root directory when running this script directly
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QGroupBox, QGridLayout, 
                             QPushButton, QRadioButton, QButtonGroup, QDoubleSpinBox,
                             QScrollArea)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.kinematics.JointLimits import clamp_joints, validate_joints, get_limits, JOINT_NAMES
from visualization.kinematics.Jacobian import get_jacobian
from visualization.kinematics.GripperSim import GripperSim
from visualization.task2b_cube_stacking import generate_pick_rotate_sequence, CUBE_SIZE_MM

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Live Simulation (Task Space Control)")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize Python Kinematics
        print("Using Python Kinematics (No MATLAB)")
        
        self.HOME_POSE = np.array([200, 0, 100, 0]) # X, Y, Z, Pitch
        self.safety_bypass = False
        self.gripper = GripperSim()

        # Central Widget and Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # 1. Left Panel: Controls (scrollable)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFixedWidth(320)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        scroll_area.setWidget(control_panel)
        main_layout.addWidget(scroll_area)

        # Task Space Sliders Group
        self.sliders = {}
        self.labels = {}
        
        task_group = QGroupBox("Task Space Coordinates (User Frame)")
        task_layout = QGridLayout()
        task_group.setLayout(task_layout)
        
        # Define ranges and defaults (Internal Coordinates: X-Fwd, Y-Left, Z-Up)
        # Pitch: -90 (Down) to +90 (Up)
        controls = [
            ('X', 0, 450, 200),     # Forward range (0 to Max Reach)
            ('Y', -450, 450, 0),    # Left/Right range (± Max Reach)
            ('Z', 0, 450, 100),     # Up/Down range (0 to Max Height)
            ('Pitch', -90, 90, 0)   # Pitch angle
        ]
        
        for i, (name, min_val, max_val, default) in enumerate(controls):
            label = QLabel(f"{name}: {default}")
            self.labels[name] = label
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setValue(default)
            slider.valueChanged.connect(self.update_robot)
            self.sliders[name] = slider
            
            task_layout.addWidget(QLabel(name), i, 0)
            task_layout.addWidget(slider, i, 1)
            task_layout.addWidget(label, i, 2)
            
        control_layout.addWidget(task_group)
        
        # View Controls Group
        view_group = QGroupBox("View Controls")
        view_layout = QGridLayout()
        view_group.setLayout(view_layout)
        
        btn_iso = QPushButton("ISO View")
        btn_iso.clicked.connect(lambda: self.plotter.view_isometric())
        
        btn_top = QPushButton("Top View")
        btn_top.clicked.connect(lambda: self.plotter.view_xy())
        
        btn_front = QPushButton("Front View")
        btn_front.clicked.connect(lambda: self.plotter.view_yz())
        
        btn_side = QPushButton("Side View")
        btn_side.clicked.connect(lambda: self.plotter.view_xz())
        
        view_layout.addWidget(btn_iso, 0, 0)
        view_layout.addWidget(btn_top, 0, 1)
        view_layout.addWidget(btn_front, 1, 0)
        view_layout.addWidget(btn_side, 1, 1)
        
        control_layout.addWidget(view_group)
        
        # Joint Limits Status Group
        limits_group = QGroupBox("Joint Status")
        limits_layout = QVBoxLayout()
        limits_group.setLayout(limits_layout)
        
        self.joint_labels = []
        limits = get_limits()
        for i in range(4):
            lbl = QLabel(f"J{i+1} ({JOINT_NAMES[i]}): -- [{limits['min'][i]:.0f}°, {limits['max'][i]:.0f}°]")
            self.joint_labels.append(lbl)
            limits_layout.addWidget(lbl)
        
        self.limit_status = QLabel("✓ All joints within limits")
        self.limit_status.setStyleSheet("color: green; font-weight: bold;")
        limits_layout.addWidget(self.limit_status)
        
        control_layout.addWidget(limits_group)
        
        # Motion Control Group
        motion_group = QGroupBox("Motion Control")
        motion_layout = QVBoxLayout()
        motion_group.setLayout(motion_layout)
        
        # Targets
        target_layout = QGridLayout()
        self.target_inputs = {}
        target_labels = ['Target X', 'Target Y', 'Target Z', 'Target Pitch']
        target_keys = ['x', 'y', 'z', 'pitch']
        defaults = [200, 0, 100, 0]
        
        for i, key in enumerate(target_keys):
            lbl = QLabel(target_labels[i])
            spin = QDoubleSpinBox()
            spin.setRange(-500, 500)
            spin.setValue(defaults[i])
            self.target_inputs[key] = spin
            target_layout.addWidget(lbl, i, 0)
            target_layout.addWidget(spin, i, 1)
            
        motion_layout.addLayout(target_layout)
        
        # Velocity Control
        vel_layout = QHBoxLayout()
        vel_layout.addWidget(QLabel("Velocity (mm/s):"))
        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(1, 200)
        self.vel_spin.setValue(40)
        vel_layout.addWidget(self.vel_spin)
        motion_layout.addLayout(vel_layout)
        
        # Mode Selection
        self.mode_group = QButtonGroup(self)
        self.radio_joint = QRadioButton("Joint Interpolation")
        self.radio_task = QRadioButton("Task Interpolation (Linear)")
        self.radio_jac = QRadioButton("Jacobian Control (Velocity)")
        self.radio_joint.setChecked(True)
        
        self.mode_group.addButton(self.radio_joint, 0)
        self.mode_group.addButton(self.radio_task, 1)
        self.mode_group.addButton(self.radio_jac, 2)
        
        motion_layout.addWidget(self.radio_joint)
        motion_layout.addWidget(self.radio_task)
        motion_layout.addWidget(self.radio_jac)
        
        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_move = QPushButton("Move to Target")
        self.btn_move.clicked.connect(self.start_motion)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.clicked.connect(self.stop_motion)
        self.btn_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        
        btn_layout.addWidget(self.btn_move)
        btn_layout.addWidget(self.btn_stop)
        motion_layout.addLayout(btn_layout)
        
        # Status Label
        self.motion_status = QLabel("Ready")
        motion_layout.addWidget(self.motion_status)
        
        control_layout.addWidget(motion_group)
        
        # Camera Control Group
        cam_group = QGroupBox("Camera Controls")
        cam_layout = QVBoxLayout()
        cam_group.setLayout(cam_layout)
        
        # Quick Views
        cam_btn_layout = QHBoxLayout()
        self.btn_cam_center = QPushButton("Center")
        self.btn_cam_center.clicked.connect(lambda: self.set_camera_view('center'))
        self.btn_cam_left = QPushButton("Left")
        self.btn_cam_left.clicked.connect(lambda: self.set_camera_view('left'))
        self.btn_cam_right = QPushButton("Right")
        self.btn_cam_right.clicked.connect(lambda: self.set_camera_view('right'))
        
        cam_btn_layout.addWidget(self.btn_cam_left)
        cam_btn_layout.addWidget(self.btn_cam_center)
        cam_btn_layout.addWidget(self.btn_cam_right)
        cam_layout.addLayout(cam_btn_layout)
        
        # Fine Controls
        cam_grid = QGridLayout()
        
        # Azimuth
        cam_grid.addWidget(QLabel("Azimuth:"), 0, 0)
        self.slider_azim = QSlider(Qt.Horizontal)
        self.slider_azim.setRange(-180, 180)
        self.slider_azim.setValue(45)
        self.slider_azim.valueChanged.connect(self.update_camera_azimuth)
        cam_grid.addWidget(self.slider_azim, 0, 1)
        
        # Elevation
        cam_grid.addWidget(QLabel("Elevation:"), 1, 0)
        self.slider_elev = QSlider(Qt.Horizontal)
        self.slider_elev.setRange(0, 90)
        self.slider_elev.setValue(30)
        self.slider_elev.valueChanged.connect(self.update_camera_elevation)
        cam_grid.addWidget(self.slider_elev, 1, 1)
        
        cam_layout.addLayout(cam_grid)
        control_layout.addWidget(cam_group)

        # ── Gripper Control Group ──
        grip_group = QGroupBox("Gripper Control")
        grip_layout = QVBoxLayout()
        grip_group.setLayout(grip_layout)

        # Gripper Slider (0=Open, 100=Closed)
        grip_slider_layout = QHBoxLayout()
        grip_slider_layout.addWidget(QLabel("Grip %:"))
        self.grip_slider = QSlider(Qt.Horizontal)
        self.grip_slider.setRange(0, 100)
        self.grip_slider.setValue(0)
        self.grip_slider.valueChanged.connect(self.update_gripper_from_slider)
        grip_slider_layout.addWidget(self.grip_slider)
        self.grip_slider_label = QLabel("0%")
        grip_slider_layout.addWidget(self.grip_slider_label)
        grip_layout.addLayout(grip_slider_layout)

        # Quick Buttons
        grip_btn_layout = QHBoxLayout()
        btn_open = QPushButton("Open")
        btn_open.clicked.connect(self.gripper_open)
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.gripper_close)
        btn_cube = QPushButton("Cube 25mm")
        btn_cube.clicked.connect(lambda: self.gripper_object(25.0))
        grip_btn_layout.addWidget(btn_open)
        grip_btn_layout.addWidget(btn_close)
        grip_btn_layout.addWidget(btn_cube)
        grip_layout.addLayout(grip_btn_layout)

        # Mode Radio Buttons
        self.grip_mode_group = QButtonGroup(self)
        self.grip_radio_pos = QRadioButton("Position")
        self.grip_radio_obj = QRadioButton("Object-Aware")
        self.grip_radio_force = QRadioButton("Force-Limited")
        self.grip_radio_pos.setChecked(True)
        self.grip_mode_group.addButton(self.grip_radio_pos, 0)
        self.grip_mode_group.addButton(self.grip_radio_obj, 1)
        self.grip_mode_group.addButton(self.grip_radio_force, 2)
        grip_mode_layout = QHBoxLayout()
        grip_mode_layout.addWidget(self.grip_radio_pos)
        grip_mode_layout.addWidget(self.grip_radio_obj)
        grip_mode_layout.addWidget(self.grip_radio_force)
        grip_layout.addLayout(grip_mode_layout)

        # Object Width / Force inputs
        grip_params = QGridLayout()
        grip_params.addWidget(QLabel("Object mm:"), 0, 0)
        self.grip_obj_spin = QDoubleSpinBox()
        self.grip_obj_spin.setRange(0, 50)
        self.grip_obj_spin.setValue(25)
        grip_params.addWidget(self.grip_obj_spin, 0, 1)
        grip_params.addWidget(QLabel("Force %:"), 1, 0)
        self.grip_force_spin = QDoubleSpinBox()
        self.grip_force_spin.setRange(0, 100)
        self.grip_force_spin.setValue(50)
        grip_params.addWidget(self.grip_force_spin, 1, 1)
        btn_apply_mode = QPushButton("Apply Mode")
        btn_apply_mode.clicked.connect(self.apply_grip_mode)
        grip_params.addWidget(btn_apply_mode, 0, 2, 2, 1)
        grip_layout.addLayout(grip_params)

        # Gripper Status
        self.grip_status = QLabel("Open | 0% | Jaw: 40.0mm | Enc: 1365")
        self.grip_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        grip_layout.addWidget(self.grip_status)

        control_layout.addWidget(grip_group)

        # ── Task 2b Control Group ──────────────────────────────────
        t2b_group = QGroupBox("Task 2b: Pick & Rotate Cube")
        t2b_layout = QVBoxLayout()
        t2b_group.setLayout(t2b_layout)

        # Cube pick/place position inputs
        t2b_pos_layout = QGridLayout()
        self.t2b_inputs = {}
        t2b_labels_defaults = [
            ("Pick X:", "pick_x", 200.0),
            ("Pick Y:", "pick_y",   0.0),
            ("Place X:", "place_x", 250.0),
            ("Place Y:", "place_y",  50.0),
        ]
        for row, (label_text, key, default) in enumerate(t2b_labels_defaults):
            lbl = QLabel(label_text)
            spin = QDoubleSpinBox()
            spin.setRange(-500, 500)
            spin.setValue(default)
            self.t2b_inputs[key] = spin
            t2b_pos_layout.addWidget(lbl, row, 0)
            t2b_pos_layout.addWidget(spin, row, 1)
        t2b_layout.addLayout(t2b_pos_layout)

        # Rotation options: no rotate, +90°, -90°
        self.t2b_rot_mode_group = QButtonGroup(self)
        rot_layout = QHBoxLayout()
        self.t2b_rot_none = QRadioButton("No rotate")
        self.t2b_rot_pos = QRadioButton("+90°")
        self.t2b_rot_neg = QRadioButton("−90°")
        self.t2b_rot_pos.setChecked(True)
        self.t2b_rot_mode_group.addButton(self.t2b_rot_none, 0)
        self.t2b_rot_mode_group.addButton(self.t2b_rot_pos, 1)
        self.t2b_rot_mode_group.addButton(self.t2b_rot_neg, -1)
        rot_layout.addWidget(self.t2b_rot_none)
        rot_layout.addWidget(self.t2b_rot_pos)
        rot_layout.addWidget(self.t2b_rot_neg)
        t2b_layout.addLayout(rot_layout)

        # Load / Play / Pause / Reset buttons
        t2b_btn_row1 = QHBoxLayout()
        self.btn_t2b_load = QPushButton("Load Task 2b")
        self.btn_t2b_load.clicked.connect(self.t2b_load)
        self.btn_t2b_load.setStyleSheet("background-color: #4488CC; color: white; font-weight: bold;")
        t2b_btn_row1.addWidget(self.btn_t2b_load)
        t2b_layout.addLayout(t2b_btn_row1)

        t2b_btn_row2 = QHBoxLayout()
        self.btn_t2b_play = QPushButton("▶ Play")
        self.btn_t2b_play.clicked.connect(self.t2b_play)
        self.btn_t2b_play.setEnabled(False)
        self.btn_t2b_pause = QPushButton("⏸ Pause")
        self.btn_t2b_pause.clicked.connect(self.t2b_pause)
        self.btn_t2b_pause.setEnabled(False)
        self.btn_t2b_reset = QPushButton("⏹ Reset")
        self.btn_t2b_reset.clicked.connect(self.t2b_reset)
        self.btn_t2b_reset.setEnabled(False)
        t2b_btn_row2.addWidget(self.btn_t2b_play)
        t2b_btn_row2.addWidget(self.btn_t2b_pause)
        t2b_btn_row2.addWidget(self.btn_t2b_reset)
        t2b_layout.addLayout(t2b_btn_row2)

        # Speed slider
        t2b_speed_layout = QHBoxLayout()
        t2b_speed_layout.addWidget(QLabel("Speed:"))
        self.t2b_speed_slider = QSlider(Qt.Horizontal)
        self.t2b_speed_slider.setRange(1, 20)   # 0.5x to 10x  (value / 2)
        self.t2b_speed_slider.setValue(2)        # 1x
        self.t2b_speed_label = QLabel("1.0×")
        self.t2b_speed_slider.valueChanged.connect(
            lambda v: self.t2b_speed_label.setText(f"{v/2:.1f}×"))
        t2b_speed_layout.addWidget(self.t2b_speed_slider)
        t2b_speed_layout.addWidget(self.t2b_speed_label)
        t2b_layout.addLayout(t2b_speed_layout)

        # Step scrubber
        t2b_scrub_layout = QHBoxLayout()
        t2b_scrub_layout.addWidget(QLabel("Step:"))
        self.t2b_scrubber = QSlider(Qt.Horizontal)
        self.t2b_scrubber.setRange(0, 0)
        self.t2b_scrubber.setEnabled(False)
        self.t2b_scrubber.valueChanged.connect(self.t2b_scrub_to)
        t2b_scrub_layout.addWidget(self.t2b_scrubber)
        self.t2b_step_label = QLabel("0 / 0")
        t2b_scrub_layout.addWidget(self.t2b_step_label)
        t2b_layout.addLayout(t2b_scrub_layout)

        # Status
        self.t2b_status = QLabel("Not loaded")
        self.t2b_status.setStyleSheet("font-weight: bold;")
        t2b_layout.addWidget(self.t2b_status)

        control_layout.addWidget(t2b_group)

        # ── Task 2b internal state ─────────────────────────────────
        self.t2b_waypoints = []
        self.t2b_step_idx = 0
        self.t2b_playing = False
        self.t2b_pick_x = 200.0
        self.t2b_pick_y = 0.0
        self.t2b_place_x = 250.0
        self.t2b_place_y = 50.0
        self.t2b_cube_z = 45.0
        self.t2b_first_grip_idx = None
        self.t2b_timer = QTimer()
        self.t2b_timer.timeout.connect(self.t2b_tick)

        # Back to Home Button
        self.btn_home = QPushButton("BACK TO HOME (BYPASS SAFETY)")
        self.btn_home.clicked.connect(self.move_to_home)
        self.btn_home.setStyleSheet("background-color: #FFAA00; font-weight: bold;")
        control_layout.addWidget(self.btn_home)

        control_layout.addStretch()
        
        # Timer for animation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_motion)
        self.is_moving = False
        self.current_q = [0, 0, 0, 0] # Store current simulated joint angles

        # 2. Right Panel: 3D Visualization
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor)
        
        # Robot Renderer
        self.renderer = RobotRenderer(self.plotter)
        
        # Camera setup
        self.plotter.view_isometric()
        self.plotter.camera.azimuth += 45 # Adjust view
        
        # Initial Update
        self.update_robot()

    def update_robot(self):
        # Get Slider Values
        x = self.sliders['X'].value()
        y = self.sliders['Y'].value()
        z = self.sliders['Z'].value()
        pitch = self.sliders['Pitch'].value()
        
        # Update Labels
        self.labels['X'].setText(f"X: {x}")
        self.labels['Y'].setText(f"Y: {y}")
        self.labels['Z'].setText(f"Z: {z}")
        self.labels['Pitch'].setText(f"Pitch: {pitch}")
        
        try:
            # Call Python IK (returns clamped angles)
            q = IK(float(x), float(y), float(z), float(pitch))
            
            # Check if any joints were at their limits
            is_valid, violations = validate_joints(q)
            
            # Update joint angle labels
            limits = get_limits()
            for i in range(4):
                angle_str = f"J{i+1} ({JOINT_NAMES[i]}): {q[i]:.1f}° [{limits['min'][i]:.0f}°, {limits['max'][i]:.0f}°]"
                is_at_limit = abs(q[i] - limits['min'][i]) < 0.1 or abs(q[i] - limits['max'][i]) < 0.1
                if is_at_limit:
                    self.joint_labels[i].setStyleSheet("color: orange; font-weight: bold;")
                else:
                    self.joint_labels[i].setStyleSheet("")
                self.joint_labels[i].setText(angle_str)
            
            if not is_valid or any(abs(q[i] - limits['min'][i]) < 0.1 or abs(q[i] - limits['max'][i]) < 0.1 for i in range(4)):
                self.limit_status.setText("⚠ Joint(s) at limit")
                self.limit_status.setStyleSheet("color: orange; font-weight: bold;")
            else:
                self.limit_status.setText("✓ All joints within limits")
                self.limit_status.setStyleSheet("color: green; font-weight: bold;")
            
            # Use FK to get global transforms for visualization
            T_ee, global_transforms = FK(q)
            self.renderer.update_actors(global_transforms)
            
        except Exception as e:
            print(f"Kinematics Error: {e}")

    def start_motion(self):
        if self.is_moving:
            return
            
        self.safety_bypass = False
            
        # Get targets
        tx = self.target_inputs['x'].value()
        ty = self.target_inputs['y'].value()
        tz = self.target_inputs['z'].value()
        tp = self.target_inputs['pitch'].value()
        self.target_pose_mp = np.array([tx, ty, tz, tp])
        
        # Get current state from slider values
        cx = self.sliders['X'].value()
        cy = self.sliders['Y'].value()
        cz = self.sliders['Z'].value()
        cp = self.sliders['Pitch'].value()
        self.start_pose_mp = np.array([cx, cy, cz, cp])
        
        try:
            # Current Joint Angles
            self.start_q = np.array(IK(cx, cy, cz, cp))
            self.current_q_anim = self.start_q.copy()
            
            # Target Joint Angles (for Joint Interp)
            self.target_q = np.array(IK(tx, ty, tz, tp))
            
            # Setup Animation
            self.anim_time = 0
            self.jac_final_phase = False
            self.jac_final_time = 0.0
            self.jac_final_duration = 0.0
            self.jac_final_start_q = None
            self.jac_final_target_q = None
            velocity = self.vel_spin.value() # mm/s (Linear)
            ang_velocity = 45.0 # deg/s (Angular)
            
            dist_lin = np.linalg.norm(self.target_pose_mp[:3] - self.start_pose_mp[:3])
            dist_rot = abs(self.target_pose_mp[3] - self.start_pose_mp[3])
            
            dur_lin = dist_lin / velocity
            dur_rot = dist_rot / ang_velocity
            
            self.duration = max(dur_lin, dur_rot)
            
            if self.duration < 0.1: 
                self.duration = 0.1
                
            self.dt = 0.05 # 50ms steps
            
            self.is_moving = True
            self.btn_move.setEnabled(False)
            self.motion_status.setText("Moving...")
            self.timer.start(int(self.dt * 1000))
            
        except Exception as e:
            self.motion_status.setText(f"Error: {str(e)}")
            print(e)

    def move_to_home(self):
        self.target_inputs['x'].setValue(self.HOME_POSE[0])
        self.target_inputs['y'].setValue(self.HOME_POSE[1])
        self.target_inputs['z'].setValue(self.HOME_POSE[2])
        self.target_inputs['pitch'].setValue(self.HOME_POSE[3])
        self.start_motion()
        self.safety_bypass = True
        self.motion_status.setText("Moving Home (Unsafe)...")

    def set_camera_view(self, view):
        if view == 'center':
            self.plotter.view_xz()
            self.plotter.camera_position = 'xz'
            self.plotter.camera.azimuth += 90
            self.plotter.reset_camera()
        elif view == 'left':
            self.plotter.view_yz()
        elif view == 'right':
            self.plotter.view_yz()
            self.plotter.camera.azimuth += 180
            
    def update_camera_azimuth(self):
        self._update_camera_spherical()
        
    def update_camera_elevation(self):
        self._update_camera_spherical()
        
    def _update_camera_spherical(self):
        azim = np.deg2rad(self.slider_azim.value())
        elev = np.deg2rad(self.slider_elev.value())
        dist = 1000

        x = dist * np.cos(elev) * np.cos(azim)
        y = dist * np.cos(elev) * np.sin(azim)
        z = dist * np.sin(elev)
        
        focal_point = np.array(self.plotter.camera.focal_point)
        self.plotter.camera.position = focal_point + np.array([x, y, z])
        self.plotter.render()

    def stop_motion(self):
        self.is_moving = False
        self.timer.stop()
        self.btn_move.setEnabled(True)
        self.motion_status.setText("Stopped.")
        self.safety_bypass = False

    def update_motion(self):
        if not self.is_moving:
            return
            
        self.anim_time += self.dt
        t_normalized = self.anim_time / self.duration
        if t_normalized > 1.0:
            t_normalized = 1.0
            
        mode = self.mode_group.checkedId()
        
        new_q = None
        
        try:
            prev_fk_T, _ = FK(self.current_q_anim)
            z_prev = prev_fk_T[2, 3]
            
            if mode == 0: # Joint Interpolation
                new_q = self.start_q + (self.target_q - self.start_q) * t_normalized
                
            elif mode == 1: # Task Interpolation
                curr_pose_target = self.start_pose_mp + (self.target_pose_mp - self.start_pose_mp) * t_normalized
                new_q = IK(curr_pose_target[0], curr_pose_target[1], curr_pose_target[2], curr_pose_target[3])
                
            elif mode == 2: # Jacobian Control
                if self.jac_final_phase:
                    self.jac_final_time += self.dt
                    s = self.jac_final_time / self.jac_final_duration if self.jac_final_duration > 1e-9 else 1.0
                    s = np.clip(s, 0.0, 1.0)
                    new_q = self.jac_final_start_q + (self.jac_final_target_q - self.jac_final_start_q) * s
                    if s >= 1.0:
                        self.stop_motion()
                        self.motion_status.setText("Target Reached (Jac Hybrid).")
                        return
                else:
                    current_fk_T, _ = FK(self.current_q_anim)
                    current_pos = current_fk_T[:3, 3]
                
                    curr_q = self.current_q_anim
                    current_pitch = -(curr_q[1] + curr_q[2] + curr_q[3])
                
                    target_pos = self.target_pose_mp[:3]
                    target_pitch = self.target_pose_mp[3]
                
                    error_pos = target_pos - current_pos
                    error_pitch = target_pitch - current_pitch
                
                    handoff_pos_mm = 15.0
                    handoff_pitch_deg = 8.0
                    if np.linalg.norm(error_pos) < handoff_pos_mm and abs(error_pitch) < handoff_pitch_deg:
                        self.jac_final_phase = True
                        self.jac_final_time = 0.0
                        self.jac_final_start_q = self.current_q_anim.copy()
                        self.jac_final_target_q = np.array(IK(
                            self.target_pose_mp[0],
                            self.target_pose_mp[1],
                            self.target_pose_mp[2],
                            self.target_pose_mp[3]
                        ))
                        max_delta = float(np.max(np.abs(self.jac_final_target_q - self.jac_final_start_q)))
                        final_joint_speed_deg_s = 90.0
                        self.jac_final_duration = max(0.12, max_delta / final_joint_speed_deg_s)
                        self.motion_status.setText("Final approach...")
                        return
                
                    vel_mag = self.vel_spin.value()
                
                    Kp_pos = 2.0
                    v_lin = Kp_pos * error_pos
                    v_norm = np.linalg.norm(v_lin)
                    if v_norm > vel_mag and v_norm > 1e-9:
                        v_lin = v_lin * (vel_mag / v_norm)
                
                    Kp_rot = 2.0
                    w_pitch_rad = Kp_rot * np.deg2rad(error_pitch)
                    w_pitch_rad = np.clip(w_pitch_rad, -np.deg2rad(90), np.deg2rad(90))
                
                    J = get_jacobian(self.current_q_anim)
                
                    J_pitch_row = np.array([0.0, -1.0, -1.0, -1.0], dtype=float)
                
                    J_task = np.vstack([J[0:3, :], J_pitch_row])
                
                    v_task = np.append(v_lin, w_pitch_rad)
                
                    lambda_val = 0.05
                    try:
                        J_dls = J_task.T @ np.linalg.inv(J_task @ J_task.T + lambda_val**2 * np.eye(4))
                        q_dot_rad = J_dls @ v_task
                        q_dot_deg = np.rad2deg(q_dot_rad)
                        q_dot_deg = np.clip(q_dot_deg, -120.0, 120.0)
                        
                        new_q = self.current_q_anim + q_dot_deg * self.dt
                        
                    except np.linalg.LinAlgError:
                        print("Jacobian Singularity")
                        self.stop_motion()
                        return
            
            # --- SAFETY CHECK: Z Floor ---
            if new_q is not None:
                T_check, _ = FK(new_q)
                z_check = T_check[2, 3]
                
                Z_LIMIT = 20.0
                
                if z_check < Z_LIMIT and not self.safety_bypass:
                    if z_check > (z_prev + 0.01): 
                        pass 
                    else:
                        self.stop_motion()
                        self.motion_status.setText(f"SAFETY STOP: Z ({z_check:.1f}) < {Z_LIMIT}mm!")
                        return
            
            if new_q is not None:
                self.current_q_anim = new_q
            
            if new_q is not None:
                T_ee, _ = FK(new_q)
                x_new = T_ee[0, 3]
                y_new = T_ee[1, 3]
                z_new = T_ee[2, 3]
                pitch_new = -(new_q[1] + new_q[2] + new_q[3])
                
                self.sliders['X'].blockSignals(True)
                self.sliders['Y'].blockSignals(True)
                self.sliders['Z'].blockSignals(True)
                self.sliders['Pitch'].blockSignals(True)
                
                self.sliders['X'].setValue(int(x_new))
                self.sliders['Y'].setValue(int(y_new))
                self.sliders['Z'].setValue(int(z_new))
                self.sliders['Pitch'].setValue(int(pitch_new))
                
                self.sliders['X'].blockSignals(False)
                self.sliders['Y'].blockSignals(False)
                self.sliders['Z'].blockSignals(False)
                self.sliders['Pitch'].blockSignals(False)
                
                self.update_explicit_q(new_q)
            
            if t_normalized >= 1.0 and mode != 2:
                self.stop_motion()
                self.motion_status.setText("Target Reached.")
                
        except Exception as e:
            print(f"Motion Error: {e}")
            self.stop_motion()

    def update_explicit_q(self, q):
        try:
            limits = get_limits()
            for i in range(4):
                angle_str = f"J{i+1} ({JOINT_NAMES[i]}): {q[i]:.1f}° [{limits['min'][i]:.0f}°, {limits['max'][i]:.0f}°]"
                self.joint_labels[i].setText(angle_str)
            
            T_ee, global_transforms = FK(q)
            self.renderer.update_actors(global_transforms)
            
            x = T_ee[0, 3]
            y = T_ee[1, 3]
            z = T_ee[2, 3]
            pitch = -(q[1] + q[2] + q[3])
            
            self.labels['X'].setText(f"X: {x:.1f}")
            self.labels['Y'].setText(f"Y: {y:.1f}")
            self.labels['Z'].setText(f"Z: {z:.1f}")
            self.labels['Pitch'].setText(f"Pitch: {pitch:.1f}")
            
        except Exception as e:
            print(f"Explicit Update Error: {e}")

    # ── Gripper UI Handlers ────────────────────────────────────────

    def update_gripper_from_slider(self, value):
        self.gripper.set_position_pct(value)
        self._sync_gripper_ui()

    def gripper_open(self):
        self.gripper.open()
        self._sync_gripper_ui()

    def gripper_close(self):
        self.gripper.close()
        self._sync_gripper_ui()

    def gripper_object(self, width_mm):
        self.gripper.grip_object(width_mm)
        self._sync_gripper_ui()

    def apply_grip_mode(self):
        mode_id = self.grip_mode_group.checkedId()
        if mode_id == 0:  # Position
            self.gripper.set_position_pct(self.grip_slider.value())
        elif mode_id == 1:  # Object-Aware
            self.gripper.grip_object(self.grip_obj_spin.value())
        elif mode_id == 2:  # Force-Limited
            self.gripper.grip_force(self.grip_force_spin.value())
        self._sync_gripper_ui()

    def _sync_gripper_ui(self):
        """Push gripper state to slider, status label, and 3D renderer."""
        state = self.gripper.get_state()
        # Update slider without re-triggering
        self.grip_slider.blockSignals(True)
        self.grip_slider.setValue(int(state['pct']))
        self.grip_slider.blockSignals(False)
        self.grip_slider_label.setText(f"{state['pct']:.0f}%")
        # Status text
        mode_str = state['mode'].capitalize()
        self.grip_status.setText(
            f"{mode_str} | {state['pct']:.0f}% | "
            f"Jaw: {state['jaw_width_mm']:.1f}mm | "
            f"Enc: {state['encoder']} | "
            f"Force: {state['force_pct']:.0f}%"
        )
        # Update 3D jaws
        self.renderer.update_gripper(state['jaw_width_mm'])

    # ── Task 2b Methods ────────────────────────────────────────────

    def t2b_load(self):
        """Generate waypoints from current inputs and place the cube in the scene."""
        self.t2b_pick_x = self.t2b_inputs['pick_x'].value()
        self.t2b_pick_y = self.t2b_inputs['pick_y'].value()
        self.t2b_place_x = self.t2b_inputs['place_x'].value()
        self.t2b_place_y = self.t2b_inputs['place_y'].value()

        rotate_mode = self.t2b_rot_mode_group.checkedId()

        self.t2b_waypoints = generate_pick_rotate_sequence(
            pick_x=self.t2b_pick_x,
            pick_y=self.t2b_pick_y,
            place_x=self.t2b_place_x,
            place_y=self.t2b_place_y,
            cube_z_surface=self.t2b_cube_z,
            cube_size=CUBE_SIZE_MM,
            rotate_mode=rotate_mode,
        )
        self.t2b_step_idx = 0

        # Pre-compute first index where cube is gripped
        self.t2b_first_grip_idx = None
        for i, wp in enumerate(self.t2b_waypoints):
            if wp.gripper_pct >= 50:
                self.t2b_first_grip_idx = i
                break

        # Remove old cubes, add fresh one
        self.renderer.remove_cubes()
        cube_centre_z = self.t2b_cube_z + CUBE_SIZE_MM / 2.0
        self.renderer.add_cube(
            'cube1',
            (self.t2b_pick_x, self.t2b_pick_y, cube_centre_z),
            size=CUBE_SIZE_MM,
        )

        # Update scrubber
        self.t2b_scrubber.setRange(0, len(self.t2b_waypoints) - 1)
        self.t2b_scrubber.setValue(0)
        self.t2b_scrubber.setEnabled(True)

        # Enable buttons
        self.btn_t2b_play.setEnabled(True)
        self.btn_t2b_pause.setEnabled(True)
        self.btn_t2b_reset.setEnabled(True)

        self.t2b_status.setText(f"Loaded {len(self.t2b_waypoints)} steps")

        # Show first waypoint
        self._t2b_apply_step(0)

    def t2b_play(self):
        """Start / resume playback."""
        if not self.t2b_waypoints:
            return
        self.t2b_playing = True
        speed = self.t2b_speed_slider.value() / 2.0
        interval_ms = max(10, int(80 / speed))  # base ~80 ms per step
        self.t2b_timer.start(interval_ms)
        self.t2b_status.setText("Playing…")

    def t2b_pause(self):
        self.t2b_playing = False
        self.t2b_timer.stop()
        self.t2b_status.setText("Paused")

    def t2b_reset(self):
        self.t2b_pause()
        self.t2b_step_idx = 0
        self._t2b_apply_step(0)
        self.t2b_scrubber.setValue(0)
        self.t2b_status.setText("Reset")

    def t2b_scrub_to(self, value):
        """Jump to a specific step (from the scrubber slider)."""
        if not self.t2b_waypoints:
            return
        self.t2b_step_idx = value
        self._t2b_apply_step(value)

    def t2b_tick(self):
        """Called by the timer each frame during playback."""
        if not self.t2b_playing:
            return
        if self.t2b_step_idx >= len(self.t2b_waypoints) - 1:
            self.t2b_pause()
            self.t2b_status.setText("Done!")
            return
        self.t2b_step_idx += 1
        # Update scrubber without re-triggering scrub_to
        self.t2b_scrubber.blockSignals(True)
        self.t2b_scrubber.setValue(self.t2b_step_idx)
        self.t2b_scrubber.blockSignals(False)
        self._t2b_apply_step(self.t2b_step_idx)

    def _t2b_apply_step(self, idx):
        """Apply a single waypoint: move robot, update gripper, update cube."""
        if idx < 0 or idx >= len(self.t2b_waypoints):
            return
        wp = self.t2b_waypoints[idx]

        # Update step label
        self.t2b_step_label.setText(f"{idx} / {len(self.t2b_waypoints) - 1}")
        self.t2b_status.setText(wp.label)

        try:
            q = IK(wp.x, wp.y, wp.z, wp.pitch)
            T_ee, global_transforms = FK(q)
            self.renderer.update_actors(global_transforms)

            # Sync sliders
            self.sliders['X'].blockSignals(True)
            self.sliders['Y'].blockSignals(True)
            self.sliders['Z'].blockSignals(True)
            self.sliders['Pitch'].blockSignals(True)
            self.sliders['X'].setValue(int(wp.x))
            self.sliders['Y'].setValue(int(wp.y))
            self.sliders['Z'].setValue(int(wp.z))
            self.sliders['Pitch'].setValue(int(wp.pitch))
            self.sliders['X'].blockSignals(False)
            self.sliders['Y'].blockSignals(False)
            self.sliders['Z'].blockSignals(False)
            self.sliders['Pitch'].blockSignals(False)

            self.labels['X'].setText(f"X: {wp.x:.1f}")
            self.labels['Y'].setText(f"Y: {wp.y:.1f}")
            self.labels['Z'].setText(f"Z: {wp.z:.1f}")
            self.labels['Pitch'].setText(f"Pitch: {wp.pitch:.1f}")

            # Gripper
            self.gripper.set_position_pct(wp.gripper_pct)
            self._sync_gripper_ui()

            # ── Cube position ──────────────────────────────────────
            is_gripped = wp.gripper_pct >= 50
            if is_gripped:
                # Cube follows the end-effector tip.
                # T_ee is the full EE transform; cube centre is at EE tip.
                T_cube = np.array(T_ee)
                # The EE X-axis points along the tool — nudge back so the
                # cube centre aligns with the gripper jaw centre.
                # (no offset needed; EE position IS the tip)
                self.renderer.update_cube_transform('cube1', T_cube)
            else:
                # Cube sits at its resting position: at pick before first grip,
                # at place after.
                T_rest = np.eye(4)
                if self.t2b_first_grip_idx is not None and idx > self.t2b_first_grip_idx:
                    T_rest[0, 3] = self.t2b_place_x
                    T_rest[1, 3] = self.t2b_place_y
                else:
                    T_rest[0, 3] = self.t2b_pick_x
                    T_rest[1, 3] = self.t2b_pick_y
                T_rest[2, 3] = self.t2b_cube_z + CUBE_SIZE_MM / 2.0
                self.renderer.update_cube_transform('cube1', T_rest)

        except Exception as e:
            print(f"Task2b step error: {e}")

    def closeEvent(self, event):
        print("Closing Application...")
        self.t2b_timer.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
