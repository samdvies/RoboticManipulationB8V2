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
        # Sequence support for scripted demo motions
        self.sequence_queue = []
        self.sequence_active = False
        self.sequence_on_complete = None

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

        # Scripted Demo Buttons
        demo_layout = QHBoxLayout()
        self.btn_demo_cup = QPushButton("Cup Pour Demo")
        self.btn_demo_cup.clicked.connect(self.run_cup_pour_demo)
        self.btn_demo_obj = QPushButton("Object Path Demo")
        self.btn_demo_obj.clicked.connect(self.run_object_path_demo)
        self.btn_demo_cup2 = QPushButton("Cup 2 Pour Demo")
        self.btn_demo_cup2.clicked.connect(self.run_second_cup_pour_demo)
        demo_layout.addWidget(self.btn_demo_cup)
        demo_layout.addWidget(self.btn_demo_obj)
        demo_layout.addWidget(self.btn_demo_cup2)
        motion_layout.addLayout(demo_layout)
        
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
            
        # When starting a standalone move from the UI, cancel any queued sequence
        if not self.sequence_active:
            self.sequence_queue = []
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
            if not self.sequence_active:
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

        # Cancel any remaining scripted sequence
        self.sequence_active = False
        self.sequence_queue = []

    def _complete_motion_step(self, status_text="Target Reached."):
        """Handle natural completion of a motion (not an emergency stop)."""
        self.is_moving = False
        self.timer.stop()
        self.btn_move.setEnabled(True)
        self.safety_bypass = False

        if self.sequence_active:
            if self.sequence_queue:
                # Proceed to next step in the scripted sequence
                self.motion_status.setText("Sequence step complete...")
                self._start_next_in_sequence()
            else:
                # Final step of the current sequence reached
                self.sequence_active = False
                cb = self.sequence_on_complete
                self.sequence_on_complete = None
                if cb is not None:
                    cb()
                else:
                    self.motion_status.setText(status_text)
        else:
            # Standalone move completion
            self.motion_status.setText(status_text)

    def _start_sequence(self, poses, on_complete=None):
        """Initialize a new scripted motion sequence given a list of task-space poses."""
        if self.is_moving:
            return
        self.sequence_queue = list(poses)
        self.sequence_active = True
        self.sequence_on_complete = on_complete
        # Use linear task-space interpolation for sequences
        self.radio_task.setChecked(True)
        self._start_next_in_sequence()

    def _start_next_in_sequence(self):
        """Start the next motion in the active scripted sequence."""
        if not self.sequence_queue:
            self.sequence_active = False
            self.motion_status.setText("Sequence complete.")
            return

        pose = self.sequence_queue.pop(0)
        x, y, z, pitch = pose
        self.target_inputs['x'].setValue(float(x))
        self.target_inputs['y'].setValue(float(y))
        self.target_inputs['z'].setValue(float(z))
        self.target_inputs['pitch'].setValue(float(pitch))
        self.start_motion()

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
                        self._complete_motion_step("Target Reached (Jac Hybrid).")
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
                self._complete_motion_step("Target Reached.")
                
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

    # ── Scripted Demo Sequences ─────────────────────────────────────

    def run_cup_pour_demo(self):
        """
        Demo 1:
        - Pick the first cup at (75, -175, 60, pitch=0)
        - Pour into the second cup at (200, 0)
        - Return the cup to its original pose.
        """
        if self.is_moving:
            return

        # Start with gripper wide open so we approach the cup safely
        self.gripper_open()

        # Source cup at (75, -175)
        src_x, src_y = 75.0, -175.0

        # Phase 1: approach the source cup, respecting Z>=100 for horizontal motion
        approach_poses = [
            np.array([src_x, src_y, 150.0, 0.0]),  # Above cup, safe height
            np.array([src_x, src_y, 100.0, 0.0]),  # Just above rim
            np.array([src_x, src_y, 60.0, 0.0]),   # Pick height
        ]

        def after_pick():
            # Tighten grip once we're at Z=60 to hold the cup (approx 60mm wide)
            self.gripper_object(60.0)

            # Phase 2: carry to the target cup at (125, 0),
            # pour while staying above the receiving cup height (~100mm),
            # then return the cup to its original position.
            tgt_x, tgt_y = 160.0, 0.0
            carry_pour_and_return_poses = [
                # Carry to second cup
                np.array([src_x, src_y, 150.0, 0.0]),          # Lift cup up vertically
                np.array([tgt_x, tgt_y, 150.0, 0.0]),          # Move over target cup at safe Z
                np.array([tgt_x, tgt_y, 130.0, 0.0]),          # Lower over target cup (still above 100mm)
                np.array([tgt_x, tgt_y, 130.0, -60.0]),        # Start pour
                # Upright and lift back up
                np.array([tgt_x, tgt_y, 150.0, 0.0]),          # Lift and return pitch to 0
                # Return to original cup position
                np.array([src_x, src_y, 150.0, 0.0]),          # Above source
                np.array([src_x, src_y, 60.0, 0.0]),           # Back to original pick height
            ]

            self._start_sequence(carry_pour_and_return_poses, on_complete=self.gripper_open)

        # First run the approach phase; when that finishes, `after_pick`
        # will be invoked to grip and continue with the pour.
        self._start_sequence(approach_poses, on_complete=after_pick)

    def run_second_cup_pour_demo(self):
        """
        Demo 3:
        - Take the second cup at (200, 0, 60, pitch=0)
        - Move it toward a "mouth" region in front/above the robot
        - Perform a natural-looking pour motion along a short arc
        - Repeat the pour motion 3 times.
        """
        if self.is_moving:
            return

        # Start with gripper open to safely approach the cup
        self.gripper_open()

        # Second cup initial pose and "mouth" region
        src_x, src_y, src_z = 200.0, 0.0, 60.0
        # Start roughly at 150, 150, 100 and move up/out to about 200, 200, 150
        mouth_start = np.array([150.0, 150.0, 100.0, 0.0])
        mouth_end   = np.array([175.0, 175.0, 125.0, -60.0])

        # Phase 1: approach and pick the second cup
        approach_poses = [
            np.array([src_x, src_y, 150.0, 0.0]),   # Above cup
            np.array([src_x, src_y, 100.0, 0.0]),   # Just above rim
            np.array([src_x, src_y, src_z,  0.0]),  # At cup height (60mm)
        ]

        def after_second_pick():
            # Grip the cup to a 60mm jaw width
            self.gripper_object(60.0)

            poses = []
            # Move from pickup to "mouth" start position
            poses.append(np.array([src_x, src_y, 150.0, 0.0]))          # Lift cup up
            poses.append(mouth_start.copy())                            # To starting drink pose

            # Three "sip" cycles: follow an arc toward the mouth while rotating,
            # then come back to the start pose.
            for _ in range(3):
                poses.append(mouth_end.copy())                          # Full tilt
                poses.append(mouth_start.copy())                        # Upright at start

            # Return cup 2 to its original pickup position
            poses.append(np.array([src_x, src_y, 150.0, 0.0]))          # Above pickup
            poses.append(np.array([src_x, src_y, src_z,  0.0]))         # At pickup height

            def release_and_lift():
                self.gripper_open()

            self._start_sequence(poses, on_complete=release_and_lift)

        self._start_sequence(approach_poses, on_complete=after_second_pick)

    def run_object_path_demo(self):
        """
        Demo 2:
        - Grip a 25mm object at (-150, 150, 160, pitch=0)
        - Move it to (175, 0, 270, pitch=0)
        - Then move around the specified coordinates 4 times.
        """
        if self.is_moving:
            return

        # Keep gripper fully open (~80mm) until we are at the stirrer, then
        # close to a 25mm gap at the pick height.
        self.gripper_open()

        # Approach and pick the stirrer at (150, -150, 170)
        src_x, src_y = 150.0, -150.0
        center_x, center_y = 200.0, 0.0

        approach_poses = [
            np.array([src_x, src_y, 210.0, 0.0]),   # Above object
            np.array([src_x, src_y, 170.0, 0.0]),   # At object height
        ]

        def after_stir_pick():
            # Now grip the stirrer to a 24mm jaw width
            self.gripper_object(24.0)

            poses = []

            # Move to drop/carry position near (200, 0, 270) without dipping near the second cup height
            poses.append(np.array([src_x,      0.0, 270.0, 0.0]))      # Intermediate over center line
            poses.append(np.array([center_x, center_y, 310.0, 0.0]))   # Above stirring center
            poses.append(np.array([center_x, center_y, 270.0, 0.0]))   # Stirring center height

            # New stirring coordinates around X=200 (Z=180, pitch=0)
            path_points = [
                (207.500,  3.107, 180.0),
                (203.107,  7.500, 180.0),
                (196.893,  7.500, 180.0),
                (192.500,  3.107, 180.0),
                (192.500, -3.107, 180.0),
                (196.893, -7.500, 180.0),
                (203.107, -7.500, 180.0),
                (207.500, -3.107, 180.0),
            ]

            for _ in range(4):  # Loop around the path 4 times
                for x, y, z in path_points:
                    poses.append(np.array([float(x), float(y), float(z), 0.0]))

            # Return stirrer to its original position without descending near the second cup
            poses.extend([
                np.array([center_x, center_y, 270.0, 0.0]),    # Back to stirring center
                np.array([src_x,      0.0,    270.0, 0.0]),    # Move away in Y only
                np.array([src_x,    src_y,    210.0, 0.0]),    # Above original pick
                np.array([src_x,    src_y,    170.0, 0.0]),    # Original pick height
            ])

            self._start_sequence(poses, on_complete=self.gripper_open)

        # First, just move to the stirrer with the gripper fully open. Once
        # that sequence finishes, we grip to 25mm and run the stirring path.
        self._start_sequence(approach_poses, on_complete=after_stir_pick)

    def closeEvent(self, event):
        print("Closing Application...")
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
