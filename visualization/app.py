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
                             QPushButton, QRadioButton, QButtonGroup, QDoubleSpinBox)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.kinematics.JointLimits import clamp_joints, validate_joints, get_limits, JOINT_NAMES
from visualization.kinematics.Jacobian import get_jacobian

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Live Simulation (Task Space Control)")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize Python Kinematics
        print("Using Python Kinematics (No MATLAB)")
        
        self.HOME_POSE = np.array([200, 0, 100, 0]) # X, Y, Z, Pitch
        self.safety_bypass = False

        # Central Widget and Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # 1. Left Panel: Controls
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_panel.setFixedWidth(300)
        main_layout.addWidget(control_panel)

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
        
        # Motion Control Group (New)
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
        self.slider_azim.setValue(45) # Default
        self.slider_azim.valueChanged.connect(self.update_camera_azimuth)
        cam_grid.addWidget(self.slider_azim, 0, 1)
        
        # Elevation
        cam_grid.addWidget(QLabel("Elevation:"), 1, 0)
        self.slider_elev = QSlider(Qt.Horizontal)
        self.slider_elev.setRange(0, 90)
        self.slider_elev.setValue(30) # Default
        self.slider_elev.valueChanged.connect(self.update_camera_elevation)
        cam_grid.addWidget(self.slider_elev, 1, 1)
        
        cam_layout.addLayout(cam_grid)
        control_layout.addWidget(cam_group)

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
            
        self.safety_bypass = False # Always reset unless explicitly set by move_to_home (which calls this, then sets it True)
            
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
            ang_velocity = 45.0 # deg/s (Angular - fixed for now, or scaled)
            
            dist_lin = np.linalg.norm(self.target_pose_mp[:3] - self.start_pose_mp[:3])
            dist_rot = abs(self.target_pose_mp[3] - self.start_pose_mp[3])
            
            dur_lin = dist_lin / velocity
            dur_rot = dist_rot / ang_velocity
            
            self.duration = max(dur_lin, dur_rot)
            
            # Ensure minimum duration to avoid div/0 or instant snaps
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
        # Set target inputs to Home Pose
        self.target_inputs['x'].setValue(self.HOME_POSE[0])
        self.target_inputs['y'].setValue(self.HOME_POSE[1])
        self.target_inputs['z'].setValue(self.HOME_POSE[2])
        self.target_inputs['pitch'].setValue(self.HOME_POSE[3])
        
        # Start motion
        self.start_motion()
        
        # Enable Bypass (Must be done AFTER start_motion resets it)
        self.safety_bypass = True
        self.motion_status.setText("Moving Home (Unsafe)...")

    def set_camera_view(self, view):
        if view == 'center':
            self.plotter.view_xz() # Side view matches 'Center' logic? Or YZ? 
            # Let's try Isometric for Center or just Front.
            # Usually X is forward. Front view is looking from +x to origin?
            self.plotter.camera_position = 'xz'
            self.plotter.camera.azimuth += 90 # Adjust to face front
            self.plotter.reset_camera()
        elif view == 'left':
            self.plotter.view_yz()
        elif view == 'right':
            self.plotter.view_yz()
            self.plotter.camera.azimuth += 180
            
    def update_camera_azimuth(self):
        val = self.slider_azim.value()
        # We can't easily set absolute azimuth without knowing the base.
        # But we can orbit.
        # Better: Set position on sphere.
        self._update_camera_spherical()
        
    def update_camera_elevation(self):
        val = self.slider_elev.value()
        self._update_camera_spherical()
        
    def _update_camera_spherical(self):
        # Calculate position from Azimuth/Elevation sliders
        azim = np.deg2rad(self.slider_azim.value())
        elev = np.deg2rad(self.slider_elev.value())
        dist = 1000 # Fixed distance
        
        # Spherical to Cartesian
        # Z is up
        x = dist * np.cos(elev) * np.cos(azim)
        y = dist * np.cos(elev) * np.sin(azim)
        z = dist * np.sin(elev)
        
        # Allow user to "Lock" to X/Y/Z adjustments effectively by using these sliders.
        focal_point = np.array(self.plotter.camera.focal_point)
        self.plotter.camera.position = focal_point + np.array([x, y, z])
        self.plotter.render()

    def stop_motion(self):
        self.is_moving = False
        self.timer.stop()
        self.btn_move.setEnabled(True)
        self.motion_status.setText("Stopped.")
        self.safety_bypass = False # Reset on stop as well

    def update_motion(self):
        if not self.is_moving:
            return
            
        self.anim_time += self.dt
        t_normalized = self.anim_time / self.duration
        if t_normalized > 1.0:
            t_normalized = 1.0
            
        mode = self.mode_group.checkedId()
        
        new_q = None
        
        # Determine previous state for safety comparison
        # (current_q_anim holds the state from the end of the previous step)
        try:
            prev_fk_T, _ = FK(self.current_q_anim)
            z_prev = prev_fk_T[2, 3]
            
            if mode == 0: # Joint Interpolation
                # Linear Interpolation of Joints
                new_q = self.start_q + (self.target_q - self.start_q) * t_normalized
                
            elif mode == 1: # Task Interpolation
                # Linear Interpolation of Task Space (X, Y, Z, Pitch)
                curr_pose_target = self.start_pose_mp + (self.target_pose_mp - self.start_pose_mp) * t_normalized
                new_q = IK(curr_pose_target[0], curr_pose_target[1], curr_pose_target[2], curr_pose_target[3])
                
            elif mode == 2: # Jacobian Control
                # Hybrid mode:
                # 1) Jacobian resolved-rate motion to approach target.
                # 2) Final joint interpolation phase for exact arrival.
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
                    # Calculate errors
                    current_fk_T, _ = FK(self.current_q_anim)
                    current_pos = current_fk_T[:3, 3] # XYZ
                
                    # Current pitch convention used across the simulator
                    curr_q = self.current_q_anim
                    current_pitch = -(curr_q[1] + curr_q[2] + curr_q[3])
                
                    target_pos = self.target_pose_mp[:3]
                    target_pitch = self.target_pose_mp[3]
                
                    error_pos = target_pos - current_pos
                    error_pitch = target_pitch - current_pitch
                
                    # Hybrid handoff threshold: close enough for deterministic final approach.
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
                
                    # Control gains
                    vel_mag = self.vel_spin.value()
                
                    # Linear velocity command (P control + saturation)
                    Kp_pos = 2.0 # 1/s
                    v_lin = Kp_pos * error_pos
                    v_norm = np.linalg.norm(v_lin)
                    if v_norm > vel_mag and v_norm > 1e-9:
                        v_lin = v_lin * (vel_mag / v_norm)
                
                    # Angular Velocity Command
                    Kp_rot = 2.0
                    w_pitch_rad = Kp_rot * np.deg2rad(error_pitch)
                    w_pitch_rad = np.clip(w_pitch_rad, -np.deg2rad(90), np.deg2rad(90))
                
                    # J is 6x4 (Rows: vx, vy, vz, wx, wy, wz)
                    J = get_jacobian(self.current_q_anim) # 6x4
                
                    # Pitch convention in this simulator is:
                    # pitch = -(q2 + q3 + q4)
                    # therefore d(pitch_rad)/d(q_rad) = [0, -1, -1, -1]
                    J_pitch_row = np.array([0.0, -1.0, -1.0, -1.0], dtype=float)
                
                    # Construct Task Jacobian (4x4)
                    # [ Jv (3x4)      ]
                    # [ J_pitch (1x4) ]
                    J_task = np.vstack([J[0:3, :], J_pitch_row])
                
                    # Task Velocity Vector
                    v_task = np.append(v_lin, w_pitch_rad)
                
                    # Damped Least Squares
                    lambda_val = 0.05
                    # We need to solve J_task * q_dot = v_task
                    try:
                        J_dls = J_task.T @ np.linalg.inv(J_task @ J_task.T + lambda_val**2 * np.eye(4))
                        q_dot_rad = J_dls @ v_task
                        q_dot_deg = np.rad2deg(q_dot_rad)
                        q_dot_deg = np.clip(q_dot_deg, -120.0, 120.0)
                        
                        # Integrate
                        new_q = self.current_q_anim + q_dot_deg * self.dt
                        
                    except np.linalg.LinAlgError:
                        print("Jacobian Singularity")
                        self.stop_motion()
                        return
            
            # --- SAFETY CHECK: Z Floor ---
            # Compute FK of proposed new_q
            if new_q is not None:
                T_check, _ = FK(new_q)
                z_check = T_check[2, 3]
                
                # Safety Limit
                Z_LIMIT = 20.0
                
                # Check Z floor unless bypassed
                if z_check < Z_LIMIT and not self.safety_bypass:
                    # Check if we are recovering (moving up)
                    # Use a small tolerance or ensure strictly greater
                    if z_check > (z_prev + 0.01): 
                        # Allow upward movement even in danger zone
                        pass 
                    else:
                        self.stop_motion()
                        self.motion_status.setText(f"SAFETY STOP: Z ({z_check:.1f}) < {Z_LIMIT}mm!")
                        return # Do not update
            
            # Update Internal Phase State for next iteration
            if new_q is not None:
                self.current_q_anim = new_q
            
            # Apply to Sliders (which triggers update_robot)
            if new_q is not None:
                # We update the sliders to reflect the new state.
                # Since we want to visualize 'new_q', and sliders drive IK, this is tricky.
                # However, calculating the Pitch from Q is valid.
                
                T_ee, _ = FK(new_q)
                x_new = T_ee[0, 3]
                y_new = T_ee[1, 3]
                z_new = T_ee[2, 3]
                pitch_new = -(new_q[1] + new_q[2] + new_q[3])
                
                # Hack: Update sliders without triggering signals to avoid double-update?
                # Actually, we WANT update_robot to run to update the visualization.
                # BUT update_robot calls IK.
                # If we are in Jacobian mode, new_q might NOT match IK(x,y,z,p) exactly if redundancies exist (unlikely for 4DoF)
                # or if we are drifting.
                # Ideally we just call update_explicit_q(new_q) and update sliders SILENTLY.
                
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
                
                # Manually call update with our explicit Q
                self.update_explicit_q(new_q)
            
            if t_normalized >= 1.0 and mode != 2:
                self.stop_motion()
                self.motion_status.setText("Target Reached.")
                
        except Exception as e:
            print(f"Motion Error: {e}")
            self.stop_motion()

    def update_explicit_q(self, q):
        # Helper to update visualization with explicit joint angles
        try:
            # Update labels
            limits = get_limits()
            for i in range(4):
                angle_str = f"J{i+1} ({JOINT_NAMES[i]}): {q[i]:.1f}° [{limits['min'][i]:.0f}°, {limits['max'][i]:.0f}°]"
                self.joint_labels[i].setText(angle_str)
            
            T_ee, global_transforms = FK(q)
            self.renderer.update_actors(global_transforms)
            
            # Update Task Labels based on FK
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

    def closeEvent(self, event):
        print("Closing Application...")
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
