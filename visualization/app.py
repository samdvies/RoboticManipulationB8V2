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
                             QScrollArea, QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.kinematics.JointLimits import clamp_joints, validate_joints, get_limits, JOINT_NAMES
from visualization.kinematics.Jacobian import get_jacobian
from visualization.kinematics.GripperSim import GripperSim
from visualization.kinematics.BridgeAvoidance import (
    BridgeNoGoZone,
    build_bridge_zones,
    clamp_gripper_width_for_bridge,
    plan_bridge_safe_waypoints,
)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Live Simulation (Task Space Control)")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize Python Kinematics
        print("Using Python Kinematics (No MATLAB)")
        
        self.HOME_POSE = np.array([200, 0, 180, 0]) # X, Y, Z, Pitch
        self.safety_bypass = False
        self.gripper = GripperSim()
        self.bridge_no_go_zone = BridgeNoGoZone(
            x_min=187.5, x_max=212.5,   # 25mm wide in X, centered at X=200
            y_min=-35.0, y_max=35.0,     # outer edges of pillars (50mm gap + 10mm pillar each side)
            z_min=0.0,   z_max=60.0      # 60mm high from ground
        )
        # Pillar geometry (Y ranges) for visual rendering
        self.bridge_pillar_width_y = 10.0   # each pillar is 10mm thick in Y
        self.bridge_gap_y = 50.0            # 50mm gap between inner pillar faces
        self.bridge_zones = build_bridge_zones(
            self.bridge_no_go_zone,
            gap_y=self.bridge_gap_y,
            pillar_width_y=self.bridge_pillar_width_y
        )
        self.bridge_approach_margin_mm = 60.0
        self.bridge_vertical_clearance_mm = 30.0
        self.bridge_gripper_min_mm = 25.0
        self.bridge_gripper_max_mm = 50.0
        self.motion_waypoints = []
        self.motion_segment_idx = 0
        self.bridge_route_active = False

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
            ('Z', 0, 450, 180),     # Up/Down range (0 to Max Height) -> default 180 avoids bridge!
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
        defaults = [200, 0, 30, 0] # Default target is UNDER the bridge (bridge deck at Z=60)
        
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
        self.grip_obj_spin.setRange(25, 50)
        self.grip_obj_spin.setValue(30)
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

        # ── Bridge Control Group ───────────────────────────────────────
        bridge_group = QGroupBox("Bridge Control")
        bridge_layout = QGridLayout()
        bridge_group.setLayout(bridge_layout)

        self.bridge_enabled_check = QCheckBox("Enable No-Go Zone + Reroute")
        self.bridge_enabled_check.setChecked(True)
        self.bridge_enabled_check.stateChanged.connect(lambda _: self.apply_bridge_settings())
        bridge_layout.addWidget(self.bridge_enabled_check, 0, 0, 1, 4)

        self.bridge_inputs = {}
        bridge_specs = [
            ("X Min", "x_min", self.bridge_no_go_zone.x_min),
            ("X Max", "x_max", self.bridge_no_go_zone.x_max),
            ("Y Min", "y_min", self.bridge_no_go_zone.y_min),
            ("Y Max", "y_max", self.bridge_no_go_zone.y_max),
            ("Z Min", "z_min", self.bridge_no_go_zone.z_min),
            ("Z Max", "z_max", self.bridge_no_go_zone.z_max),
            ("Approach mm", "approach_margin", self.bridge_approach_margin_mm),
            ("Clearance mm", "clearance", self.bridge_vertical_clearance_mm),
            ("Grip Min mm", "grip_min", self.bridge_gripper_min_mm),
            ("Grip Max mm", "grip_max", self.bridge_gripper_max_mm),
        ]
        for i, (label, key, default) in enumerate(bridge_specs, start=1):
            lbl = QLabel(label)
            spin = QDoubleSpinBox()
            if key in ("grip_min", "grip_max"):
                spin.setRange(0, 50)
            elif key in ("approach_margin", "clearance"):
                spin.setRange(0, 300)
            elif key.startswith("z_"):
                spin.setRange(0, 450)
            else:
                spin.setRange(-450, 450)
            spin.setValue(float(default))
            self.bridge_inputs[key] = spin
            row = i
            bridge_layout.addWidget(lbl, row, 0)
            bridge_layout.addWidget(spin, row, 1)

        self.btn_apply_bridge = QPushButton("Apply Bridge Settings")
        self.btn_apply_bridge.clicked.connect(self.apply_bridge_settings)
        bridge_layout.addWidget(self.btn_apply_bridge, len(bridge_specs) + 1, 0, 1, 2)

        self.bridge_status = QLabel("Bridge settings active")
        self.bridge_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        bridge_layout.addWidget(self.bridge_status, len(bridge_specs) + 2, 0, 1, 4)

        control_layout.addWidget(bridge_group)

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
        self.apply_bridge_settings()
        self._sync_gripper_ui()

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
            # Current joint state from start pose
            self.start_q = np.array(IK(cx, cy, cz, cp))
            self.current_q_anim = self.start_q.copy()

            if self._is_bridge_enabled():
                planned_waypoints = plan_bridge_safe_waypoints(
                    self.start_pose_mp,
                    self.target_pose_mp,
                    self.bridge_no_go_zone,
                    zones=self.bridge_zones,
                    approach_margin_mm=self.bridge_approach_margin_mm,
                    vertical_clearance_mm=self.bridge_vertical_clearance_mm,
                )
            else:
                planned_waypoints = [self.target_pose_mp]
            self.motion_waypoints = [np.array(wp, dtype=float) for wp in planned_waypoints]
            self.motion_segment_idx = 0
            self.bridge_route_active = len(self.motion_waypoints) > 1

            self.dt = 0.05  # 50ms steps
            self.is_moving = True
            self.btn_move.setEnabled(False)
            self._configure_motion_segment(
                self.start_pose_mp,
                self.motion_waypoints[self.motion_segment_idx],
                use_current_q=True,
            )
            if self.bridge_route_active:
                total = len(self.motion_waypoints)
                self.motion_status.setText(f"Bridge-safe route: segment 1/{total}")
            else:
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

    def _is_bridge_enabled(self):
        return hasattr(self, 'bridge_enabled_check') and self.bridge_enabled_check.isChecked()

    def apply_bridge_settings(self):
        try:
            x_min = self.bridge_inputs["x_min"].value()
            x_max = self.bridge_inputs["x_max"].value()
            y_min = self.bridge_inputs["y_min"].value()
            y_max = self.bridge_inputs["y_max"].value()
            z_min = self.bridge_inputs["z_min"].value()
            z_max = self.bridge_inputs["z_max"].value()
            if x_min > x_max:
                x_min, x_max = x_max, x_min
            if y_min > y_max:
                y_min, y_max = y_max, y_min
            if z_min > z_max:
                z_min, z_max = z_max, z_min

            self.bridge_no_go_zone = BridgeNoGoZone(
                x_min=float(x_min), x_max=float(x_max),
                y_min=float(y_min), y_max=float(y_max),
                z_min=float(z_min), z_max=float(z_max),
            )
            self.bridge_zones = build_bridge_zones(
                self.bridge_no_go_zone,
                gap_y=self.bridge_gap_y,
                pillar_width_y=self.bridge_pillar_width_y
            )
            self.bridge_approach_margin_mm = max(0.0, float(self.bridge_inputs["approach_margin"].value()))
            self.bridge_vertical_clearance_mm = max(0.0, float(self.bridge_inputs["clearance"].value()))

            grip_min = float(self.bridge_inputs["grip_min"].value())
            grip_max = float(self.bridge_inputs["grip_max"].value())
            grip_min = float(np.clip(grip_min, 0.0, 50.0))
            grip_max = float(np.clip(grip_max, 0.0, 50.0))
            if grip_min > grip_max:
                grip_min, grip_max = grip_max, grip_min
            self.bridge_gripper_min_mm = grip_min
            self.bridge_gripper_max_mm = grip_max

            # Push normalized values back to inputs
            self.bridge_inputs["x_min"].setValue(self.bridge_no_go_zone.x_min)
            self.bridge_inputs["x_max"].setValue(self.bridge_no_go_zone.x_max)
            self.bridge_inputs["y_min"].setValue(self.bridge_no_go_zone.y_min)
            self.bridge_inputs["y_max"].setValue(self.bridge_no_go_zone.y_max)
            self.bridge_inputs["z_min"].setValue(self.bridge_no_go_zone.z_min)
            self.bridge_inputs["z_max"].setValue(self.bridge_no_go_zone.z_max)
            self.bridge_inputs["approach_margin"].setValue(self.bridge_approach_margin_mm)
            self.bridge_inputs["clearance"].setValue(self.bridge_vertical_clearance_mm)
            self.bridge_inputs["grip_min"].setValue(self.bridge_gripper_min_mm)
            self.bridge_inputs["grip_max"].setValue(self.bridge_gripper_max_mm)

            state = "ON" if self._is_bridge_enabled() else "OFF"
            self.bridge_status.setText(
                f"Bridge[{state}] x=({self.bridge_no_go_zone.x_min:.0f},{self.bridge_no_go_zone.x_max:.0f}) "
                f"y=({self.bridge_no_go_zone.y_min:.0f},{self.bridge_no_go_zone.y_max:.0f}) "
                f"z=({self.bridge_no_go_zone.z_min:.0f},{self.bridge_no_go_zone.z_max:.0f}) "
                f"grip=({self.bridge_gripper_min_mm:.0f}-{self.bridge_gripper_max_mm:.0f})"
            )
            
            # Update 3D visualization
            if self._is_bridge_enabled():
                self.renderer.set_bridge_zone(
                    self.bridge_no_go_zone,
                    gap_y=self.bridge_gap_y,
                    pillar_width_y=self.bridge_pillar_width_y
                )
            else:
                self.renderer.set_bridge_zone(None)
                
            self._sync_gripper_ui()
        except Exception as e:
            self.bridge_status.setText(f"Bridge settings error: {e}")

    def _configure_motion_segment(self, start_pose, target_pose, use_current_q=False):
        self.start_pose_mp = np.array(start_pose, dtype=float)
        self.target_pose_mp = np.array(target_pose, dtype=float)

        if use_current_q:
            self.start_q = self.current_q_anim.copy()
        else:
            self.start_q = np.array(IK(
                self.start_pose_mp[0],
                self.start_pose_mp[1],
                self.start_pose_mp[2],
                self.start_pose_mp[3]
            ))
            self.current_q_anim = self.start_q.copy()

        self.target_q = np.array(IK(
            self.target_pose_mp[0],
            self.target_pose_mp[1],
            self.target_pose_mp[2],
            self.target_pose_mp[3]
        ))

        self.anim_time = 0.0
        self.jac_final_phase = False
        self.jac_final_time = 0.0
        self.jac_final_duration = 0.0
        self.jac_final_start_q = None
        self.jac_final_target_q = None

        velocity = max(float(self.vel_spin.value()), 1.0)  # mm/s
        ang_velocity = 45.0  # deg/s
        dist_lin = np.linalg.norm(self.target_pose_mp[:3] - self.start_pose_mp[:3])
        dist_rot = abs(self.target_pose_mp[3] - self.start_pose_mp[3])
        self.duration = max(dist_lin / velocity, dist_rot / ang_velocity, 0.1)

    def _current_pose_from_q(self, q):
        T_ee, _ = FK(q)
        return np.array([
            T_ee[0, 3],
            T_ee[1, 3],
            T_ee[2, 3],
            -(q[1] + q[2] + q[3]),
        ], dtype=float)

    def _advance_motion_segment_or_finish(self, final_message):
        next_idx = self.motion_segment_idx + 1
        if next_idx < len(self.motion_waypoints):
            self.motion_segment_idx = next_idx
            start_pose = self._current_pose_from_q(self.current_q_anim)
            self._configure_motion_segment(
                start_pose,
                self.motion_waypoints[self.motion_segment_idx],
                use_current_q=True,
            )
            if self.bridge_route_active:
                total = len(self.motion_waypoints)
                self.motion_status.setText(
                    f"Bridge-safe route: segment {self.motion_segment_idx + 1}/{total}"
                )
            else:
                self.motion_status.setText("Moving...")
            return

        if self.bridge_route_active and "Bridge-safe route" not in final_message:
            final_message = final_message.rstrip(".") + " (Bridge-safe route)."
        self._finish_motion(final_message)

    def _finish_motion(self, status_text):
        self.is_moving = False
        self.timer.stop()
        self.btn_move.setEnabled(True)
        self.motion_status.setText(status_text)
        self.safety_bypass = False

    def _check_bridge_collision(self, global_transforms):
        if not self._is_bridge_enabled():
            return False, "", np.zeros(3, dtype=float)
            
        links_to_check = [
            ("Base to Shoulder", 0),
            ("Upper Arm", 1),
            ("Forearm", 2),
            ("Wrist", 3),
        ]
        
        for label, idx in links_to_check:
            if idx + 1 >= len(global_transforms):
                continue
                
            p_start = np.array(global_transforms[idx])[:3, 3]
            p_end = np.array(global_transforms[idx+1])[:3, 3]
            
            # Use 10 samples along each link segment
            from visualization.kinematics.BridgeAvoidance import segment_intersects_no_go_zone
            collision_zones = self.bridge_zones if hasattr(self, 'bridge_zones') else [self.bridge_no_go_zone]
            if segment_intersects_no_go_zone(p_start, p_end, collision_zones, samples=10):
                # Return the midpoint of the link as the collision point for logging
                p_mid = (p_start + p_end) / 2.0
                return True, label, p_mid
                
        return False, "", np.zeros(3, dtype=float)

    def stop_motion(self):
        self._finish_motion("Stopped.")

    def update_motion(self):
        if not self.is_moving:
            return
            
        self.anim_time += self.dt
        t_normalized = self.anim_time / self.duration
        if t_normalized > 1.0:
            t_normalized = 1.0
            
        mode = self.mode_group.checkedId()
        # Force Task Interpolation (straight line) if following a bridge-safe detour
        # The bridge detour planner only guarantees safety for linear task-space paths!
        if self.bridge_route_active:
            mode = 1
        
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
                        self.current_q_anim = new_q
                        self._advance_motion_segment_or_finish("Target Reached (Jac Hybrid).")
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
                        self._finish_motion("Jacobian Singularity.")
                        return
            
            # --- SAFETY CHECK: Z Floor ---
            if new_q is not None:
                T_check, tf_check = FK(new_q)
                z_check = T_check[2, 3]
                
                Z_LIMIT = 10.0
                
                if z_check < Z_LIMIT and not self.safety_bypass:
                    if z_check > (z_prev + 0.01): 
                        pass 
                    else:
                        self._finish_motion(f"SAFETY STOP: Z ({z_check:.1f}) < {Z_LIMIT}mm!")
                        return

                if not self.safety_bypass:
                    is_collision, part_name, p_xyz = self._check_bridge_collision(tf_check)
                    if is_collision:
                        self._finish_motion(
                            f"SAFETY STOP: {part_name} entered bridge no-go zone "
                            f"at [{p_xyz[0]:.1f}, {p_xyz[1]:.1f}, {p_xyz[2]:.1f}]"
                        )
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
                self._advance_motion_segment_or_finish("Target Reached.")
                
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

    def _enforce_bridge_gripper_band(self):
        if not self._is_bridge_enabled():
            return
        jaw_mm = self.gripper.jaw_width_mm
        clamped = clamp_gripper_width_for_bridge(
            jaw_mm,
            min_width_mm=self.bridge_gripper_min_mm,
            max_width_mm=self.bridge_gripper_max_mm,
        )
        if abs(clamped - jaw_mm) > 1e-6:
            self.gripper.set_jaw_width_mm(clamped)

    def _sync_gripper_ui(self):
        """Push gripper state to slider, status label, and 3D renderer."""
        self._enforce_bridge_gripper_band()
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
            f"Force: {state['force_pct']:.0f}% | "
            f"Bridge Jaw Band: {self.bridge_gripper_min_mm:.0f}-{self.bridge_gripper_max_mm:.0f}mm"
        )
        # Update 3D jaws
        self.renderer.update_gripper(state['jaw_width_mm'])

    def closeEvent(self, event):
        print("Closing Application...")
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
