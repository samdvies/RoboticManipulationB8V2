import os
import sys
import numpy as np

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(CURRENT_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)
from pyvistaqt import QtInteractor

from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.BridgeAvoidance import (
    BridgeNoGoZone,
    build_bridge_zones,
    clamp_gripper_width_for_bridge,
    plan_bridge_safe_waypoints,
    segment_intersects_no_go_zone,
    solve_optimal_pitch,
)
from visualization.kinematics.FK import FK
from visualization.kinematics.GripperSim import GripperSim
from visualization.kinematics.IK import IK
from visualization.kinematics.Jacobian import get_jacobian


class BridgePickTestWindow(QMainWindow):
    MODE_CONTINUOUS = 0
    MODE_JACOBIAN = 1

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Bridge Pick Solver Test")
        self.setGeometry(80, 80, 1320, 860)

        self.HOME_POSE = np.array([200.0, 0.0, 180.0, 0.0], dtype=float)
        self.DEFAULT_PICK_POSE = np.array([225.0, 0.0, 30.0, 0.0], dtype=float)
        self.safety_bypass = False

        self.bridge_no_go_zone = BridgeNoGoZone(
            x_min=220.0, x_max=230.0,
            y_min=-35.0, y_max=35.0,
            z_min=90.0, z_max=105.0,
        )
        self.bridge_pillar_width_y = 10.0
        self.bridge_gap_y = 50.0
        self.bridge_zones = build_bridge_zones(
            self.bridge_no_go_zone,
            gap_y=self.bridge_gap_y,
            pillar_width_y=self.bridge_pillar_width_y,
        )
        self.bridge_approach_margin_mm = 60.0
        self.bridge_vertical_clearance_mm = 30.0
        self.bridge_gripper_min_mm = 25.0
        self.bridge_gripper_max_mm = 40.0

        self.motion_waypoints = []
        self.motion_segment_idx = 0
        self.bridge_route_active = False
        self.is_moving = False
        self.dt = 0.05
        self.duration = 0.1
        self.anim_time = 0.0
        self._last_solved_pitch = float(self.HOME_POSE[3])
        self.current_q = np.array(IK(*self.HOME_POSE), dtype=float)
        self.current_q_anim = self.current_q.copy()
        self.user_target_pose = self.HOME_POSE.copy()

        self.jac_final_phase = False
        self.jac_final_time = 0.0
        self.jac_final_duration = 0.0
        self.jac_final_start_q = None
        self.jac_final_target_q = None

        self.gripper = GripperSim()

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(390)
        panel = QWidget()
        self.panel_layout = QVBoxLayout(panel)
        scroll.setWidget(panel)
        main_layout.addWidget(scroll)

        self._build_target_group()
        self._build_solver_group()
        self._build_bridge_group()
        self._build_motion_group()
        self._build_gripper_group()
        self._build_pose_group()
        self._build_view_group()
        self.panel_layout.addStretch()

        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor)
        self.renderer = RobotRenderer(self.plotter)
        self.plotter.view_isometric()
        self.plotter.camera.azimuth += 45

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_motion)

        self.apply_bridge_settings()
        self._apply_q_state(self.current_q)
        self._sync_gripper_ui()
        self._refresh_solver_state(reason_override="Ready.")

    def _make_spin(self, min_val, max_val, value, step=1.0):
        spin = QDoubleSpinBox()
        spin.setRange(float(min_val), float(max_val))
        spin.setSingleStep(float(step))
        spin.setValue(float(value))
        return spin

    def _build_target_group(self):
        group = QGroupBox("Bridge Pick Target")
        layout = QGridLayout()
        group.setLayout(layout)

        self.target_inputs = {}
        specs = [
            ("X (mm)", "x", 50.0, 350.0, self.DEFAULT_PICK_POSE[0]),
            ("Y (mm)", "y", -120.0, 120.0, self.DEFAULT_PICK_POSE[1]),
            ("Z (mm)", "z", 5.0, 220.0, self.DEFAULT_PICK_POSE[2]),
            ("Pitch (deg)", "pitch", -90.0, 45.0, self.DEFAULT_PICK_POSE[3]),
        ]
        for row, (label, key, lo, hi, default) in enumerate(specs):
            layout.addWidget(QLabel(label), row, 0)
            spin = self._make_spin(lo, hi, default, step=1.0)
            self.target_inputs[key] = spin
            layout.addWidget(spin, row, 1)

        btn_defaults = QPushButton("Load Under-Bridge Pick Default")
        btn_defaults.clicked.connect(self.load_bridge_pick_default_target)
        layout.addWidget(btn_defaults, len(specs), 0, 1, 2)

        self.panel_layout.addWidget(group)

    def _build_solver_group(self):
        group = QGroupBox("Solver")
        layout = QVBoxLayout()
        group.setLayout(layout)

        self.mode_group = QButtonGroup(self)
        self.radio_continuous = QRadioButton(
            "Continuous Pose Solver (Task + dynamic pitch)"
        )
        self.radio_jacobian = QRadioButton("Jacobian Hybrid (resolved-rate + final IK)")
        self.radio_continuous.setChecked(True)
        self.mode_group.addButton(self.radio_continuous, self.MODE_CONTINUOUS)
        self.mode_group.addButton(self.radio_jacobian, self.MODE_JACOBIAN)

        self.force_continuous_on_route = QCheckBox(
            "Force continuous solver when bridge route has waypoints"
        )
        self.force_continuous_on_route.setChecked(True)

        layout.addWidget(self.radio_continuous)
        layout.addWidget(self.radio_jacobian)
        layout.addWidget(self.force_continuous_on_route)

        self.solver_requested_label = QLabel("Requested: -")
        self.solver_active_label = QLabel("Active: -")
        self.solver_reason_label = QLabel("Reason: -")
        self.solver_pitch_label = QLabel("Solved Pitch: -")
        self.solver_reason_label.setWordWrap(True)
        for lbl in (
            self.solver_requested_label,
            self.solver_active_label,
            self.solver_reason_label,
            self.solver_pitch_label,
        ):
            lbl.setStyleSheet("font-family: monospace; font-size: 10px;")
            layout.addWidget(lbl)

        self.radio_continuous.toggled.connect(lambda _: self._refresh_solver_state())
        self.radio_jacobian.toggled.connect(lambda _: self._refresh_solver_state())
        self.force_continuous_on_route.stateChanged.connect(lambda _: self._refresh_solver_state())

        self.panel_layout.addWidget(group)

    def _build_bridge_group(self):
        group = QGroupBox("Bridge Route")
        layout = QGridLayout()
        group.setLayout(layout)

        self.bridge_enabled_check = QCheckBox("Enable bridge no-go route planner")
        self.bridge_enabled_check.setChecked(True)
        self.bridge_enabled_check.stateChanged.connect(lambda _: self.apply_bridge_settings())
        layout.addWidget(self.bridge_enabled_check, 0, 0, 1, 2)

        layout.addWidget(QLabel("Approach margin (mm)"), 1, 0)
        self.bridge_approach_spin = self._make_spin(0.0, 200.0, self.bridge_approach_margin_mm)
        layout.addWidget(self.bridge_approach_spin, 1, 1)

        layout.addWidget(QLabel("Vertical clearance (mm)"), 2, 0)
        self.bridge_clearance_spin = self._make_spin(0.0, 200.0, self.bridge_vertical_clearance_mm)
        layout.addWidget(self.bridge_clearance_spin, 2, 1)

        btn_apply = QPushButton("Apply Bridge Settings")
        btn_apply.clicked.connect(self.apply_bridge_settings)
        layout.addWidget(btn_apply, 3, 0, 1, 2)

        self.bridge_status = QLabel("Bridge status: -")
        self.bridge_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        self.bridge_status.setWordWrap(True)
        layout.addWidget(self.bridge_status, 4, 0, 1, 2)

        self.panel_layout.addWidget(group)

    def _build_motion_group(self):
        group = QGroupBox("Motion")
        layout = QGridLayout()
        group.setLayout(layout)

        layout.addWidget(QLabel("Velocity (mm/s)"), 0, 0)
        self.vel_spin = self._make_spin(1.0, 200.0, 40.0, step=1.0)
        layout.addWidget(self.vel_spin, 0, 1)

        self.btn_move = QPushButton("Move To Target")
        self.btn_move.clicked.connect(self.start_motion_to_target)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.clicked.connect(self.stop_motion)
        self.btn_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_home = QPushButton("Move Home (Bypass)")
        self.btn_home.clicked.connect(self.move_to_home)
        self.btn_snap_home = QPushButton("Snap To Home (No Motion)")
        self.btn_snap_home.clicked.connect(self.snap_to_home_pose)

        layout.addWidget(self.btn_move, 1, 0, 1, 2)
        layout.addWidget(self.btn_stop, 2, 0, 1, 2)
        layout.addWidget(self.btn_home, 3, 0, 1, 2)
        layout.addWidget(self.btn_snap_home, 4, 0, 1, 2)

        self.route_status = QLabel("Route: idle")
        self.motion_status = QLabel("Ready.")
        self.route_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        self.motion_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        self.motion_status.setWordWrap(True)
        layout.addWidget(self.route_status, 5, 0, 1, 2)
        layout.addWidget(self.motion_status, 6, 0, 1, 2)

        self.panel_layout.addWidget(group)

    def _build_gripper_group(self):
        group = QGroupBox("Gripper")
        layout = QGridLayout()
        group.setLayout(layout)

        layout.addWidget(QLabel("Jaw width (mm)"), 0, 0)
        self.jaw_spin = self._make_spin(0.0, 40.0, 40.0, step=1.0)
        layout.addWidget(self.jaw_spin, 0, 1)

        btn_apply = QPushButton("Apply Jaw Width")
        btn_apply.clicked.connect(self.apply_jaw_width)
        btn_open = QPushButton("Open")
        btn_open.clicked.connect(self.gripper_open)
        btn_half = QPushButton("Half (20mm)")
        btn_half.clicked.connect(lambda: self.set_gripper_jaw_mm(20.0))
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.gripper_close)

        layout.addWidget(btn_apply, 1, 0, 1, 2)
        layout.addWidget(btn_open, 2, 0)
        layout.addWidget(btn_half, 2, 1)
        layout.addWidget(btn_close, 3, 0, 1, 2)

        self.gripper_status = QLabel("Gripper: -")
        self.gripper_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        self.gripper_status.setWordWrap(True)
        layout.addWidget(self.gripper_status, 4, 0, 1, 2)

        self.panel_layout.addWidget(group)

    def _build_pose_group(self):
        group = QGroupBox("Current Pose")
        layout = QVBoxLayout()
        group.setLayout(layout)

        self.pose_status = QLabel("EE: -")
        self.joint_status = QLabel("Joints: -")
        self.target_compare_status = QLabel("Target vs Actual: -")
        self.pose_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        self.joint_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        self.target_compare_status.setStyleSheet("font-family: monospace; font-size: 10px;")
        self.pose_status.setWordWrap(True)
        self.joint_status.setWordWrap(True)
        self.target_compare_status.setWordWrap(True)
        layout.addWidget(self.pose_status)
        layout.addWidget(self.joint_status)
        layout.addWidget(self.target_compare_status)

        self.panel_layout.addWidget(group)

    def _build_view_group(self):
        group = QGroupBox("View")
        layout = QGridLayout()
        group.setLayout(layout)

        btn_iso = QPushButton("ISO")
        btn_top = QPushButton("Top")
        btn_front = QPushButton("Front")
        btn_side = QPushButton("Side")
        btn_iso.clicked.connect(lambda: self.plotter.view_isometric())
        btn_top.clicked.connect(lambda: self.plotter.view_xy())
        btn_front.clicked.connect(lambda: self.plotter.view_yz())
        btn_side.clicked.connect(lambda: self.plotter.view_xz())

        layout.addWidget(btn_iso, 0, 0)
        layout.addWidget(btn_top, 0, 1)
        layout.addWidget(btn_front, 1, 0)
        layout.addWidget(btn_side, 1, 1)

        self.panel_layout.addWidget(group)

    def _is_bridge_enabled(self):
        return self.bridge_enabled_check.isChecked()

    def apply_bridge_settings(self):
        self.bridge_approach_margin_mm = float(self.bridge_approach_spin.value())
        self.bridge_vertical_clearance_mm = float(self.bridge_clearance_spin.value())

        if self._is_bridge_enabled():
            self.renderer.set_bridge_zone(
                self.bridge_no_go_zone,
                gap_y=self.bridge_gap_y,
                pillar_width_y=self.bridge_pillar_width_y,
            )
            state = "ON"
        else:
            self.renderer.set_bridge_zone(None)
            state = "OFF"

        self.bridge_status.setText(
            f"Bridge[{state}] x=({self.bridge_no_go_zone.x_min:.0f},{self.bridge_no_go_zone.x_max:.0f}) "
            f"y=({self.bridge_no_go_zone.y_min:.0f},{self.bridge_no_go_zone.y_max:.0f}) "
            f"z=({self.bridge_no_go_zone.z_min:.0f},{self.bridge_no_go_zone.z_max:.0f}) "
            f"| approach={self.bridge_approach_margin_mm:.0f} | clearance={self.bridge_vertical_clearance_mm:.0f}"
        )
        self._sync_gripper_ui()

    def load_bridge_pick_default_target(self):
        self.target_inputs["x"].setValue(float(self.DEFAULT_PICK_POSE[0]))
        self.target_inputs["y"].setValue(float(self.DEFAULT_PICK_POSE[1]))
        self.target_inputs["z"].setValue(float(self.DEFAULT_PICK_POSE[2]))
        self.target_inputs["pitch"].setValue(float(self.DEFAULT_PICK_POSE[3]))
        self.radio_continuous.setChecked(True)
        self.force_continuous_on_route.setChecked(True)
        self._refresh_solver_state(reason_override="Loaded under-bridge default target.")

    def snap_to_home_pose(self):
        self.stop_motion()
        self._apply_pose(self.HOME_POSE)
        self._refresh_solver_state(reason_override="Snapped to HOME.")
        self.motion_status.setText("Snapped to HOME pose.")

    def _mode_name(self, mode_id):
        if mode_id == self.MODE_CONTINUOUS:
            return "Continuous Pose Solver"
        if mode_id == self.MODE_JACOBIAN:
            return "Jacobian Hybrid"
        return "Unknown"

    def _resolve_active_mode(self):
        requested_mode = self.mode_group.checkedId()
        active_mode = requested_mode
        reason = "Requested solver is active."

        if (
            self.bridge_route_active
            and self._is_bridge_enabled()
            and self.force_continuous_on_route.isChecked()
        ):
            active_mode = self.MODE_CONTINUOUS
            if requested_mode != active_mode:
                reason = "Bridge route override: continuous solver forced."
            else:
                reason = "Bridge route active: continuous solver selected."
        elif self.bridge_route_active and requested_mode == self.MODE_JACOBIAN:
            reason = "Bridge route active with Jacobian (path safety not guaranteed)."

        return requested_mode, active_mode, reason

    def _refresh_solver_state(self, reason_override=None):
        requested_mode, active_mode, reason = self._resolve_active_mode()
        if reason_override is not None:
            reason = reason_override

        self.solver_requested_label.setText(f"Requested: {self._mode_name(requested_mode)}")
        self.solver_active_label.setText(f"Active:    {self._mode_name(active_mode)}")
        self.solver_reason_label.setText(f"Reason:    {reason}")
        self.solver_pitch_label.setText(f"Solved Pitch: {self._last_solved_pitch:.1f} deg")

        mismatch = requested_mode != active_mode
        color = "orange" if mismatch else "#4CAF50"
        self.solver_active_label.setStyleSheet(
            f"font-family: monospace; font-size: 10px; color: {color}; font-weight: bold;"
        )

    def _refresh_route_status(self):
        if not self.motion_waypoints:
            self.route_status.setText("Route: idle")
            return

        total = len(self.motion_waypoints)
        segment = min(self.motion_segment_idx + 1, total)
        route_type = "bridge-safe waypoints" if self.bridge_route_active else "direct"
        self.route_status.setText(f"Route: {route_type} | segment {segment}/{total}")

    def _apply_pose(self, pose4):
        q = np.array(IK(float(pose4[0]), float(pose4[1]), float(pose4[2]), float(pose4[3])), dtype=float)
        self._apply_q_state(q)

    def _apply_q_state(self, q):
        self.current_q = np.array(q, dtype=float)
        self.current_q_anim = self.current_q.copy()
        T_ee, transforms = FK(self.current_q)
        self.renderer.update_actors(transforms)
        self._update_pose_labels(T_ee, self.current_q)
        self._update_target_compare_status(T_ee, self.current_q)

    def _update_pose_labels(self, T_ee, q):
        x = T_ee[0, 3]
        y = T_ee[1, 3]
        z = T_ee[2, 3]
        pitch = -(q[1] + q[2] + q[3])
        self.pose_status.setText(f"EE: x={x:.1f}  y={y:.1f}  z={z:.1f}  pitch={pitch:.1f}")
        self.joint_status.setText(f"Joints: q1={q[0]:.1f}  q2={q[1]:.1f}  q3={q[2]:.1f}  q4={q[3]:.1f}")

    def _update_target_compare_status(self, T_ee, q):
        if T_ee is None or q is None:
            return
        actual_x = float(T_ee[0, 3])
        actual_y = float(T_ee[1, 3])
        actual_z = float(T_ee[2, 3])
        actual_pitch = float(-(q[1] + q[2] + q[3]))

        target = np.array(self.user_target_pose, dtype=float)
        dx = actual_x - target[0]
        dy = actual_y - target[1]
        dz = actual_z - target[2]
        dp = actual_pitch - target[3]
        pos_err = float(np.linalg.norm([dx, dy, dz]))

        ok = (pos_err <= 3.0) and (abs(dp) <= 3.0)
        color = "#4CAF50" if ok else "orange"
        self.target_compare_status.setStyleSheet(
            f"font-family: monospace; font-size: 10px; color: {color};"
        )
        self.target_compare_status.setText(
            "Target vs Actual: "
            f"T=({target[0]:.1f},{target[1]:.1f},{target[2]:.1f},{target[3]:.1f}) "
            f"A=({actual_x:.1f},{actual_y:.1f},{actual_z:.1f},{actual_pitch:.1f}) "
            f"Err[pos={pos_err:.1f}mm, pitch={dp:.1f}deg]"
        )

    def _current_pose_from_q(self, q):
        T_ee, _ = FK(q)
        return np.array(
            [T_ee[0, 3], T_ee[1, 3], T_ee[2, 3], -(q[1] + q[2] + q[3])],
            dtype=float,
        )

    def start_motion_to_target(self):
        self.start_motion(safety_bypass=False)

    def move_to_home(self):
        self.target_inputs["x"].setValue(float(self.HOME_POSE[0]))
        self.target_inputs["y"].setValue(float(self.HOME_POSE[1]))
        self.target_inputs["z"].setValue(float(self.HOME_POSE[2]))
        self.target_inputs["pitch"].setValue(float(self.HOME_POSE[3]))
        self.start_motion(safety_bypass=True)

    def start_motion(self, safety_bypass=False):
        if self.is_moving:
            return

        self.safety_bypass = bool(safety_bypass)
        target_pose = np.array(
            [
                self.target_inputs["x"].value(),
                self.target_inputs["y"].value(),
                self.target_inputs["z"].value(),
                self.target_inputs["pitch"].value(),
            ],
            dtype=float,
        )
        self.user_target_pose = target_pose.copy()
        start_pose = self._current_pose_from_q(self.current_q)

        try:
            self.start_q = self.current_q.copy()
            self.current_q_anim = self.start_q.copy()

            if self._is_bridge_enabled():
                planned_waypoints = plan_bridge_safe_waypoints(
                    start_pose,
                    target_pose,
                    self.bridge_no_go_zone,
                    zones=self.bridge_zones,
                    approach_margin_mm=self.bridge_approach_margin_mm,
                    vertical_clearance_mm=self.bridge_vertical_clearance_mm,
                )
            else:
                planned_waypoints = [target_pose]

            self.motion_waypoints = [np.array(wp, dtype=float) for wp in planned_waypoints]
            self.motion_segment_idx = 0
            self.bridge_route_active = len(self.motion_waypoints) > 1
            self.is_moving = True
            self.btn_move.setEnabled(False)

            self._configure_motion_segment(
                start_pose,
                self.motion_waypoints[self.motion_segment_idx],
                use_current_q=True,
            )
            self._refresh_route_status()
            self._refresh_solver_state()
            wp_preview = " -> ".join(
                f"({wp[0]:.0f},{wp[1]:.0f},{wp[2]:.0f},{wp[3]:.0f})"
                for wp in self.motion_waypoints[:4]
            )
            if len(self.motion_waypoints) > 4:
                wp_preview += " -> ..."
            self.motion_status.setText(f"Motion started. Route: {wp_preview}")
            self.timer.start(int(self.dt * 1000))
        except Exception as exc:
            self.motion_status.setText(f"Error: {exc}")
            self.is_moving = False
            self.btn_move.setEnabled(True)

    def _configure_motion_segment(self, start_pose, target_pose, use_current_q=False):
        self.start_pose_mp = np.array(start_pose, dtype=float)
        self.target_pose_mp = np.array(target_pose, dtype=float)

        if use_current_q:
            self.start_q = self.current_q_anim.copy()
        else:
            self.start_q = np.array(IK(*self.start_pose_mp), dtype=float)
            self.current_q_anim = self.start_q.copy()

        self.target_q = np.array(IK(*self.target_pose_mp), dtype=float)
        self.anim_time = 0.0
        self.jac_final_phase = False
        self.jac_final_time = 0.0
        self.jac_final_duration = 0.0
        self.jac_final_start_q = None
        self.jac_final_target_q = None
        self._last_solved_pitch = float(self.start_pose_mp[3])

        velocity = max(float(self.vel_spin.value()), 1.0)
        ang_velocity = 45.0
        dist_lin = np.linalg.norm(self.target_pose_mp[:3] - self.start_pose_mp[:3])
        dist_rot = abs(self.target_pose_mp[3] - self.start_pose_mp[3])
        self.duration = max(dist_lin / velocity, dist_rot / ang_velocity, 0.1)

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
            self._refresh_route_status()
            self.motion_status.setText("Advancing to next waypoint segment.")
            self._refresh_solver_state()
            return
        self._finish_motion(final_message)

    def _finish_motion(self, status_text):
        self.is_moving = False
        self.timer.stop()
        self.btn_move.setEnabled(True)
        self.motion_status.setText(status_text)
        self.safety_bypass = False
        self.motion_waypoints = []
        self.motion_segment_idx = 0
        self.bridge_route_active = False
        self._refresh_route_status()
        self._refresh_solver_state()

    def stop_motion(self):
        if self.is_moving:
            self._finish_motion("Stopped.")
        else:
            self.motion_status.setText("Already idle.")

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
            p_end = np.array(global_transforms[idx + 1])[:3, 3]
            if segment_intersects_no_go_zone(p_start, p_end, self.bridge_zones, samples=10):
                p_mid = (p_start + p_end) / 2.0
                return True, label, p_mid

        return False, "", np.zeros(3, dtype=float)

    def update_motion(self):
        if not self.is_moving:
            return

        self.anim_time += self.dt
        t_normalized = min(self.anim_time / self.duration, 1.0)

        requested_mode, active_mode, _ = self._resolve_active_mode()
        self._refresh_solver_state()
        mode = active_mode
        new_q = None

        try:
            prev_fk, _ = FK(self.current_q_anim)
            z_prev = prev_fk[2, 3]

            if mode == self.MODE_CONTINUOUS:
                curr_pose_target = self.start_pose_mp + (
                    self.target_pose_mp - self.start_pose_mp
                ) * t_normalized

                if self._is_bridge_enabled():
                    final_target_pose = (
                        self.motion_waypoints[-1]
                        if self.motion_waypoints
                        else self.target_pose_mp
                    )
                    dist_to_final_mm = float(
                        np.linalg.norm(curr_pose_target[:3] - final_target_pose[:3])
                    )
                    # Smooth distance ramp: start steering early, then tighten near target.
                    d_far = 160.0
                    d_near = 25.0
                    u = (d_far - dist_to_final_mm) / max(1e-6, (d_far - d_near))
                    u = float(np.clip(u, 0.0, 1.0))
                    proximity = u * u * (3.0 - 2.0 * u)  # smoothstep

                    dynamic_target_weight = 25.0 + 220.0 * proximity
                    dynamic_max_pitch_rate = 1.8 - 0.9 * proximity
                    desired_pitch = (
                        (1.0 - proximity) * float(self._last_solved_pitch)
                        + proximity * float(self.target_pose_mp[3])
                    )
                    opt_p = solve_optimal_pitch(
                        curr_pose_target[0],
                        curr_pose_target[1],
                        curr_pose_target[2],
                        zones=self.bridge_zones,
                        preferred_pitch=desired_pitch,
                        prev_pitch=self._last_solved_pitch,
                        max_pitch_rate=dynamic_max_pitch_rate,
                        pitch_range=(-90.0, 45.0),
                        terminal_target_pitch=self.target_pose_mp[3],
                        terminal_target_weight=dynamic_target_weight,
                        enforce_terminal_target_if_feasible=False,
                        bridge_proximity_weight=55.0,
                        bridge_proximity_decay_mm=12.0,
                        bridge_x_proximity_weight=95.0,
                        bridge_x_proximity_decay_mm=9.0,
                    )
                    if opt_p is not None:
                        curr_pose_target[3] = float(opt_p)
                        self._last_solved_pitch = float(opt_p)
                    else:
                        curr_pose_target[3] = float(self._last_solved_pitch)

                new_q = np.array(
                    IK(
                        curr_pose_target[0],
                        curr_pose_target[1],
                        curr_pose_target[2],
                        curr_pose_target[3],
                    ),
                    dtype=float,
                )

            elif mode == self.MODE_JACOBIAN:
                if self.jac_final_phase:
                    self.jac_final_time += self.dt
                    s = (
                        self.jac_final_time / self.jac_final_duration
                        if self.jac_final_duration > 1e-9
                        else 1.0
                    )
                    s = float(np.clip(s, 0.0, 1.0))
                    new_q = self.jac_final_start_q + (
                        self.jac_final_target_q - self.jac_final_start_q
                    ) * s
                    if s >= 1.0:
                        self.current_q_anim = new_q
                        self._advance_motion_segment_or_finish(
                            "Target reached (Jacobian hybrid)."
                        )
                        return
                else:
                    current_fk, _ = FK(self.current_q_anim)
                    current_pos = current_fk[:3, 3]
                    curr_q = self.current_q_anim
                    current_pitch = -(curr_q[1] + curr_q[2] + curr_q[3])
                    target_pos = self.target_pose_mp[:3]
                    target_pitch = self.target_pose_mp[3]

                    error_pos = target_pos - current_pos
                    error_pitch = target_pitch - current_pitch

                    if np.linalg.norm(error_pos) < 15.0 and abs(error_pitch) < 8.0:
                        self.jac_final_phase = True
                        self.jac_final_time = 0.0
                        self.jac_final_start_q = self.current_q_anim.copy()
                        self.jac_final_target_q = np.array(
                            IK(
                                self.target_pose_mp[0],
                                self.target_pose_mp[1],
                                self.target_pose_mp[2],
                                self.target_pose_mp[3],
                            ),
                            dtype=float,
                        )
                        max_delta = float(
                            np.max(
                                np.abs(self.jac_final_target_q - self.jac_final_start_q)
                            )
                        )
                        self.jac_final_duration = max(0.12, max_delta / 90.0)
                        self.motion_status.setText("Jacobian final IK handoff...")
                        return

                    vel_mag = float(self.vel_spin.value())
                    v_lin = 2.0 * error_pos
                    v_norm = np.linalg.norm(v_lin)
                    if v_norm > vel_mag and v_norm > 1e-9:
                        v_lin = v_lin * (vel_mag / v_norm)

                    w_pitch_rad = 2.0 * np.deg2rad(error_pitch)
                    w_pitch_rad = np.clip(
                        w_pitch_rad, -np.deg2rad(90.0), np.deg2rad(90.0)
                    )

                    J = get_jacobian(self.current_q_anim)
                    J_pitch_row = np.array([0.0, -1.0, -1.0, -1.0], dtype=float)
                    J_task = np.vstack([J[0:3, :], J_pitch_row])
                    v_task = np.append(v_lin, w_pitch_rad)

                    lambda_val = 0.05
                    J_dls = J_task.T @ np.linalg.inv(
                        J_task @ J_task.T + lambda_val**2 * np.eye(4)
                    )
                    q_dot_rad = J_dls @ v_task
                    q_dot_deg = np.rad2deg(q_dot_rad)
                    q_dot_deg = np.clip(q_dot_deg, -120.0, 120.0)
                    new_q = self.current_q_anim + q_dot_deg * self.dt

            if new_q is not None:
                T_check, transforms_check = FK(new_q)
                z_check = float(T_check[2, 3])
                if z_check < 10.0 and not self.safety_bypass:
                    if z_check <= (z_prev + 0.01):
                        self._finish_motion(f"Safety stop: z {z_check:.1f} < 10.0 mm.")
                        return

                if not self.safety_bypass:
                    is_collision, part_name, p_xyz = self._check_bridge_collision(
                        transforms_check
                    )
                    if is_collision:
                        self._finish_motion(
                            f"Safety stop: {part_name} in bridge zone at "
                            f"[{p_xyz[0]:.1f}, {p_xyz[1]:.1f}, {p_xyz[2]:.1f}]."
                        )
                        return

                self.current_q_anim = np.array(new_q, dtype=float)
                self.current_q = self.current_q_anim.copy()
                T_ee, transforms = FK(self.current_q)
                self.renderer.update_actors(transforms)
                self._update_pose_labels(T_ee, self.current_q)
                self._update_target_compare_status(T_ee, self.current_q)

            if t_normalized >= 1.0 and mode != self.MODE_JACOBIAN:
                self._advance_motion_segment_or_finish("Target reached.")
                return

            if requested_mode != active_mode:
                self.motion_status.setText(
                    "Moving (bridge route override to continuous solver)."
                )
            elif mode == self.MODE_CONTINUOUS:
                self.motion_status.setText("Moving (continuous pose solver).")
            else:
                self.motion_status.setText("Moving (jacobian hybrid).")

        except np.linalg.LinAlgError:
            self._finish_motion("Jacobian singularity.")
        except Exception as exc:
            self._finish_motion(f"Motion error: {exc}")

    def set_gripper_jaw_mm(self, jaw_mm):
        self.gripper.set_jaw_width_mm(float(jaw_mm))
        self._sync_gripper_ui()

    def apply_jaw_width(self):
        self.set_gripper_jaw_mm(self.jaw_spin.value())

    def gripper_open(self):
        self.gripper.open()
        self._sync_gripper_ui()

    def gripper_close(self):
        self.gripper.close()
        self._sync_gripper_ui()

    def _enforce_bridge_gripper_band(self):
        if not self._is_bridge_enabled():
            return
        clamped = clamp_gripper_width_for_bridge(
            self.gripper.jaw_width_mm,
            min_width_mm=self.bridge_gripper_min_mm,
            max_width_mm=self.bridge_gripper_max_mm,
        )
        if abs(clamped - self.gripper.jaw_width_mm) > 1e-6:
            self.gripper.set_jaw_width_mm(clamped)

    def _sync_gripper_ui(self):
        self._enforce_bridge_gripper_band()
        state = self.gripper.get_state()
        self.jaw_spin.blockSignals(True)
        self.jaw_spin.setValue(float(state["jaw_width_mm"]))
        self.jaw_spin.blockSignals(False)

        self.gripper_status.setText(
            f"Mode={state['mode']} | jaw={state['jaw_width_mm']:.1f} mm | "
            f"pct={state['pct']:.0f}% | enc={state['encoder']} | "
            f"bridge_band={self.bridge_gripper_min_mm:.0f}-{self.bridge_gripper_max_mm:.0f} mm"
        )
        self.renderer.update_gripper(float(state["jaw_width_mm"]))

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BridgePickTestWindow()
    window.show()
    sys.exit(app.exec_())
