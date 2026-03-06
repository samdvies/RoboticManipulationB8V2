#!/usr/bin/env python3
"""
Visualize Task 2.c — Tool Through Gates  (PyQt5 + pyvistaqt)

Animates the OpenManipulator-X through the Task 2c pick-and-place path.
Uses the same PyQt5 / QtInteractor format as app.py.

Usage (from project root):
    python visualization/visualize_task2c.py
"""

import sys, os
import numpy as np

# Allow standalone execution from any directory
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_DIR = os.path.dirname(_THIS_DIR)
if _REPO_DIR not in sys.path:
    sys.path.insert(0, _REPO_DIR)

import pyvista as pv
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QGroupBox, QPushButton,
                             QSlider, QGridLayout)
from PyQt5.QtCore import Qt, QTimer
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.task2c_tool_through_gates import (
    build_task2c_path, GATE_WAYPOINTS_XY, GATE_Z,
    TOOL_PICK_XY, TOOL_GRASP_Z, DROP_XY, DROP_Z,
)


class Task2cWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Task 2.c — Tool Through Gates (Visualization)")
        self.setGeometry(100, 100, 1400, 900)

        # ── Pre-compute path ──────────────────────────────────────────
        self.poses, self.labels = build_task2c_path()
        self.num_poses = len(self.poses)

        self.all_q = []
        self.all_transforms = []
        self.all_fk_pos = []

        for pose in self.poses:
            x, y, z, p = pose
            q = IK(x, y, z, p)
            T_ee, gt = FK(q)
            self.all_q.append(q)
            self.all_transforms.append(gt)
            self.all_fk_pos.append(T_ee[:3, 3].tolist())

        # ── Layout ────────────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # Left panel — controls
        control_panel = QWidget()
        control_panel.setFixedWidth(320)
        control_layout = QVBoxLayout(control_panel)
        main_layout.addWidget(control_panel)

        # ── Status Group ──────────────────────────────────────────────
        status_group = QGroupBox("Current State")
        status_layout = QVBoxLayout()
        status_group.setLayout(status_layout)

        self.lbl_phase = QLabel("Phase: —")
        self.lbl_phase.setStyleSheet("font-weight: bold; font-size: 13px;")
        self.lbl_pose  = QLabel("Pose: 0 / 0")
        self.lbl_xyz   = QLabel("XYZ: (—, —, —)")
        self.lbl_pitch = QLabel("Pitch: —°")
        self.lbl_grip  = QLabel("Gripper: OPEN")
        self.lbl_grip.setStyleSheet("color: green; font-weight: bold;")
        self.lbl_joints = QLabel("Joints: —")
        self.lbl_joints.setStyleSheet("font-family: monospace; font-size: 10px;")

        for w in [self.lbl_phase, self.lbl_pose, self.lbl_xyz, self.lbl_pitch,
                  self.lbl_grip, self.lbl_joints]:
            status_layout.addWidget(w)

        control_layout.addWidget(status_group)

        # ── Playback Group ────────────────────────────────────────────
        play_group = QGroupBox("Playback Controls")
        play_layout = QVBoxLayout()
        play_group.setLayout(play_layout)

        btn_row = QHBoxLayout()
        self.btn_play  = QPushButton("▶ Play")
        self.btn_play.clicked.connect(self.toggle_play)
        self.btn_play.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.btn_reset = QPushButton("⏮ Reset")
        self.btn_reset.clicked.connect(self.reset_anim)
        self.btn_step  = QPushButton("⏭ Step")
        self.btn_step.clicked.connect(self.step_forward)
        btn_row.addWidget(self.btn_play)
        btn_row.addWidget(self.btn_reset)
        btn_row.addWidget(self.btn_step)
        play_layout.addLayout(btn_row)

        # Speed slider
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 15)
        self.speed_slider.setValue(3)
        self.speed_slider.valueChanged.connect(self.update_speed_label)
        speed_row.addWidget(self.speed_slider)
        self.speed_label = QLabel("3x")
        speed_row.addWidget(self.speed_label)
        play_layout.addLayout(speed_row)

        # Scrub slider
        scrub_row = QHBoxLayout()
        scrub_row.addWidget(QLabel("Frame:"))
        self.scrub_slider = QSlider(Qt.Horizontal)
        self.scrub_slider.setRange(0, self.num_poses - 1)
        self.scrub_slider.setValue(0)
        self.scrub_slider.valueChanged.connect(self.scrub_to)
        scrub_row.addWidget(self.scrub_slider)
        play_layout.addLayout(scrub_row)

        control_layout.addWidget(play_group)

        # ── View Group ────────────────────────────────────────────────
        view_group = QGroupBox("Camera Views")
        view_layout = QGridLayout()
        view_group.setLayout(view_layout)

        btn_iso   = QPushButton("ISO View")
        btn_iso.clicked.connect(lambda: self.plotter.view_isometric())
        btn_top   = QPushButton("Top View")
        btn_top.clicked.connect(lambda: self.plotter.view_xy())
        btn_front = QPushButton("Front View")
        btn_front.clicked.connect(lambda: self.plotter.view_yz())
        btn_side  = QPushButton("Side View")
        btn_side.clicked.connect(lambda: self.plotter.view_xz())

        view_layout.addWidget(btn_iso,   0, 0)
        view_layout.addWidget(btn_top,   0, 1)
        view_layout.addWidget(btn_front, 1, 0)
        view_layout.addWidget(btn_side,  1, 1)

        control_layout.addWidget(view_group)

        # ── Legend Group ──────────────────────────────────────────────
        legend_group = QGroupBox("Legend")
        legend_layout = QVBoxLayout()
        legend_group.setLayout(legend_layout)

        for txt, col in [("Orange posts — Gates", "darkorange"),
                         ("Blue line — EE path", "royalblue"),
                         ("Green — Pick location", "green"),
                         ("Red — Drop location", "red"),
                         ("Cyan sphere — Current EE", "darkcyan")]:
            lbl = QLabel(txt)
            lbl.setStyleSheet(f"color: {col}; font-weight: bold;")
            legend_layout.addWidget(lbl)

        control_layout.addWidget(legend_group)
        control_layout.addStretch()

        # ── 3D Plotter ────────────────────────────────────────────────
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor)
        self.plotter.set_background('white', top='lightblue')

        # Robot renderer
        self.renderer = RobotRenderer(self.plotter)

        # Draw scene decorations
        self._draw_gates()
        self._draw_path()
        self._draw_markers()
        self._draw_ground()

        # EE sphere
        self._ee_mesh = pv.Sphere(radius=6, center=self.all_fk_pos[0])
        self._ee_actor = self.plotter.add_mesh(self._ee_mesh, color='cyan', opacity=0.9)

        # Camera
        self.plotter.camera_position = [(500, -500, 400), (100, 0, 50), (0, 0, 1)]

        # ── Animation state ───────────────────────────────────────────
        self.anim_idx = 0
        self.anim_speed = 3
        self.is_playing = False
        self.gripper_open = True

        # QTimer for animation (same pattern as app.py)
        self.timer = QTimer()
        self.timer.timeout.connect(self.advance_frame)

        # Show initial frame
        self._show_frame(0)

    # ── Scene Decoration ──────────────────────────────────────────────

    def _draw_gates(self):
        GATE_WIDTH, GATE_HEIGHT, POST_R = 60, 30, 4
        for i, (gx, gy) in enumerate(GATE_WAYPOINTS_XY):
            half_w = GATE_WIDTH / 2
            left  = pv.Cylinder(radius=POST_R, height=GATE_HEIGHT,
                                center=(gx, gy - half_w, GATE_Z), direction=(0, 0, 1))
            right = pv.Cylinder(radius=POST_R, height=GATE_HEIGHT,
                                center=(gx, gy + half_w, GATE_Z), direction=(0, 0, 1))
            bar   = pv.Cylinder(radius=POST_R * 0.6, height=GATE_WIDTH,
                                center=(gx, gy, GATE_Z + GATE_HEIGHT / 2), direction=(0, 1, 0))
            self.plotter.add_mesh(left,  color='orange', opacity=0.8)
            self.plotter.add_mesh(right, color='orange', opacity=0.8)
            self.plotter.add_mesh(bar,   color='darkorange', opacity=0.6)
            self.plotter.add_point_labels(
                np.array([[gx, gy, GATE_Z + GATE_HEIGHT / 2 + 10]]),
                [f"G{i+1}"], font_size=14, text_color='darkorange',
                point_size=0, shape=None, always_visible=True
            )

    def _draw_path(self):
        pts = np.array(self.all_fk_pos)
        # Use straight line segments (not spline) to show actual linear path
        poly = pv.PolyData(pts)
        poly.lines = np.hstack([
            [len(pts)] + list(range(len(pts)))
        ])
        self.plotter.add_mesh(poly, color='royalblue', line_width=2, opacity=0.5)

    def _draw_markers(self):
        pick_pt = np.array([[TOOL_PICK_XY[0], TOOL_PICK_XY[1], TOOL_GRASP_Z]])
        drop_pt = np.array([[DROP_XY[0], DROP_XY[1], DROP_Z]])
        self.plotter.add_point_labels(pick_pt, ["PICK"], font_size=16, text_color='green',
                                      point_size=15, point_color='green', always_visible=True)
        self.plotter.add_point_labels(drop_pt, ["DROP"], font_size=16, text_color='red',
                                      point_size=15, point_color='red', always_visible=True)

    def _draw_ground(self):
        ground = pv.Plane(center=(100, 0, 0), direction=(0, 0, 1),
                          i_size=500, j_size=500, i_resolution=10, j_resolution=10)
        self.plotter.add_mesh(ground, color='lightgrey', opacity=0.3, style='wireframe')

    # ── Frame Display ─────────────────────────────────────────────────

    def _show_frame(self, idx):
        """Render robot at frame idx and update all UI labels."""
        idx = idx % self.num_poses
        self.anim_idx = idx

        q = self.all_q[idx]
        label = self.labels[idx]
        x, y, z, p = self.poses[idx]

        # Update robot arm
        self.renderer.update_actors(self.all_transforms[idx])

        # Gripper actions
        if label == "close_gripper":
            self.gripper_open = False
        elif label == "open_gripper":
            self.gripper_open = True

        self.renderer.update_gripper(40.0 if self.gripper_open else 0.0)

        # Update EE sphere
        pos = self.all_fk_pos[idx]
        new_sphere = pv.Sphere(radius=6, center=pos)
        self._ee_actor.GetMapper().SetInputData(new_sphere)

        # Update UI labels
        self.lbl_phase.setText(f"Phase: {label}")
        self.lbl_pose.setText(f"Pose: {idx + 1} / {self.num_poses}")
        self.lbl_xyz.setText(f"XYZ: ({x:.1f}, {y:.1f}, {z:.1f})")
        self.lbl_pitch.setText(f"Pitch: {p:.1f}°")

        grip_str = "OPEN" if self.gripper_open else "CLOSED"
        grip_col = "green" if self.gripper_open else "red"
        self.lbl_grip.setText(f"Gripper: {grip_str}")
        self.lbl_grip.setStyleSheet(f"color: {grip_col}; font-weight: bold;")

        self.lbl_joints.setText(
            f"J: [{q[0]:.1f}, {q[1]:.1f}, {q[2]:.1f}, {q[3]:.1f}]°"
        )

        # Sync scrub slider without re-triggering
        self.scrub_slider.blockSignals(True)
        self.scrub_slider.setValue(idx)
        self.scrub_slider.blockSignals(False)

    # ── Playback Controls ─────────────────────────────────────────────

    def toggle_play(self):
        if self.is_playing:
            self.timer.stop()
            self.is_playing = False
            self.btn_play.setText("▶ Play")
        else:
            self.is_playing = True
            self.btn_play.setText("⏸ Pause")
            self.timer.start(50)  # 20 FPS

    def reset_anim(self):
        self.timer.stop()
        self.is_playing = False
        self.btn_play.setText("▶ Play")
        self.gripper_open = True
        self._show_frame(0)

    def step_forward(self):
        self._show_frame(self.anim_idx + 1)

    def advance_frame(self):
        """Called by QTimer — advance by speed steps."""
        self._show_frame(self.anim_idx + self.anim_speed)

    def scrub_to(self, value):
        """Scrub slider changed — jump to that frame."""
        # Reset gripper state by replaying from start
        self.gripper_open = True
        for i in range(value + 1):
            lbl = self.labels[i]
            if lbl == "close_gripper":
                self.gripper_open = False
            elif lbl == "open_gripper":
                self.gripper_open = True
        self._show_frame(value)

    def update_speed_label(self, val):
        self.anim_speed = val
        self.speed_label.setText(f"{val}x")

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


# ── Entry Point ───────────────────────────────────────────────────────

if __name__ == '__main__':
    print("Pre-computing Task 2.c path...")
    app = QApplication(sys.argv)
    window = Task2cWindow()
    window.show()
    print("Window ready.")
    sys.exit(app.exec_())
