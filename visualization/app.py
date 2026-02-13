import sys
import os
import numpy as np

# Ensure we can import from the root directory when running this script directly
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QGroupBox, QGridLayout, QPushButton)
from PyQt5.QtCore import Qt, pyqtSignal
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK
from visualization.kinematics.JointLimits import clamp_joints, validate_joints, get_limits, JOINT_NAMES

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Live Simulation (Task Space Control)")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize Python Kinematics
        print("Using Python Kinematics (No MATLAB)")

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
        control_layout.addStretch()

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

    def closeEvent(self, event):
        print("Closing Application...")
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
