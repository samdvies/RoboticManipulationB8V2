import sys
import os
import time
import numpy as np

# Ensure import paths
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel, QTextEdit
from PyQt5.QtCore import QTimer
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK

class MovementDebugger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Movement Debugger")
        self.setGeometry(100, 100, 1000, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Text/Status Area
        self.status_label = QLabel("Initializing...")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold; color: blue;")
        main_layout.addWidget(self.status_label)
        
        self.description_text = QTextEdit()
        self.description_text.setReadOnly(True)
        self.description_text.setMaximumHeight(100)
        main_layout.addWidget(self.description_text)

        # 3D Plotter
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor)
        
        self.renderer = RobotRenderer(self.plotter)
        self.plotter.view_isometric()
        self.plotter.camera.azimuth += 45

        # Control
        control_layout = QWidget()
        layout_h = QVBoxLayout(control_layout)
        
        self.next_btn = QPushButton("Next Step")
        self.next_btn.clicked.connect(self.next_step)
        layout_h.addWidget(self.next_btn)
        
        self.toggle_btn = QPushButton("Toggle Config (Current: Elbow Up)")
        self.toggle_btn.clicked.connect(self.toggle_method)
        layout_h.addWidget(self.toggle_btn)
        
        main_layout.addWidget(control_layout)

        # Steps
        self.step_index = -1
        self.method = 'elbow_up'
        self.steps = [
            {
                "name": "Home Position",
                "target": [200, 0, 100, 0],
                "desc": "Moving to Task Home."
            },
            {
                "name": "Lift Z (Pure Heave)",
                "target": [200, 0, 200, 0],
                "desc": "Increasing Z to 200."
            },
            {
                "name": "Pitch Down (Pure Tilt)",
                "target": [200, 0, 200, -45],
                "desc": "Pitch to -45 (Down)."
            },
            {
                "name": "Reach Forward (Pure Surge)",
                "target": [280, 0, 200, -45],
                "desc": "Reaching forward to 280."
            },
            {
                "name": "Low Reach (Test Elbow Down ideal)",
                "target": [250, 0, 0, 0],
                "desc": "Low reach. Elbow Down often better here."
            }
        ]
        
        self.next_step()

    def toggle_method(self):
        if self.method == 'elbow_up':
            self.method = 'elbow_down'
            self.toggle_btn.setText("Toggle Config (Current: Elbow Down)")
        else:
            self.method = 'elbow_up'
            self.toggle_btn.setText("Toggle Config (Current: Elbow Up)")
        
        # Re-run current step
        self.run_ik()

    def next_step(self):
        self.step_index = (self.step_index + 1) % len(self.steps)
        self.run_ik()
        
    def run_ik(self):
        step = self.steps[self.step_index]
        x, y, z, pitch = step['target']
        self.status_label.setText(f"Step {self.step_index + 1}: {step['name']} ({self.method})")
        self.description_text.setText(step['desc'] + f"\nTarget: {step['target']}")
        
        try:
            # IK
            q = IK(float(x), float(y), float(z), float(pitch), method=self.method)
            print(f"IK ({self.method}) Target: {step['target']} -> Joints: {q}")
            
            # FK for Visuals
            T_ee, global_transforms = FK(q)
            self.renderer.update_actors(global_transforms)
            
        except Exception as e:
            self.description_text.setText(f"ERROR: {e}")
            print(f"Error: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MovementDebugger()
    window.show()
    sys.exit(app.exec_())
