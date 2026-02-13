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
        self.next_btn = QPushButton("Next Step")
        self.next_btn.clicked.connect(self.next_step)
        main_layout.addWidget(self.next_btn)

        # Steps
        self.step_index = -1
        self.steps = [
            {
                "name": "Home Position",
                "target": [200, 0, 100, 0],
                "desc": "Moving to Task Home.\nArm should be reaching forward (X=200), centered (Y=0), slightly up (Z=100).\nPitch is horizontal (0)."
            },
            {
                "name": "Lift Z (Pure Heave)",
                "target": [200, 0, 200, 0],
                "desc": "Increasing Z to 200.\nEXPECTATION: Gripper moves STRAIGHT UP.\n- X should NOT change.\n- Pitch should remain 0 (Horizontal).\n- Arm should unfold vertically."
            },
            {
                "name": "Pitch Down (Pure Tilt)",
                "target": [200, 0, 200, -45],
                "desc": "Changing Pitch to -45 (Down).\nEXPECTATION: Wrist tilts down.\n- Wrist position (X,Y,Z) should remain roughly constant.\n- Only the gripper angle changes."
            },
            {
                "name": "Reach Forward (Pure Surge)",
                "target": [280, 0, 200, -45],
                "desc": "Increasing X to 280.\nEXPECTATION: Arm reaches forward.\n- Height (Z) should NOT drop.\n- Pitch should remain -45.\n- No side-to-side wobble."
            },
            {
                "name": "Return Home",
                "target": [200, 0, 100, 0],
                "desc": "Returning to start.\nVerifying repeatability."
            }
        ]
        
        self.next_step()

    def next_step(self):
        self.step_index = (self.step_index + 1) % len(self.steps)
        step = self.steps[self.step_index]
        
        x, y, z, pitch = step['target']
        self.status_label.setText(f"Step {self.step_index + 1}: {step['name']} (Target: [{x}, {y}, {z}, {pitch}])")
        self.description_text.setText(step['desc'])
        
        try:
            # IK
            q = IK(float(x), float(y), float(z), float(pitch))
            print(f"IK Target: {step['target']} -> Joints: {q}")
            
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
