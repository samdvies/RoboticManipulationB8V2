import sys
import os
import numpy as np

# Ensure we can import from the root directory when running this script directly
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QGroupBox, QGridLayout)
from PyQt5.QtCore import Qt
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.FK import FK

class JointDebugWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Joint Debugger")
        self.setGeometry(100, 100, 1200, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # 1. Left Panel: Joint Controls
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_panel.setFixedWidth(300)
        main_layout.addWidget(control_panel)

        self.sliders = {}
        self.labels = {}
        
        joint_group = QGroupBox("Joint Angles (Degrees)")
        joint_layout = QGridLayout()
        joint_group.setLayout(joint_layout)
        
        # Joint Ranges
        joints = [
            ('q1 (Base)', -180, 180, 0),
            ('q2 (Shoulder)', -180, 180, 0),
            ('q3 (Elbow)', -180, 180, 0),
            ('q4 (Wrist)', -180, 180, 0)
        ]
        
        for i, (name, min_val, max_val, default) in enumerate(joints):
            key = f"q{i+1}"
            label = QLabel(f"{name}: {default}")
            self.labels[key] = label
            
            slider = QSlider(Qt.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setValue(default)
            slider.valueChanged.connect(self.update_robot)
            self.sliders[key] = slider
            
            joint_layout.addWidget(QLabel(name), i, 0)
            joint_layout.addWidget(slider, i, 1)
            joint_layout.addWidget(label, i, 2)
            
        control_layout.addWidget(joint_group)
        control_layout.addStretch()

        # 2. Right Panel: 3D Visualization
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor)
        
        self.renderer = RobotRenderer(self.plotter)
        self.plotter.view_isometric()
        self.plotter.camera.azimuth += 45
        
        self.update_robot()

    def update_robot(self):
        q1 = self.sliders['q1'].value()
        q2 = self.sliders['q2'].value()
        q3 = self.sliders['q3'].value()
        q4 = self.sliders['q4'].value()
        
        self.labels['q1'].setText(f"q1 (Base): {q1}")
        self.labels['q2'].setText(f"q2 (Shoulder): {q2}")
        self.labels['q3'].setText(f"q3 (Elbow): {q3}")
        self.labels['q4'].setText(f"q4 (Wrist): {q4}")
        
        # Calculate FK
        q = [q1, q2, q3, q4]
        try:
            T_ee, global_transforms = FK(q)
            self.renderer.update_actors(global_transforms)
            
            # Print EE Pos
            pos = T_ee[:3, 3]
            print(f"Joints: {q} -> EE Pos: {pos}")
            
        except Exception as e:
            print(f"FK Error: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = JointDebugWindow()
    window.show()
    sys.exit(app.exec_())
