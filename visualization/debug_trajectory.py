import sys
import os
import time
import numpy as np

# Ensure import paths
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtCore import QTimer
from pyvistaqt import QtInteractor
from visualization.robot_renderer import RobotRenderer
from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK

class TrajectoryDebugger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Trajectory Tracer")
        self.setGeometry(100, 100, 1000, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Controls
        controls = QWidget()
        control_layout = QVBoxLayout(controls)
        
        self.shape_combo = QComboBox()
        self.shape_combo.addItems(["Circle (Radius=80mm)", "Square (150x150mm)", "Figure 8"])
        control_layout.addWidget(self.shape_combo)
        
        self.run_btn = QPushButton("Run Trajectory")
        self.run_btn.clicked.connect(self.start_trajectory)
        control_layout.addWidget(self.run_btn)
        
        self.status = QLabel("Ready")
        control_layout.addWidget(self.status)
        
        main_layout.addWidget(controls)

        # 3D Plotter
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor)
        self.plotter.view_isometric()
        self.plotter.add_axes()
        self.plotter.show_grid()
        
        self.renderer = RobotRenderer(self.plotter)
        
        # Timer for animation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.path_points = []
        self.current_frame = 0
        
        # Trail
        self.trail_actor = None

    def generate_circle(self):
        points = []
        center = [200, 0, 150]
        radius = 80
        pitch = 0  # Fixed horizontal pitch
        
        for theta in np.linspace(0, 2*np.pi, 100):
            x = center[0]
            y = center[1] + radius * np.cos(theta)
            z = center[2] + radius * np.sin(theta)
            points.append([x, y, z, pitch])
        return points

    def generate_square(self):
        points = []
        z = 150
        pitch = -45
        # Square 150x150 centered at X=200, Y=0
        corners = [
            [200, -75, z],
            [275, -75, z],
            [275, 75, z],
            [200, 75, z],
            [200, -75, z] # Close loop
        ]
        
        # Interpolate
        steps_per_leg = 25
        for i in range(len(corners)-1):
            start = np.array(corners[i])
            end = np.array(corners[i+1])
            for t in np.linspace(0, 1, steps_per_leg):
                pt = start + t * (end - start)
                points.append([pt[0], pt[1], pt[2], pitch])
                
        return points

    def start_trajectory(self):
        shape = self.shape_combo.currentText()
        if "Circle" in shape:
            self.path_points = self.generate_circle()
        elif "Square" in shape:
            self.path_points = self.generate_square()
        else:
            self.path_points = self.generate_circle() # Default
            
        self.current_frame = 0
        self.timer.start(50) # 20fps
        
        # Visualize Path
        path_coords = np.array([p[:3] for p in self.path_points])
        if self.trail_actor:
            self.plotter.remove_actor(self.trail_actor)
        self.trail_actor = self.plotter.add_lines(path_coords, color='yellow', width=3)

    def update_frame(self):
        if self.current_frame >= len(self.path_points):
            self.current_frame = 0 # Loop
            
        target = self.path_points[self.current_frame]
        x, y, z, pitch = target
        
        self.status.setText(f"Frame {self.current_frame}: Target [{x:.1f}, {y:.1f}, {z:.1f}]")
        
        try:
            # Solve IK (Default Elbow Up)
            q = IK(x, y, z, pitch, method='elbow_up')
            
            # Update Robot
            T_ee, global_transforms = FK(q)
            self.renderer.update_actors(global_transforms)
            
        except Exception as e:
            print(f"IK Fail: {e}")
            
        self.current_frame += 1

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TrajectoryDebugger()
    window.show()
    sys.exit(app.exec_())
