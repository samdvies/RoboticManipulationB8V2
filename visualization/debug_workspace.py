import sys
import os
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel

# Ensure import paths
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from visualization.kinematics.IK import IK
from visualization.kinematics.FK import FK

class WorkspaceDebugger(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenManipulator-X Workspace Visualization")
        self.setGeometry(100, 100, 1000, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        self.status = QLabel("Generating Workspace Point Cloud...")
        main_layout.addWidget(self.status)

        # 3D Plotter
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor)
        self.plotter.view_isometric()
        self.plotter.add_axes()
        self.plotter.show_grid()

        # Generate Points
        self.generate_workspace()

    def generate_workspace(self):
        num_samples = 2000
        up_points = []
        down_points = []
        invalid_points = []

        # Sampling Range (mm)
        x_range = [-400, 400]
        y_range = [-400, 400]
        z_range = [-100, 500]

        print(f"Sampling {num_samples} points for both configurations...")

        for _ in range(num_samples):
            x = np.random.uniform(*x_range)
            y = np.random.uniform(*y_range)
            z = np.random.uniform(*z_range)
            pitch = 0 # Assume horizontal picking

            # Check Elbow Up
            try:
                q_up = IK(x, y, z, pitch, method='elbow_up')
                if not any(np.isnan(q_up)):
                    up_points.append([x, y, z])
            except:
                pass

            # Check Elbow Down
            try:
                q_down = IK(x, y, z, pitch, method='elbow_down')
                if not any(np.isnan(q_down)):
                    down_points.append([x, y, z])
            except:
                pass

        # Convert to numpy
        up_points = np.array(up_points)
        down_points = np.array(down_points)
        
        print(f"Elbow Up: {len(up_points)}, Elbow Down: {len(down_points)}")
        self.status.setText(f"Workspace: {len(up_points)} Up (Green), {len(down_points)} Down (Blue)")

        # Visualize
        if len(up_points) > 0:
            self.plotter.add_mesh(up_points, color='green', point_size=4, render_points_as_spheres=True, label='Elbow Up', opacity=0.6)
        
        if len(down_points) > 0:
            self.plotter.add_mesh(down_points, color='blue', point_size=4, render_points_as_spheres=True, label='Elbow Down', opacity=0.6)

        # Add Robot Base for context
        self.plotter.add_mesh(pv.Cylinder(radius=20, height=50, center=(0,0,25)), color='grey')
        self.plotter.add_legend()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WorkspaceDebugger()
    window.show()
    sys.exit(app.exec_())
