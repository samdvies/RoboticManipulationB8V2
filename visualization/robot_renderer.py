import pyvista as pv
import numpy as np

class RobotRenderer:
    def __init__(self, plotter):
        self.plotter = plotter
        self.actors = []
        self.link_meshes = []
        
        # Colors
        self.colors = ['grey', 'white', 'grey', 'white', 'red'] # Base, L1, L2, L3, EE
        
        # Initialize Mesh Geometry (defined relative to their ATTACHED FRAME)
        
        # Mesh 0 (Base): Fixed at World Origin.
        # Simple cylinder for base.
        self.link_meshes.append(pv.Cylinder(radius=20, height=20, center=(0,0,10), direction=(0,0,1)))
        
        # Mesh 1 (Link 1 - Shoulder): Attached to Frame 1
        # Frame 1 is at Z=77 relative to Base.
        # Mesh should go from Frame 0 (Z=-77 relative to Frame 1) to Frame 1 (Z=0).
        self.link_meshes.append(pv.Cylinder(radius=15, height=77, center=(0,0,-77/2), direction=(0,0,1)))
        
        # Mesh 2 (Link 2 - Upper Arm): Attached to Frame 2.
        # Connects Joint 2 (Frame 2) to Joint 3 (Frame 3).
        # Frame 3 is at X=+128 relative to Frame 2.
        # Mesh should go from 0 to 128 along +X.
        self.link_meshes.append(pv.Box(bounds=(0, 128, -15, 15, -15, 15)))
        
        # Mesh 3 (Link 3 - Forearm): Attached to Frame 3.
        # Connects Joint 3 to Joint 4 (at X=+124).
        self.link_meshes.append(pv.Box(bounds=(0, 124, -12, 12, -12, 12)))
        
        # Mesh 4 (Link 4 - Wrist/Gripper): Attached to Frame 4.
        # Connects Joint 4 to End Effector (at X=+126).
        self.link_meshes.append(pv.Box(bounds=(0, 126, -10, 10, -10, 10)))

        # Add initial actors
        for i, mesh in enumerate(self.link_meshes):
            actor = self.plotter.add_mesh(mesh, color=self.colors[i], show_edges=False)
            self.actors.append(actor)
            
        # Axes
        self.add_axes()
        self.add_workspace()

    def add_axes(self):
        # Draw custom axes at origin to represent User Coordinates
        # User X = Red (Forward/PyVista X)
        # User Y = Green (Right/PyVista -Y)
        # User Z = Blue (Up/PyVista Z) (Assuming PyVista Z is Up)
        
        origin = np.array([0, 0, 0])
        scale = 100
        
        # X Axis (Red) - Forward
        self.plotter.add_mesh(pv.Arrow(start=origin, direction=[1, 0, 0], scale=scale*4), color='red')
        self.plotter.add_text("X+", position=(4.2*scale, 0, 0), color='red', font_size=12)
        
        # Y Axis (Green) - Left (Internal Frame) -> PyVista +Y
        self.plotter.add_mesh(pv.Arrow(start=origin, direction=[0, 1, 0], scale=scale*4), color='green')
        self.plotter.add_text("Y+", position=(0, 4.2*scale, 0), color='green', font_size=12)
        
        # Z Axis (Blue) - Up
        self.plotter.add_mesh(pv.Arrow(start=origin, direction=[0, 0, 1], scale=scale*4), color='blue')
        self.plotter.add_text("Z+", position=(0, 0, 4.2*scale), color='blue', font_size=12)

        # Add some numbers/ticks at 100mm intervals
        for i in range(1, 5):
            val = i * 100
            # X labels
            self.plotter.add_text(str(val), position=(val, 0, 0), color='red', font_size=8)
            # Y labels
            self.plotter.add_text(str(val), position=(0, val, 0), color='green', font_size=8)
            # Z labels
            self.plotter.add_text(str(val), position=(0, 0, val), color='blue', font_size=8)

    def add_workspace(self):
        # Semi-transparent sphere indicating max reach ~380mm
        # Centered at (0,0,77) which is shoulder (roughly)
        sphere = pv.Sphere(radius=380, center=(0,0,77))
        self.plotter.add_mesh(sphere, color='lightblue', opacity=0.1, style='wireframe')

    def update_actors(self, global_transforms_flat):
        # global_transforms_flat: List or array of 16*N elements (column-major or row-major?)
        # MATLAB returns matrices. matlab.engine returns standard Python lists/arrays.
        # If passed as 4x4xN numpy array, good.
        # If passed as list, we need to reshape.
        
        # Let's assume input is a flattened list of 4x4 matrices in Row-Major order (Python default)?
        # Or MATLAB linear indexing? 
        # MATLAB engine usually converts matrices to list of lists (2D) for 2D.
        # For 3D array? It might return a large list.
        # We will inspect input type in app.
        
        # Assuming input is valid list of 4x4 matrices (T1, T2, T3, T4, T_ee)
        # Wait, Base (Mesh 0) is static (Identity).
        # Mesh 1 uses T1.
        # Mesh 2 uses T2.
        # Mesh 3 uses T3.
        # Mesh 4 uses T4.
        
        transforms = global_transforms_flat # Expecting list of 4x4 arrays
        
        # Update Base (Mesh 0): Identity
        self.actors[0].user_matrix = np.eye(4)
        
        # Update Links
        # transforms[0] -> T1
        # transforms[1] -> T2
        # ...
        for i in range(len(transforms)):
            if i+1 < len(self.actors): # Avoid index error if mismatch
                T = np.array(transforms[i])
                self.actors[i+1].user_matrix = T
