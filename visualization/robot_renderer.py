import pyvista as pv
import numpy as np

class RobotRenderer:
    def __init__(self, plotter):
        self.plotter = plotter
        self.actors = []
        self.link_meshes = []
        
        # Colors
        self.colors = ['grey', 'white', 'grey', 'white', 'red'] # Base, L1, L2, L3, EE(wrist)
        
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
        
        # Mesh 4 (Link 4 - Wrist stub): Attached to Frame 4.
        # Shortened wrist: 0 to 90mm (leaves 36mm for jaw assembly)
        self.link_meshes.append(pv.Box(bounds=(0, 90, -10, 10, -10, 10)))

        # Add initial actors for the 5 link meshes
        for i, mesh in enumerate(self.link_meshes):
            actor = self.plotter.add_mesh(mesh, color=self.colors[i], show_edges=False)
            self.actors.append(actor)

        # ── Gripper Jaw Meshes (attached to Frame 4, positioned at wrist tip) ──
        # Jaw geometry: each jaw is a thin box extending from wrist tip (X=90)
        #   to EE (X=126). Offset symmetrically along Z (pincers open up/down).
        jaw_length = 36   # mm (126 - 90)
        jaw_thick  = 6    # mm (Z extent of each jaw finger)
        jaw_height = 8    # mm (Y extent — width of each finger)
        self._jaw_half_gap_open = 20.0  # mm — half of max jaw gap (40mm total)

        # Jaws split along Z: top jaw at +Z, bottom jaw at -Z
        self._jaw_left_mesh  = pv.Box(bounds=(90, 126, -jaw_height/2, jaw_height/2, 0, jaw_thick))
        self._jaw_right_mesh = pv.Box(bounds=(90, 126, -jaw_height/2, jaw_height/2, -jaw_thick, 0))

        self._jaw_left_actor  = self.plotter.add_mesh(self._jaw_left_mesh, color='darkgrey', show_edges=True)
        self._jaw_right_actor = self.plotter.add_mesh(self._jaw_right_mesh, color='darkgrey', show_edges=True)

        # Store current jaw state
        self._current_jaw_width_mm = 40.0  # start fully open
        self._ee_transform = np.eye(4)     # Frame 4 transform cache
            
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
        self.plotter.add_mesh(pv.Arrow(start=origin, direction=[1, 0, 0], scale=scale), color='red')
        self.plotter.add_text("X+", position=(1.2*scale, 0, 0), color='red', font_size=10)
        
        # Y Axis (Green) - Left (Internal Frame) -> PyVista +Y
        self.plotter.add_mesh(pv.Arrow(start=origin, direction=[0, 1, 0], scale=scale), color='green')
        self.plotter.add_text("Y+", position=(0, 1.2*scale, 0), color='green', font_size=10)
        
        # Z Axis (Blue) - Up
        self.plotter.add_mesh(pv.Arrow(start=origin, direction=[0, 0, 1], scale=scale), color='blue')
        self.plotter.add_text("Z+", position=(0, 0, 1.2*scale), color='blue', font_size=10)

        # Add some numbers/ticks at 100mm intervals (Start at 200 to avoid clutter with axis arrow)
        for i in range(2, 5):
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
        # Expecting list of 4x4 numpy arrays: [T1, T2, T3, T4, T_ee]
        transforms = global_transforms_flat
        
        # Update Base (Mesh 0): Identity
        self.actors[0].user_matrix = np.eye(4)
        
        # Update Links (transforms[0]→T1, [1]→T2, [2]→T3, [3]→T4, [4]→T_ee)
        for i in range(len(transforms)):
            if i+1 < len(self.actors):
                T = np.array(transforms[i])
                self.actors[i+1].user_matrix = T

        # Cache Frame 4 transform for jaw positioning (transforms[3] = T04)
        if len(transforms) >= 4:
            self._ee_transform = np.array(transforms[3])
            self._update_jaw_transforms()

    def update_gripper(self, jaw_width_mm):
        """Update jaw opening width (mm). 0 = closed, 40 = fully open."""
        self._current_jaw_width_mm = float(np.clip(jaw_width_mm, 0.0, 40.0))
        self._update_jaw_transforms()

    def _update_jaw_transforms(self):
        """Recompute jaw actor transforms from cached Frame 4 + current jaw width."""
        T4 = self._ee_transform
        half_gap = self._current_jaw_width_mm / 2.0

        # Top jaw: offset +Z in Frame 4's local coordinate system
        T_top = T4.copy()
        T_top[:3, 3] += T4[:3, 2] * half_gap   # T4 Z-column * offset
        self._jaw_left_actor.user_matrix = T_top

        # Bottom jaw: offset -Z in Frame 4's local coordinate system
        T_bot = T4.copy()
        T_bot[:3, 3] -= T4[:3, 2] * half_gap
        self._jaw_right_actor.user_matrix = T_bot


