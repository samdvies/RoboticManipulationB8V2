---
date: 2026-02-12
topic: OpenManipulator-X Forward Kinematics Implementation
tags: [kinematics, python, open-manipulator, research]
status: research_complete
---

# Project State: New Implementation
The goal is to implement a Forward Kinematics (FK) solver for the OpenManipulator-X robotic arm from scratch.

## User Review Required
- **Coordinate System:** User requested X-Forward, Y-Right, Z-Up (Left-Handed). This requires careful transform handling if using standard libraries.
- **Elbow Config:** "Elbow Down" preference for IK.
- **Verification:** Round-trip `FK(IK(pos)) == pos`.
- **Language:** MATLAB (Core Logic), Python (Visualization)
- **Toolbox:** Robotics System Toolbox (optional).
- **Visualization:** Python `PyQt5` (GUI) + `PyVista` (3D Rendering).
- **Interfacing:** MATLAB Engine API for Python.

## Recommended Folder Structure
For a clean, maintainable robotic simulation project in MATLAB with interactive external visualization:

```
RoboticManipulationV2/
├── docs/               # Documentation (PRDs, specs)
├── src/                # Source code
│   └── +OpenManipulator/  # Package folder (denoted by +)
│       ├── FK.m           # Forward Kinematics function
│       ├── IK.m           # Inverse Kinematics function (Geometric)
│       ├── GetDH.m        # Returns DH parameters
│       ├── GetTransform.m # Helper for DH transformation matrix
├── visualization/      # External visualization logic
│   ├── app.py          # Main PyQt5 application
│   └── robot_renderer.py # PyVista rendering logic
├── tests/              # Unit tests
│   ├── TestFK.m        # FK validation
│   └── TestIK.m        # IK validation (Round-trip tests)
├── scripts/            # Executable scripts/examples
│   └── demo_fk.m       # Usage example
└── README.md
```

## Implementation Standards
1.  **Documentation:** Use MATLAB help comments (`%`) at the beginning of functions.
2.  **Vectorization:** Use matrix operations where possible.
3.  **Packages:** Use MATLAB packages (`+Folder`) for namespace management.
4.  **Verification:** Create a test script that asserts calculated positions against known values.

## Next Steps
Research concluded. Please give a `/clear` and load `.agent/2-spec.md` for planning.
