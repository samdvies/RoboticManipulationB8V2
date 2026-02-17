# OpenManipulator-X Reference

## Overview
The OpenManipulator-X (Model: RM-X52-TNM) is a 4-DOF (Degrees of Freedom) robotic arm with a 1-DOF gripper, widely used for research and education. It is powered by DYNAMIXEL XM430-W350-T smart actuators.

## Coordinates & Configuration
> [!NOTE]
> **Coordinate System (User Specified):**
> - **X-axis:** Forward (increasing away from base).
> - **Y-axis:** Right (increasing to the right).
> - **Z-axis:** Up (increasing upwards).
> - **Origin:** Robot Base.
> - *Note: This is a **Left-Handed** coordinate system.*

> [!NOTE]
> **Elbow Configuration:**
> - **Preference:** "Elbow Down" (Joint 3 bent downwards).
> - **Reason:** To keep the arm close to the table.

## Specifications
- **Payload:** 500g
- **Reach:** 380mm
- **Weight:** 0.70kg (Assembly)
- **Actuators:** DYNAMIXEL XM430-W350-T
- **DYNAMIXEL Communication:** TTL Level Multidrop BUS (1Mbps default)
- **Power:** 12V DC

## Kinematics

### Denavit-Hartenberg (DH) Parameters
The standard DH parameters for the OpenManipulator-X are as follows:

| Link ($i$) | $\alpha_{i-1}$ (deg) | $a_{i-1}$ (mm) | $d_i$ (mm) | $\theta_i$ (deg) |
| :---: | :---: | :---: | :---: | :---: |
| 1     | 0      | 0       | 77      | $\theta_1$ |
| 2     | -90    | 0       | 0       | $\theta_2 - 90^\circ$ * |
| 3     | 0      | 128     | 0       | $\theta_3 + 90^\circ$ * |
| 4     | 0      | 124     | 0       | $\theta_4$ |
| End   | 0      | 126     | 0       | 0          |

*> **Note:** Offsets in $\theta$ (e.g., $\theta_2 - 90^\circ$) are common in standard DH conventions to align the "home" pose (often L-shape) with the mathematical zero state. The values above ($a$ links) represent the physical link lengths:
> - **Link 2 ($a_2$):** 128mm
> - **Link 3 ($a_3$):** 124mm
> - **Link 4 ($a_4$):** 126mm (to End Effector Center)

### Forward Kinematics
The position and orientation of the end-effector are calculated by multiplying the transformation matrices derived from the DH table:
$$ T_{base}^{end} = T_0^1 T_1^2 T_2^3 T_3^4 T_4^{end} $$

### Inverse Kinematics
Inverse kinematics (calculating joint angles from a desired end-effector pose) for the OpenManipulator-X can be solved using:
1.  **Geometric (Analytical) Solution:** Due to the 4-DOF planar nature of the arm (with offsets), a geometric approach is efficient. It typically involves:
    - Calculation of $\theta_1$ using `atan2(y, x)` of the target position.
    - Reducing the problem to a 3-link planar arm in the plane defined by $\theta_1$.
    - Using the Law of Cosines to solve for $\theta_2$ and $\theta_3$.
    - Calculating $\theta_4$ based on the desired pitch/orientation.
2.  **Geometric (Analytical) Solution:**
    The OpenManipulator-X (4-DOF) can be solved by decoupling the base rotation from the planar arm.

    **Step 1: Base Rotation ($\theta_1$)**
    $$ \theta_1 = \text{atan2}(P_y, P_x) $$

    **Step 2: Planar Projection**
    Calculate the target position in the frame of the planar arm (plane defined by $\theta_1$):
    $$ r = \sqrt{P_x^2 + P_y^2} - (\text{L1 offset if any}) $$
    $$ z = P_z - d_1 $$
    The target for the wrist center (assuming $\theta_{pitch}$ is the desired end-effector pitch) is:
    $$ r_{wc} = r - L_4 \cos(\theta_{pitch}) $$
    $$ z_{wc} = z - L_4 \sin(\theta_{pitch}) $$

    **Step 3: Elbow Angle ($\theta_3$) - Law of Cosines**
    Consider the triangle formed by $L_2$, $L_3$, and the distance to the wrist center $D = \sqrt{r_{wc}^2 + z_{wc}^2}$.
    $$ \cos(\alpha) = \frac{L_2^2 + L_3^2 - D^2}{2 L_2 L_3} $$
    $$ \theta_3 = \pi - \alpha \quad (\text{or similar depending on zero definition}) $$
    *Note: OpenManipulator's $\theta_3$ zero might be defined differently (e.g., straight vs bent).*

    **Step 4: Shoulder Angle ($\theta_2$)**
    $$ \theta_2 = \text{atan2}(z_{wc}, r_{wc}) - \text{atan2}(L_3 \sin(\pi - \theta_3), L_2 + L_3 \cos(\pi - \theta_3)) $$

    **Step 5: Wrist Angle ($\theta_4$)**
    $$ \theta_4 = \theta_{pitch} - (\theta_2 + \theta_3) $$

## Joint Limits

Software-enforced joint limits for safe operation:

| Joint | Name | Min (°) | Max (°) | Notes |
| :---: | :--- | :---: | :---: | :--- |
| 1 | Base | -90 | +90 | User restriction: front half-plane only |
| 2 | Shoulder | -117 | +117 | Conservative safe default |
| 3 | Elbow | -117 | +117 | Conservative safe default |
| 4 | Wrist | -117 | +117 | Conservative safe default |

> [!NOTE]
> Joint limits are enforced at the IK output via `JointLimits.clamp_joints()` (Python) / `OpenManipulator.JointLimits.Clamp()` (MATLAB). Angles exceeding limits are clamped to the nearest boundary with a console warning.

## Software & Control
- **ROS/ROS2:** Fully supported packages `open_manipulator` and `open_manipulator_controls`.
- **MoveIt!:** Integrated for motion planning and IK.
- **Middleware:** Uses `robotis_manipulator` library for kinematic calculations and joint control.

## Velocity Kinematics (Jacobian)
The Jacobian matrix $J$ relates joint velocities $\dot{\mathbf{q}}$ to end-effector velocities $\mathbf{v}_{ee}$:
$$ \mathbf{v}_{ee} = J(\mathbf{q}) \dot{\mathbf{q}} $$
For OpenManipulator-X, $J$ is derived from the partial derivatives of the forward kinematics equations. It is essential for differential control and singularity analysis.

## Python Implementation & Control
- **ROS2 + MoveIt!:** Best for high-level control. Use `omx_cpp_interface` to bridge Python and MoveIt 2.
- **`robotis_manipulator`:** The core library handling kinematics and trajectory generation.
- **Custom Implementation:** DH parameters allow for direct Python implementation of FK/IK for lightweight simulations.

## Gripper Control (Pincer)

The OpenManipulator-X features a 1-DOF gripper (Joint ID 15).

### Configuration
- **Control Interface:** `src/+OpenManipulator/HardwareInterface.m`
- **Control Method:** Position control via DYNAMIXEL (0-100%).
- **Encoder Limits:**
  - **Open (0%):** 1365 (120°)
  - **Closed (100%):** 2276 (200°)

### How to Run (MATLAB)
You can find a test script at `scripts/test_gripper.m`.

```matlab
% Basic usage in Command Window:
hw = OpenManipulator.HardwareInterface('COM4', 1000000);
hw.enableTorque();

hw.openGripper();              % Fully Open
hw.closeGripper();             % Fully Close
hw.setGripperPosition(50);     % Set to 50% width
pos = hw.readGripperPosition(); % Read current position (0-100)

hw.disconnect();
```
