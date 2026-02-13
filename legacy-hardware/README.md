# Task 1 - Hardware Interface

Functions for controlling the physical OpenManipulator-X robot.

## Files

| File | Description |
|------|-------------|
| `robotSafeInit.m` | Initialize robot with safety limits |
| `emergencyStop.m` | Immediately disable all motor torques |
| `moveToPosition.m` | Move end-effector to XYZ position using IK |
| `progressiveMotorTest.m` | Test motors one at a time safely |
| `dxl_x64_c_thunk_pcwin64.dll` | Dynamixel SDK library |

## Quick Start

```matlab
% 1. Initialize robot (EDIT COM PORT!)
COM_PORT = 'COM3';  % <-- Change to your port
[port_num, lib_name, cleanup] = robotSafeInit(COM_PORT, 50);

% 2. Move to position
moveToPosition(port_num, lib_name, [200, 0, 150], 'auto');

% 3. Emergency stop (or just Ctrl+C)
emergencyStop(port_num, lib_name);

% 4. Cleanup happens automatically when 'cleanup' variable is cleared
```

## Safety Features

- **Joint limits**: ±90° (encoder 1024-3072)
- **Velocity limit**: Default 50 (safe slow speed)
- **Emergency stop**: Ctrl+C triggers torque disable
- **Automatic cleanup**: Motors disabled when script ends

## Finding Your COM Port

1. Open Device Manager (Windows)
2. Expand "Ports (COM & LPT)"
3. Look for "USB Serial Port (COMx)"
4. Use that COM port number

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Failed to open port" | Check COM port number, close other programs |
| "No response from motor" | Check power, USB connection |
| Robot moves erratically | Reduce velocity, check joint limits |
