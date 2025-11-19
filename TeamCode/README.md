# FTC Team 25756 Nano Ninjas - Robot Code Architecture

## Overview
This codebase implements a modular, subsystem-based architecture for the 2025-26 DECODE season robot. The design separates hardware management, subsystem control, and OpMode logic for better maintainability and reusability.

## Architecture

### Main OpMode
**`MainTeleOp.java`** - Primary driver-controlled OpMode
- Coordinates all subsystems
- Handles gamepad input
- Provides dual drive modes (field-relative and robot-relative)

### Hardware Layer
**`RobotHardware.java`** - Centralized hardware management
- Initializes all motors, servos, and sensors
- Sets motor directions and configurations
- Provides utility methods for hardware control
- Hardware status logging for debugging

**Hardware Components:**
- **Drive Motors:** `fl_motor`, `fr_motor`, `bl_motor`, `br_motor`
- **Shooter Motors:** `leftShooter`, `rightShooter`
- **Intake Motor:** `pushMotor`
- **Servo:** `gateServo`
- **Sensor:** `imu` (for field-relative driving)

### Mechanisms Layer (`mechanisms` package)

#### **`MecanumDrive.java`**
- **Mecanum Drive:** Standard robot-relative movement with power scaling (0.40) and normalization
- **Field-Relative Drive:** IMU-based driving where joystick direction remains consistent regardless of robot orientation
- **Methods:**
  - `mecanumDrive(drive, strafe, turn)` - Robot-relative movement
  - `driveFieldRelative(forward, right, rotate, imu)` - Field-relative movement
  - `stop()` - Emergency stop

#### **`Shooter.java`**
- Controls dual-motor shooter mechanism
- **Methods:**
  - `startShooting()` - Full power shooting
  - `stopShooting()` - Stop shooter motors
  - `setShooterPower(power)` - Variable power control
  - `isRunning()` - Status check

#### **`Intaker.java`**
- Manages artifact manipulation motor
- **Methods:**
  - `startPushing()` - Forward intake operation
  - `stopPushing()` - Stop intake motor
  - `reversePush()` - Reverse intake direction
  - `setPushPower(power)` - Variable power control

#### **`Gate.java`**
- Controls servo-based gate mechanism
- **Methods:**
  - `openGate()` - Move to open position (0.5)
  - `closeGate()` - Move to closed position (-1.0)
  - `setGatePosition(position)` - Custom positioning
  - `getGatePosition()` - Current position
  - `isOpen()` - Status check

## Control Scheme

### Gamepad 1 (Driver)
- **Left Stick Y:** Forward/Backward movement
- **Left Stick X:** Left/Right strafing
- **Right Stick X:** Rotation
- **Left Bumper:** Switch to robot-relative drive mode
- **A Button:** Reset IMU heading (field-relative calibration)

### Gamepad 2 (Operator)
- **Right Bumper:** Activate shooter
- **Left Bumper:** Activate intake/pusher
- **Triangle:** Toggle gate (open/close)

## Drive Modes

### Field-Relative Drive (Default)
- Joystick directions remain consistent relative to the field
- Forward always moves away from the driver station
- Uses IMU for orientation correction
- Easier for drivers to control

### Robot-Relative Drive (Left Bumper)
- Traditional RC car-style control
- Forward moves in robot's current forward direction
- Useful for precise maneuvering

## Motor Configuration

### Drive Motors (Mecanum)
- **Front Left:** REVERSE direction
- **Front Right:** Normal direction  
- **Back Left:** REVERSE direction
- **Back Right:** Normal direction

### Mechanism Motors
- **Left Shooter:** Normal direction
- **Right Shooter:** REVERSE direction
- **Push Motor:** Normal direction

## File Structure
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── teleop/
│   ├── MainTeleOp.java       # Main driver-controlled OpMode
│   └── FTCNanoNinjasCode.java # Original monolithic code (reference)
├── autonomous/
│   ├── AutoTestBlue.java     # Blue alliance autonomous
│   └── AutoTestRed.java      # Red alliance autonomous
├── mechanisms/
│   ├── MecanumDrive.java     # Drive train control
│   ├── Shooter.java          # Shooter mechanism control
│   ├── Intaker.java          # Intake/pusher control
│   └── Gate.java             # Gate servo control
├── util/
│   ├── RobotHardware.java    # Hardware initialization and management
│   └── Constants.java        # Robot constants and configurations
└── readme.md                 # This documentation
```

## Benefits of This Architecture

### **Modularity**
- Each subsystem is independent and testable
- Easy to modify individual mechanisms without affecting others
- Clear separation of concerns

### **Reusability**
- Same subsystems work in TeleOp and Autonomous OpModes
- Consistent behavior across different OpModes
- Easy to create new OpModes using existing subsystems

### **Maintainability**
- Bug fixes in one place affect all OpModes
- Hardware changes only require updates to RobotHardware class
- Clear code organization for team collaboration

### **Debugging**
- Individual subsystems can be tested separately
- Hardware status logging for troubleshooting
- Isolated functionality for easier problem identification

## Usage Example

```java
// Initialize robot and mechanisms
RobotHardware robot = new RobotHardware();
robot.init(hardwareMap);

MecanumDrive drive = new MecanumDrive(robot);
Shooter shooter = new Shooter(robot);

// Use in OpMode
drive.mecanumDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
if (gamepad2.right_bumper) {
    shooter.startShooting();
} else {
    shooter.stopShooting();
}
```

## Future Enhancements
- Add autonomous OpModes using the same subsystems
- Implement PID control for precise movements
- Add sensor feedback for closed-loop control
- Create utility classes for common autonomous functions
- Add telemetry and logging for match analysis

---
**Team 25756 Nano Ninjas - 2025-26 DECODE Season**