# WVU Blimps ROS2 Workspace Documentation

## Project Overview
This is a ROS2-based autonomous blimp control system developed by the West Virginia University Blimps Team. The system enables both manual and autonomous control of blimps for various tasks including balloon detection, goal detection, and altitude control.

---

## Workspace Structure

```
ros2_ws/
├── src/wvu_blimps_ros2_src/    # Main source directory
│   ├── blimp_interfaces/        # Custom ROS2 message/service definitions
│   ├── camera/                  # Camera testing utilities
│   ├── controls/                # Control algorithms (Python)
│   ├── manual_control/          # Manual joystick control
│   ├── sensors/                 # Sensor nodes (Python)
│   ├── sensors_cpp/             # Sensor & control nodes (C++)
│   └── launch/                  # Launch files for different configurations
├── build/                       # Build artifacts
├── install/                     # Installed packages
└── log/                         # Build and runtime logs
```

---

## Package Descriptions

### 1. **blimp_interfaces** (Custom Message Definitions)

Defines custom ROS2 messages and services for blimp communication.

#### Messages:
- **`EscInput.msg`**: Motor control commands
  - `int8[] esc_pins`: GPIO pins for ESC motors
  - `float64 pwm_l`: Left motor PWM (1050-1950)
  - `float64 pwm_r`: Right motor PWM
  - `float64 pwm_d`: Down/vertical motor PWM

- **`ImuData.msg`**: IMU sensor data
  - `float32[] imu_lin_accel`: Linear acceleration [x, y, z]
  - `float32[] imu_gyro`: Gyroscope data [x, y, z]
  - `float32[] imu_euler`: Euler angles [roll, pitch, yaw]

- **`BaroData.msg`**: Barometer altitude data
  - `float64 height`: Relative height in meters

- **`CameraCoord.msg`**: Camera detection coordinates
  - `int64[] position`: [x, y] pixel coordinates

- **`CartCoord.msg`**: Cartesian coordinate control inputs
  - Position and orientation data for 6-DOF control

- **`UtcTime.msg`**: UTC timestamp data

- **`Bool.msg`**: Boolean flag messages

#### Services:
- **`Detection.srv`**: Object detection service
- **`GetCamera.srv`**: Camera data retrieval service

---

### 2. **sensors** (Python Sensor Nodes)

Interfaces with hardware sensors and publishes data to ROS2 topics.

#### Key Nodes:

**`BNO085.py`** - IMU Data Publisher
- **Topic Published**: `/imu_data` (ImuData)
- **Hardware**: BNO085 IMU sensor via I2C
- **Rate**: 100 Hz (0.01s timer)
- **Functionality**:
  - Reads linear acceleration, gyroscope, and quaternion data
  - Converts quaternions to Euler angles (roll, pitch, yaw)
  - Calibrates initial orientation on startup
  - Publishes relative orientation data

**`barometer.py`** - Altitude Sensor
- **Topic Published**: `/barometer_data` (BaroData)
- **Hardware**: BMP3XX barometer via I2C
- **Rate**: 10 Hz (0.1s timer)
- **Parameters**:
  - `sea_level_pressure`: Calibration pressure (default: 999.32 hPa)
- **Functionality**:
  - Measures absolute altitude
  - Calculates relative height from startup position
  - Error handling with fallback values

**`servo.py`** - Servo Test Utility
- Tests servo motor control via GPIO
- Uses pigpio daemon for PWM control

**`record_data.py`** - Data Logger
- **Topic Subscribed**: `/ESC_input`
- **Functionality**: Logs motor commands to file with timestamps

**`Lidar.py`**, **`Sonar.py`**, **`LED.py`** - Additional sensor interfaces

---

### 3. **sensors_cpp** (C++ Sensor & Control Nodes)

High-performance C++ implementations for computer vision and advanced control.

#### Key Nodes:

**`old_cam.cpp`** - Vision Detection System (Balloon & Goal Detection)
- **Topics Published**: 
  - `/cam_data` (CameraCoord)
  - `/cam_flag` (Bool)
- **Topics Subscribed**: 
  - `/joy` (Joy) - for mode switching
  - `/net_flag` (Bool)
- **Rate**: ~5 Hz (200ms timer)
- **Modes**:
  1. **Balloon Detection Mode** (`cam_mode=true`):
     - Color filtering (purple/green HSV range)
     - Contour detection using OpenCV
     - Finds largest contour as balloon
     - Publishes center coordinates with radius filtering
  
  2. **Goal Detection Mode** (`cam_mode=false`):
     - Yellow/orange color filtering
     - Canny edge detection
     - Hough Line Transform to detect rectangular goals
     - Calculates goal center from line midpoints

- **Camera Settings**: 1280x720 resolution, V4L2 backend

**`Balloon_pi.cpp`** - PI Controller for Balloon Tracking
- **Topics Subscribed**: 
  - `/cam_data` (CameraCoord)
  - `/barometer_data` (BaroData)
- **Topic Published**: `/balloon_input` (CartCoord)
- **Rate**: 20 Hz (50ms timer)
- **Parameters**:
  - `kpx`, `kix`: X-axis PI gains (yaw control)
  - `kpyu`, `kpyd`, `kiy`: Y-axis PI gains (altitude control)
  - `kpb`: Barometer proportional gain
  - `x_goal`, `y_goal`: Target pixel coordinates
  - `iheight`: Initial target height
- **Control Logic**:
  - Camera tracking: PI control on pixel error → yaw and vertical thrust
  - Barometer fallback: If no camera data for >5s, uses altitude hold
  - Publishes Cartesian acceleration commands

**`Extremum_Seeking.cpp`** - Advanced Extremum Seeking Control
- **Topic Subscribed**: `/cam_data` (CameraCoord)
- **Topic Published**: `/ESC_extremum_seeking_input` (EscInput)
- **Functionality**:
  - Uses extremum seeking algorithm to optimize balloon tracking
  - High-pass filtering and demodulation signals
  - Sinusoidal perturbation for gradient estimation
  - Timeout handling (5s) with motor reset

**`inv_kine.cpp`** - Inverse Kinematics / Dynamic Model
- Converts desired forces/torques to motor commands

**`force_to_ESC_input.cpp`** - Force to ESC Converter
- Translates Cartesian force commands to PWM values

---

### 4. **manual_control** (Manual Control Package)

Handles joystick input for manual blimp control.

#### Nodes:

**`joy_to_esc_input.py`** - Joystick to ESC Converter
- **Topic Subscribed**: `/joy` (Joy)
- **Topic Published**: `/ESC_Manual_input` (EscInput)
- **Rate**: Triggered by joystick events
- **Parameters**:
  - `Klm`: Left motor gain correction
  - `Krm`: Right motor gain correction
- **Joystick Mapping** (Xbox Controller):
  - Left stick Y-axis: Forward/backward thrust
  - Left/Right triggers: Yaw rotation (trim)
  - Right stick Y-axis: Vertical thrust
- **Functionality**:
  - Converts analog stick positions to PWM (1050-1950)
  - Applies motor calibration gains
  - Combines forward motion with yaw control

**`manual_bridge.py`** - Manual Input Bridge
- **Topic Subscribed**: `/ESC_Manual_input`
- **Topic Published**: `/ESC_input`
- **Functionality**: Simple passthrough for manual commands

---

### 5. **controls** (Control Algorithms Package)

Implements various control strategies for autonomous and semi-autonomous operation.

#### Nodes:

**`esc_driver.py`** - ESC Motor Driver (Hardware Interface)
- **Topic Subscribed**: `/ESC_input` (EscInput)
- **Hardware**: Controls ESC motors via pigpio GPIO PWM
- **Parameters**: 
  - `MAC`: Bluetooth controller MAC address
- **Pin Assignments**:
  - Pin 5: Left motor
  - Pin 6: Right motor
  - Pin 26: Down/vertical motor
- **Safety Features**:
  - Bluetooth connection check - motors stop if controller disconnected
  - PWM limiting (1050-1950 range)
  - Automatic pigpiod startup/cleanup

**`mode_switcher.py`** - Mode Switching Controller
- **Topics Subscribed**: 
  - `/ESC_Manual_input` (manual commands)
  - `/ESC_balloon_input` (autonomous commands)
  - `/joy` (mode switch button)
- **Topic Published**: `/ESC_input`
- **Functionality**:
  - Button A (button[0]) toggles between manual/autonomous
  - Selects which input stream to forward to motors
  - 2-second cooldown on mode changes

**`Bomber_Cntrl.py`** - Bomber Mode Controller
- **Topic Subscribed**: `/cam_data` (CameraCoord)
- **Topic Published**: `/ESC_Bomber_input` (EscInput)
- **Parameters**: `k` - proportional gain
- **Functionality**:
  - Uses least-squares optimization for 3-motor holonomic control
  - Computes motor forces to move toward camera target
  - Force direction matrix accounts for motor geometry
  - Applies bounded optimization (non-negative forces)

**`baro_control.py`** - Altitude Hold Controller
- **Topic Subscribed**: `/barometer_data` (BaroData)
- **Topic Published**: `/ESC_Baro_input` (EscInput)
- **Parameters**:
  - `kpb`: Proportional gain
  - `height`: Target height (meters)
- **Functionality**:
  - P-controller for altitude regulation
  - Only increases thrust when below target
  - Passive descent when above target

**`random_walk.py`** - Random Walk Generator
- **Topic Published**: `/ESC_random_input`
- **Rate**: Every 2 seconds
- **Functionality**:
  - Randomly selects one motor
  - Applies random PWM (1100-1300)
  - Used for system testing and data collection

**`net_servo.py`** - Net Deployment Servo
- Controls servo for payload/net deployment

**`bomber_mux.py`** - Input Multiplexer
- Combines multiple control inputs (bomber mode + altitude)

**`rudolph_mode_switcher.py`** - Alternative Mode Switcher
- Variant mode switcher for specific blimp configurations

---

### 6. **camera** (Camera Testing)

**`camera_test.py`** - Simple Camera Test
- Opens camera feed for verification
- 480x360 resolution
- OpenCV display window

---

### 7. **launch** (Launch Configurations)

Launch files orchestrate multiple nodes for different mission profiles.

**`test_launch.py`** - Basic Testing Configuration
- **Nodes Launched**:
  1. `joy` - Game controller node
  2. `joy_to_esc` - Joystick converter
  3. `manual_bridge` - Manual input bridge
  4. `esc_driver` - Motor driver
  5. `read_imu` - IMU sensor
  6. `balloon_detect` - Vision detection
  7. `read_altitude` - Barometer
  8. `balloon_detect_PI` - PI controller
  9. `record_data` - Data logger (commented)

**`autonomous_launch.py`** - Autonomous Mission
- **Nodes**:
  1. `joy` - Mode switching controller
  2. `joy_to_esc` - Manual override
  3. `balloon_detect_cpp` - C++ vision node
  4. `balloon_detect_control` - PI controller with tuned gains
  5. `esc_driver` - Motor driver
  6. `read_altitude` - Barometer with local pressure
  7. `mode_switcher` - Manual/auto toggle
- **Parameters**:
  - PI gains: kpx=0.35, kix=0.0
  - Sea level pressure: 1019.0 hPa (Morgantown, WV)

**`bomber_launch.py`** - Bomber Mission Profile
- **Nodes**:
  1. `esc_driver` - Motor control
  2. `random_walk_node` - Random exploration
  3. `bomber_cntrl` - Target tracking (k=0.5)
  4. `mux` - Input multiplexer
  5. `baro_cntrl` - Altitude hold (kpb=900, height=3.0m)
  6. `net_servo` - Payload deployment
  7. `balloon_detect_cpp` - Vision
  8. `read_altitude` - Barometer

**`Rudolph_Launch.py`, `cpp_auto_launch.py`, `drone_manual_launch.py`**
- Additional mission-specific configurations

**`BNO085_calibrate.py`, `Servo_Test.py`, `LED_test.py`, `program_esc.py`**
- Hardware calibration and testing utilities

---

## System Architecture & Data Flow

### Manual Control Flow
```
Xbox Controller (Hardware)
    ↓
[joy] ROS2 joy package
    ↓
/joy topic (sensor_msgs/Joy)
    ↓
[joy_to_esc_input] Joystick converter
    ↓
/ESC_Manual_input topic (EscInput)
    ↓
[manual_bridge] OR [mode_switcher] (if using auto/manual toggle)
    ↓
/ESC_input topic (EscInput)
    ↓
[esc_driver] Motor driver with Bluetooth safety check
    ↓
GPIO PWM via pigpio
    ↓
ESC Motors (Hardware)
```

### Autonomous Balloon Tracking Flow
```
Camera (Hardware)
    ↓
[old_cam / balloon_detect_cpp] Vision detection
    ↓
/cam_data topic (CameraCoord) - balloon position [x, y]
    ↓
[Balloon_pi / balloon_detect_control] PI Controller
    ↓
/balloon_input topic (CartCoord) - desired accelerations
    ↓
[force_to_ESC_input] (if using) OR direct mapping
    ↓
/ESC_balloon_input topic (EscInput)
    ↓
[mode_switcher] (selects autonomous or manual)
    ↓
/ESC_input topic
    ↓
[esc_driver] Motor driver
    ↓
ESC Motors
```

### Altitude Control Integration
```
Barometer (BMP3XX Hardware)
    ↓
[barometer] Sensor node
    ↓
/barometer_data topic (BaroData) - relative height
    ↓
┌──────────────────┬─────────────────┐
↓                  ↓                 ↓
[baro_control]  [Balloon_pi]    [bomber_mux]
P-controller    (fallback mode)  (combined ctrl)
    ↓                  ↓                 ↓
/ESC_Baro_input   (integrated)    (combined)
```

### Sensor Fusion Flow
```
IMU (BNO085) → [BNO085.py] → /imu_data → [dynamic_model/controllers]
Barometer → [barometer.py] → /barometer_data → [altitude controllers]
Camera → [old_cam.cpp] → /cam_data → [vision-based controllers]
Joystick → [joy node] → /joy → [manual control + mode switching]
```

---

## Control Modes

### 1. **Manual Mode**
- Direct joystick control
- Real-time pilot input
- Bluetooth safety: Motors stop if controller disconnects
- Used for takeoff, landing, and manual override

### 2. **Autonomous Balloon Tracking**
- Vision-based target tracking
- PI control on pixel error
- Maintains balloon in camera center
- Barometer fallback if vision lost

### 3. **Autonomous Goal Detection**
- Detects rectangular goals (yellow/orange)
- Centers blimp on goal
- Uses edge detection + Hough transforms

### 4. **Bomber Mode**
- Combines random walk exploration
- Target tracking when balloon detected
- Altitude hold at fixed height
- Net deployment capability

### 5. **Extremum Seeking Mode**
- Advanced optimization-based control
- Gradient-free optimization
- Adapts to system dynamics in real-time

---

## Hardware Configuration

### Motors (ESC Control via GPIO)
- **Left Motor (A)**: GPIO Pin 5
- **Right Motor (B)**: GPIO Pin 6  
- **Vertical/Down Motor (C)**: GPIO Pin 26
- **PWM Range**: 1050 (min) to 1950 (max)
- **Neutral**: 1500

### Sensors
- **IMU**: BNO085 via I2C (orientation, acceleration, gyro)
- **Barometer**: BMP3XX via I2C (altitude)
- **Camera**: USB webcam via V4L2 (1280x720 or 640x480)
- **Joystick**: Xbox controller via Bluetooth

### Compute Platform
- Raspberry Pi or Orange Pi
- Ubuntu 22.04 + ROS2 Humble
- `pigpiod` daemon for GPIO control

---

## Key Parameters & Tuning

### Vision Detection
- **Balloon HSV Range**: Purple (115-150 H, 40-255 S, 30-255 V)
- **Goal HSV Range**: Yellow (28-36 H, 80-255 S, 120-255 V)
- **Min/Max Radius**: 5-300 pixels
- **Camera Resolution**: 1280x720
- **Detection Rate**: 5 Hz

### PI Controller Tuning
- **kpx**: 0.35 (X-axis proportional gain)
- **kix**: 0.0 (X-axis integral gain - typically disabled)
- **kpyu/kpyd**: Y-axis gains (up/down asymmetry)
- **kiy**: Y-axis integral gain

### Altitude Control
- **kpb**: 900 (barometer proportional gain)
- **Target Height**: 3.0 meters (bomber mode)

### Motor Calibration
- **Klm**: Left motor gain (compensates for motor differences)
- **Krm**: Right motor gain

---

## Building & Running

### Build Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Launch Autonomous Mode
```bash
ros2 launch launch/autonomous_launch.py
```

### Launch Manual Test
```bash
ros2 launch launch/test_launch.py
```

### Launch Bomber Mission
```bash
ros2 launch launch/bomber_launch.py
```

### View Topics
```bash
ros2 topic list
ros2 topic echo /cam_data
ros2 topic echo /ESC_input
```

---

## Safety Features

1. **Bluetooth Disconnection Check**: Motors automatically stop if controller MAC not connected
2. **PWM Limiting**: All PWM values bounded to safe range (1050-1950)
3. **Vision Timeout**: Switches to barometer control if no camera data for 5 seconds
4. **Manual Override**: Mode switcher allows instant return to manual control
5. **Error Handling**: Sensor failures logged, fallback values published

---

## Development Notes

### Adding New Sensors
1. Create node in `sensors/` or `sensors_cpp/`
2. Define message in `blimp_interfaces/msg/`
3. Publish to descriptive topic name
4. Add to appropriate launch file

### Adding New Controllers
1. Create node in `controls/`
2. Subscribe to sensor topics
3. Publish to `/ESC_<controller_name>_input`
4. Add to mode switcher or mux if needed

### Calibration Workflow
1. Run ESC calibration: `program_esc.py`
2. Calibrate IMU: `BNO085_calibrate.py`
3. Measure sea level pressure at location
4. Test motors: `Servo_Test.py`
5. Test camera: `camera_test.py`
6. Tune controller gains in launch files

---

## Common Issues & Troubleshooting

**Motors don't respond:**
- Check `pigpiod` is running: `sudo pigpiod`
- Verify Bluetooth connection (check MAC address)
- Confirm ESCs are calibrated and armed

**Vision detection fails:**
- Adjust HSV color ranges in `old_cam.cpp`
- Check camera permissions: `/dev/video0`
- Verify lighting conditions

**IMU data noisy:**
- Run calibration routine
- Check I2C connections
- Verify sensor mounting (vibration isolation)

**Altitude drift:**
- Recalibrate sea level pressure parameter
- Check barometer sensor connections

---

## Contributors
WVU Blimps Team - West Virginia University

## License
See LICENSE file in repository root
