# OpenMV Ball & Goal Detection System

Vision system for detecting and tracking colored objects (ball and goals) using OpenMV camera module. Designed for robotics applications like soccer robots.

## Overview

This project uses computer vision to detect an orange ball and colored goals (blue/yellow), calculating their real-world positions relative to the camera. The system outputs X/Y coordinates that can be used for robot navigation and targeting.

## Files

- **ball-identifier.py** - Original version (V1) with UART communication
- **ball-identifierV2.py** - Improved version with better structure and testing mode
- **ball-identifierV3.py** - Latest version with USB VCP communication and LED feedback ⭐

## V3 Features & Improvements

### Key Enhancements

1. **USB VCP Communication**
   - Replaced UART with USB Virtual COM Port (`pyb.USB_VCP`)
   - More reliable connection for development and debugging
   - No additional wiring required - uses USB cable connection

2. **LED Status Indicators**
   - Full RGB LED support with color-coded status
   - Purple LED when ball is detected
   - White LED when no ball is detected
   - Blue LED during initialization
   - Provides immediate visual feedback on detection state

3. **Enhanced Color System**
   - Added `Color` class enum for better LED control
   - Supports: RED, GREEN, BLUE, WHITE, YELLOW, CYAN, PURPLE
   - Cleaner, more maintainable code structure

4. **Improved Exit Handling**
   - Exit pin configuration changed to `PULL_DOWN` (was `PULL_UP` in V2)
   - More reliable interrupt-based exit mechanism

5. **Continuous Operation**
   - Removed test-mode iteration limit
   - Runs continuously until exit signal received
   - Faster update rate (20ms sleep vs 200ms in V2)

6. **Better Data Transmission**
   - Smart USB connection checking before sending data
   - Silent operation when USB not connected (no print spam)
   - Exception handling for robust communication

## How It Works

### Detection Pipeline

1. **Calibration** - On startup, calculates focal length using a ball at known distance (40cm)
2. **Blob Detection** - Finds colored regions matching thresholds
3. **Shape Refinement** - Uses Hough transform for circles (ball) and rectangles (goals)
4. **Distance Calculation** - Computes real-world distance using similar triangles
5. **Position Mapping** - Converts to X/Y coordinates accounting for camera height

### Color Thresholds (LAB color space)

```python
"orange": (16, 100, 12, 127, 20, 127)   # Ball
"blue":   (0, 48, -7, 127, -128, -10)   # Goal
"yellow": (39, 100, -31, 3, 18, 127)    # Goal
```

### Physical Parameters

- Ball diameter: 4.3 cm
- Goal diameter: 64.0 cm
- Camera height: 13.0 cm (above ground)
- Max detection range: 200 cm
- Default focal length: 265.12 (or auto-calibrated)

## Usage

### Running V3

```python
from ball-identifierV3 import CameraDetection

# Initialize with exit pin and optional focal length
camera = CameraDetection(exit_pin="P3", focal_length=265.12)

# Run detection loop (exits via pin interrupt)
camera.run()
```

### Data Output Format

Data is sent via USB as `#`-delimited string:

```
ballX#ballY#blueX#blueY#yellowX#yellowY\n
```

Example: `12.5#45.3#0#0#-30.2#80.1\n`

- Coordinates in centimeters
- `0` indicates object not detected
- Values >200cm reported as `0`

## Hardware Setup

1. **OpenMV Camera** - H7 or similar
2. **Exit Pin** - Connect to P3 (pull-down configuration)
3. **USB Connection** - For power and data transmission
4. **Lighting** - Consistent lighting improves color detection

## Installation

1. Install [OpenMV IDE](https://openmv.io/pages/download)
2. Connect OpenMV camera via USB
3. Open `ball-identifierV3.py` in OpenMV IDE
4. Click "Run" or save to camera flash

## Calibration

On first run (or when `focal_length=0`):

1. Place orange ball exactly 40cm from camera
2. System will auto-detect and calibrate
3. Focal length will be printed and saved
4. Use this value in `CameraDetection()` for future runs

## Version Comparison

| Feature | V1 | V2 | V3 |
|---------|----|----|-----|
| Communication | UART | UART | USB VCP ✓ |
| LED Feedback | Basic | Basic | Full RGB ✓ |
| Exit Pin Pull | UP | UP | DOWN ✓ |
| Run Mode | Continuous | Test (50 iter) | Continuous ✓ |
| Update Rate | Variable | 200ms | 20ms ✓ |
| Status Output | Print | Print | USB only ✓ |

## Troubleshooting

**No ball detected:**
- Check lighting conditions
- Verify ball color matches threshold
- Adjust `ball_pixels_threshold` or `ball_area_threshold`

**Distance inaccurate:**
- Re-run calibration procedure
- Verify physical measurements (ball diameter, camera height)
- Ensure camera lens is clean and focused

**USB connection issues:**
- Check cable connection
- Verify USB VCP is enabled in OpenMV firmware
- Use OpenMV IDE serial terminal to monitor output

## Future Enhancements

- Dynamic threshold adjustment
- Multiple ball tracking
- Field orientation detection
- Kalman filtering for smoother tracking
- Configuration file support

## License

Open source - feel free to modify and adapt for your robotics projects.

## Author

Developed for robotics soccer applications using OpenMV platform.
