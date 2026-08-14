# olfaction_msgs

Standard ROS 2 message definitions for robotic olfaction sensors including gas detectors and anemometers.

## Overview

This package provides message types for working with olfactory sensing in robotics applications. It is part of the [GSExploration](../documents/knowledge-overview.md) gas source localization system.

## Message Types

### Anemometer.msg

Wind speed and direction measurements from ultrasonic or mechanical anemometers.

```
std_msgs/Header header      # Timestamp and frame_id
string sensor_label         # Sensor identifier
float32 wind_speed          # Wind speed in m/s
float32 wind_direction      # Clockwise UPWIND bearing in sensor frame (0 = North, π/2 = East)
```

**Usage Example:**

```python
from olfaction_msgs.msg import Anemometer

msg = Anemometer()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'anemometer'
msg.wind_speed = 1.5        # m/s
msg.wind_direction = 0.78   # radians (~45°)
```

### GasSensor.msg

Comprehensive gas sensor readings with technology, manufacturer, and calibration metadata.

```
std_msgs/Header header      # Timestamp and frame_id

# Sensor Info
uint8 technology            # TECH_MOX, TECH_PID, TECH_TDLAS, etc.
uint8 manufacturer          # MANU_FIGARO, MANU_ALPHASENSE, etc.
uint8 mpn                   # Model/Part Number identifier

# Measurement
float64 raw                 # Raw sensor reading
uint8 raw_units             # UNITS_PPM, UNITS_VOLT, UNITS_OHM, etc.
float64 raw_air             # Baseline reading in clean air
float64 calib_a             # Calibration constant A
float64 calib_b             # Calibration constant B
```

**Supported Technologies:**
| Constant | Value | Description |
|----------|-------|-------------|
| `TECH_MOX` | 1 | Metal Oxide Semiconductor |
| `TECH_AEC` | 2 | Amperometric Electrochemical |
| `TECH_PID` | 51 | Photoionization Detector |
| `TECH_TDLAS` | 53 | Tunable Diode Laser |

**Supported Units:**
| Constant | Value | Description |
|----------|-------|-------------|
| `UNITS_PPM` | 3 | Parts per million |
| `UNITS_PPB` | 4 | Parts per billion |
| `UNITS_VOLT` | 1 | Voltage |
| `UNITS_OHM` | 5 | Resistance |

### GasSensorArray.msg

Array of gas sensor readings for multi-sensor e-nose systems.

```
std_msgs/Header header
GasSensor[] sensors         # Array of gas sensor readings
```

### TDLAS.msg

Specialized message for Tunable Diode Laser Absorption Spectroscopy sensors.

## Installation

This package is built as part of the GSExploration workspace:

```bash
cd ~/Bio_ws
colcon build --packages-select olfaction_msgs
source install/setup.bash
```

## Dependencies

- `std_msgs` - Standard ROS 2 message types
- `rosidl_default_generators` - Message generation

## Integration

Used by the following GSExploration packages:

- `main_decision` - Sensor data subscription
- `gas_distribution_mapping` - GDM map updates
- `gsl_local_search` - Localization algorithms
- `GMRF-wind` - Wind field estimation

## License

GPL-3.0

## References

- [MAPIRlab olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs) - Original repository
