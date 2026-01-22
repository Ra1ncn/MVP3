# MVP3 Revamp

A multi-agent path planning and coordination system with real-time vision processing and simulation capabilities.

## Project Overview

This project is developed and tested under **Python 3.12**. It provides a comprehensive system for multi-robot coordination, featuring both simulation and real-world modes with real-time camera vision processing.

It supports two execution modes:
- **Simulation Mode** - Virtual environment for testing algorithms without hardware
- **Real-World Mode** - Live camera input with actual robot coordination

---

## System Requirements

- **Python 3.12**
- **Camera** (for real-world mode): 480  680 resolution
- **Operating System**: Windows/Linux/macOS

### Dependencies

All required packages are listed in 
equirements.txt. Key dependencies include:
- PyQt6 - GUI framework
- OpenCV - Vision processing
- ZMQ - Network communication
- NumPy - Numerical computing
- Pygame/Pygame_gui - Graphics and UI components
- PySerial - Serial communication
- Munkres - Assignment algorithm

---

## Project Structure

```text
MVP3_Revamp/
├── qt_gui.py                 # Main GUI application
├── comm.py                   # Serial communication module
├── vision_pub.py             # Camera / ArUco detection publisher
├── positionZMQSub.py         # Position subscriber via ZMQ
├── DDR.py                    # Robot control module
├── utils.py                  # Utility functions
├── algorithms/               # Path planning algorithms
│   ├── mrpp.py               # Multi-robot path planning
│   ├── mrpp_b.py             # Alternative MRPP implementation
│   ├── rvo2.py               # Reciprocal Velocity Obstacle
│   └── rvobin/               # RVO library binaries
├── patterns/                 # Pre-defined movement patterns
│   ├── circle1.py
│   ├── circle2.py
│   ├── figure8_2.py
│   └── template.py
├── firmware/                 # Arduino firmware
│   ├── xiao_ap.ino
│   └── xiao1-8.ino
├── projects/
│   └── multipath/            # Advanced multi-agent solving
│       ├── advanced/         # Advanced graph algorithms (Java)
│       └── ILP/              # Integer Linear Programming solvers (Java)
├── test/                     # Test utilities
└── gurobi/                   # Gurobi optimization files



## Running the Project

### Simulation Mode

Run the GUI in simulation mode (no hardware required):

`ash
python qt_gui.py -s
`

This mode allows you to:
- Test algorithms without physical robots
- Simulate multi-agent path planning
- Visualize movement patterns
- Debug coordination logic

### Real-World Mode

Run the GUI with live camera input and real robot coordination:

`ash
python qt_gui.py
`

**Requirements for real-world mode:**
- Connected camera with 480  680 resolution
- Serial connections to robots
- ArUco markers for position tracking

---

## Key Components

### Vision System (vision_pub.py)
- Captures video from camera at configurable FPS (default: 30 FPS)
- Detects ArUco markers and extracts corner coordinates
- Publishes marker data via ZMQ network (port 5556)
- Format: id x0 y0 x1 y1 x2 y2 x3 y3

### Communication System (comm.py)
- Serial communication with robots via USB
- CRC8 checksum validation
- Command framing and protocol handling

### Position Tracking (positionZMQSub.py)
- Subscribes to vision data via ZMQ
- Tracks robot positions in real-time
- Coordinates multi-agent state

### Robot Control (DDR.py)
- Differential drive robot control
- Velocity and heading commands
- Motor synchronization

### Path Planning (algorithms/)
- **MRPP** - Multi-Robot Path Planning
- **RVO2** - Collision avoidance using Reciprocal Velocity Obstacles
- Support for various optimization methods

---

## Usage Examples

### Running Simulation

`ash
# Activate environment
conda activate MVP3_1

# Run simulation mode
python qt_gui.py -s
`

### Running with Real Hardware

`ash
# Activate environment
conda activate MVP3_1

# Ensure camera is connected and robots are powered on
python qt_gui.py
`

---

## Configuration

- **Vision Port**: 5556 (configured in vision_pub.py)
- **Camera Resolution**: 480  680 pixels
- **Default FPS**: 30 frames per second
- **Serial Communication**: USB FTDI or CH340 adapters
