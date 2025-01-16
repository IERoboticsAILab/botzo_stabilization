# IMU Stabilization

This project implements IMU-based stabilization using MPU-6050 sensor data and rotation matrices. It includes both a real-time implementation and a test simulation environment.

## Setup

1. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Unix/macOS
# or
.\venv\Scripts\activate  # On Windows
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Project Structure

- `imu_stabilization.py`: Main implementation for real-time IMU data processing
- `test_imu_stabilization.py`: Simulation environment for testing without hardware
- `requirements.txt`: Project dependencies

## Usage

### Testing Environment
To run the simulation without actual IMU hardware:
```bash
python test_imu_stabilization.py
```

### Real Hardware Implementation
To run with actual MPU-6050 hardware:
```bash
python imu_stabilization.py
```

Press Ctrl+C to stop either program.

## Notes
- The real hardware implementation requires an MPU-6050 IMU sensor connected via I2C
- The test environment simulates oscillating motion patterns for testing the rotation matrix computations