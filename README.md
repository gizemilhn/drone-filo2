# Drone Fleet Delivery Optimization System

A Python-based system for optimizing drone fleet delivery operations under various constraints.

## Features

- Dynamic drone fleet management
- Delivery point optimization
- No-fly zone handling
- Multiple optimization algorithms (A*, CSP, Genetic Algorithm)
- Interactive visualization
- Comprehensive reporting

## Installation

1. Clone the repository
2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

Run the main application:
```bash
python main.py
```

### Input Methods

1. GUI Interface:
   - Add/Edit drones
   - Add/Edit delivery points
   - Add/Edit no-fly zones

2. JSON File:
   - Load configuration from `data.json`

### Output

- Interactive visualization of drone paths
- Detailed delivery reports
- Performance metrics

## Project Structure

- `drone.py`: Drone class and management
- `delivery.py`: Delivery point handling
- `zone.py`: No-fly zone management
- `routing.py`: Path finding algorithms
- `optimizer.py`: CSP and GA implementations
- `visualizer.py`: Visualization tools
- `main.py`: Main application entry point

## Test Scenarios

1. Small Scale:
   - 5 drones
   - 20 deliveries
   - 2 no-fly zones

2. Large Scale:
   - 10 drones
   - 50 deliveries
   - 5 dynamic no-fly zones

## License

MIT License 