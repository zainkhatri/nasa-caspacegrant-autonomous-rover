# NASA Rover V2 with Enhanced Obstacle Avoidance and Machine Learning

**NASA Rover V2** is a project designed to showcase the integration of machine learning and sensor-based navigation in an autonomous robotic system. Built as a revamped version of the original V1 code, this rover incorporates advanced machine learning techniques to improve obstacle detection and decision-making capabilities. It demonstrates how cutting-edge technologies can be combined to create a robust and adaptable system capable of navigating complex terrains.

## Features

### Autonomous Navigation
- Utilizes GPS data to plot waypoints and navigate autonomously, adjusting its heading using real-time compass data.
- Dynamically calculates navigation vectors to ensure it stays on course.

### Enhanced Obstacle Avoidance
- Equipped with multiple ultrasonic sensors to detect obstacles in its path.
- Integrates a TensorFlow Lite model for real-time decision-making, helping the rover avoid obstacles and find the clearest path forward.

### Precision Motor Control
- Implements precise motor control using stepper motors for accurate movements.
- Features different speed modes for sharp turns, slow pivots, and full-speed navigation.

### Machine Learning Integration
- Uses AI to interpret sensor data and make intelligent navigation decisions.
- Capable of scanning its environment and recalibrating its path in case of blockages.

### Versatile Sensor Suite
- **Magnetometer:** Provides compass data for accurate heading adjustments.
- **Ultrasonic Sensors:** Detects obstacles from multiple directions (front, left, and right).
- **GPS:** Tracks real-time location and guides the rover to its next waypoint.

### Modular Design
- Easy to adapt for different terrains and missions by adding or modifying sensors and motors.
- Fully configurable constants for tweaking navigation and obstacle detection settings.

### Offline Functionality
- The TensorFlow Lite model and sensor integrations allow the rover to operate without constant external inputs, making it suitable for remote locations.

## Purpose
This project serves as an enhanced version of the original NASA Rover V1, incorporating machine learning to improve its performance in real-time obstacle avoidance and decision-making. It demonstrates the potential of combining AI with autonomous navigation for robotics applications in various fields, including space exploration, disaster response, and search-and-rescue missions.
