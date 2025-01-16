# TurtleBot Motion Control Using ROS

This repository contains a series of personal assignments focused on controlling a TurtleBot using the Robot Operating System (ROS). The tasks involve creating movement patterns, implementing dynamic stopping mechanisms, and utilizing ROS features like publishers, subscribers, services, and launch files.

## Features
- **Robot Movement Patterns:**
  - Circular motion, square traversal, and straight-line movement.
  - Multi-phase motion (circle, stop, move forward, stop).
- **Dynamic Stopping:**
  - Distance-based stopping using Euclidean calculations.
- **ROS Integration:**
  - Implementation of publishers, subscribers, services, and launch files.
- **Bash and Python Scripts:**
  - Scripts to trigger and control TurtleBot behaviors.

## File Details
- **Python Scripts:**
  - `ToobaZahid_publisher.py`: Controls robot movement in a circle.
  - `ToobaZahid_publisher_line.py`: Moves the robot in a straight line.
  - `ToobaZahid_pubsub.py`: Combines publishing and subscribing for distance-based stopping.
- **Bash Scripts:**
  - `bash_dancing_turtle_echo.sh`: Executes Python scripts to control movement patterns.
- **Launch Files:**
  - `ToobaZahid_publisher_line.launch`: Launches the straight-line movement script.
  - `ToobaZahid_Pubsub.launch`: Launches the pub-sub system for distance-based stopping.

## Observations
- The robot successfully performed tasks like:
  - Moving in predefined patterns (circle, square, straight).
  - Dynamically stopping after a distance threshold.
  - Multi-phase motion sequences.
