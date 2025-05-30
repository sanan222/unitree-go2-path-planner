# Real-time Local Path Planning for Unitree Go2 Legged Robot

## Overview

This project implements and compares three local path planning algorithms for a legged robot in ROS2, visualized in RViz and tested in simulation with convex and concave obstacles. The tasks explore edge-following techniques, Bug0 and Bug1 algorithms, with various enhancements such as smoothing, waypoint filtering, and robust recovery behaviors.

---

## Tasks Implemented

### Task 1 – Counterclockwise Edge Following

An edge-following algorithm guides the robot in a counterclockwise direction around obstacles.

#### Key Components:
- **Edge Detection:** Local map edges filtered using curvature and refined via Chaikin’s smoothing.
- **Waypoint Generation:** Waypoints are published unless the robot is farther than 1.5m from the nearest edge.
- **Motion Logic:** Moves in counterclockwise direction (`moving_forward = False`).
- **Edge Extension:** Adds 50cm extensions when nearing the edge ends.
- **Stuck Detection:** Activates recovery motion if movement is <10cm for 2 seconds or no waypoints found after 20 iterations.

#### Chaikin’s Algorithm:
Iterative corner-cutting algorithm to smooth polylines:
```math
Q_i = \frac{3}{4}P_i + \frac{1}{4}P_{i+1}, \quad R_i = \frac{1}{4}P_i + \frac{3}{4}P_{i+1}
```
This process is repeated to achieve a smoother curve.

---

### Task 2 – Bug0 Algorithm

A reactive path planner using a state machine to alternate between direct motion and obstacle avoidance.

#### Features:
- **Two-State Machine:** `GO_TO_GOAL` vs `AVOID_OBSTACLE`
- **Edge Processing:** Filters and merges line segments based on curvature and collinearity.
- **Edge Smoothing:** Chaikin’s algorithm applied again for smooth obstacle contours.
- **Waypoint Smoothing:** Uses exponential moving average for stable motion.
- **Obstacle Clearance:** Checks goal direction for free path before transitioning states.

---

### Task 3 – Bug1 Algorithm

A more complex path planner that records the **hit point** when an obstacle is first encountered, follows the boundary, then resumes to the **leave point** closest to the goal.

#### Modes:
- `go_to_goal`
- `follow_boundary_to_hit_point`
- `follow_boundary_to_leave_point`
- `goal_reached`

#### Special Behaviors:
- **Hit Point Detection:** Records when edge is too close.
- **Leave Point Computation:** Tracks the point on the obstacle closest to the goal.
- **Recovery Logic:** Resets or rotates robot (±60°) if progress stalls or path diverges.
- **Angular Smoothing Only:** Retains only significant edge direction changes (better for path consistency).

---

## Visual Outputs

- RViz visualizations show obstacle boundary tracking, hit and leave point markers, and goal progression.
- Marker colors: Green = Goal, Red = Hit Point, Blue = Leave Point

---

## Technologies Used

- ROS2 (Robot Operating System)
- RViz for visualization
- Python/C++ for ROS node implementations
- Chaikin smoothing for curve generation

---

## Future Improvements

- Integration of global planning and SLAM modules
- Real-world deployment on actual legged platforms
- Better tuning of hysteresis and obstacle margin parameters

---

## Reference

Chaikin, G. M. (1974).  
*An algorithm for high-speed curve generation.*  
Computer Graphics and Image Processing, 3(4), 346–349.  
[Link to Paper](https://www.sciencedirect.com/science/article/pii/0146664X74900288)

---

© Team 12 – UCL COMP0244, 2025

