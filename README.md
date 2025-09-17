# 🦖 Six-Legged Robot Gait Simulation in MATLAB

> A MATLAB-based simulation project for a six-legged (hexapod) robot, leveraging the **Robotics System Toolbox** for kinematic modeling, trajectory generation, and dynamic visualization.

This repository implements gait planning, workspace analysis, and joint-space trajectory profiling (including velocity, acceleration, and jerk) for a hexapod robot. Designed for educational, research, and prototyping purposes.

---

## 📦 Requirements

- **MATLAB R2023b** (or compatible version)
- **Robotics System Toolbox** (required for kinematic chains, transformations, and trajectory utilities)
- Optional: Simulink (not required for core scripts)

> 💡 *Ensure Robotics Toolbox is installed via MATLAB Add-Ons → Get Add-Ons → Search “Robotics System Toolbox”*

---

## 🗂️ Project Structure

| File               | Description |
|--------------------|-------------|
| `robot.m`          | Main gait simulation script — animates the full walking cycle of the hexapod. |
| `rotation_matrix.m`| Helper function to compute 3D rotation matrices (used in `worktrack.m` for coordinate transformations). |
| `workspace.m`      | Visualizes the 3D reachable workspace of a single leg using forward kinematics. |
| `worktrack.m`      | Plots joint-space trajectories over one gait cycle, including position, velocity, acceleration, and jerk profiles. |

---
