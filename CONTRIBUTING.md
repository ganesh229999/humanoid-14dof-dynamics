# Contributing to humanoid-14dof-dynamics

Thank you for your interest in contributing! Here's how to get involved.

## Priority Areas

The following extensions are most needed and will be reviewed quickly:

| Area | Description | Difficulty |
|------|-------------|------------|
| 🤖 ROS2 Bridge | Publish joint states, subscribe to trajectory actions | Medium |
| 🌍 Gazebo | URDF export + Gazebo simulation launch | Medium |
| ⚡ C++ Port | Real-time RNE core for embedded deployment | Hard |
| 📐 Contact Dynamics | Friction, impact, grasp force modelling | Hard |
| 🧪 Unit Tests | MATLAB unit test framework for RNE validation | Easy |
| 📈 More Tasks | Additional manipulation scenarios (door handle, tool use) | Easy |

## Getting Started

1. **Fork** the repository
2. **Clone** your fork: `git clone https://github.com/YOUR_USERNAME/humanoid-14dof-dynamics`
3. **Create a branch**: `git checkout -b feature/ros2-bridge`
4. Run `startup.m` in MATLAB to configure your environment
5. Make your changes with clear comments
6. **Test** thoroughly — validate against analytical solutions where possible
7. Open a **Pull Request** with a clear description

## Code Style Guidelines

- Function names: `camelCase` (matching existing MATLAB conventions)
- All functions must have a **header comment block** with: purpose, inputs, outputs, algorithm notes
- Use `%% Section Headers` for logical blocks within long functions
- Avoid toolbox dependencies — keep everything pure MATLAB
- Validate new dynamic functions with energy conservation or analytical comparison

## Reporting Issues

Please include:
- MATLAB version
- Operating system
- Exact error message + stack trace
- Minimal reproducible example

## Questions?

Open a GitHub Discussion or email: deshmukhg21@gmail.com
