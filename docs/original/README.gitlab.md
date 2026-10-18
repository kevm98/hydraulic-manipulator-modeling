
# Mechanical_Hydraulic Manipulator Simscape Project

## Overview

This project is a **MATLAB Simulink / Simscape-based simulation** focused on modeling, simulating, and analyzing a **hydraulic robotic arm system** using physically realistic components. The goal is to bridge **CAD/URDF-based mechanical structure** with **hydraulic dynamics and control-oriented simulation**, enabling realistic testing of motion, constraints, and control strategies before deployment or hardware integration.

The project is designed to be **modular, extensible, and control-ready**, making it suitable for research, prototyping, and further integration with advanced control approaches such as MPC or data-driven control.

---

## Key Features

* Physically accurate **Simscape Multibody** model
* Integration of **hydraulic actuation dynamics**
* Support for **URDF / CAD-based geometry**
* Joint limits, kinematic constraints, and realistic motion
* Ready for **trajectory tracking and controller integration**
* Structured model hierarchy for scalability

---

## System Architecture

The overall system consists of the following layers:

1. **Mechanical Structure**

   * Imported rigid bodies and joints (e.g., revolute joints)
   * Properly defined joint frames and reference axes
   * Mass, inertia, and geometry based on CAD/URDF data

2. **Hydraulic Subsystem**

   * Hydraulic cylinders driving selected joints
   * Pressure, flow, and actuator force modeling
   * Parameterized components for tuning and realism

3. **Actuation & Interfaces**

   * Force/torque application via Simscape interfaces
   * Signal-based inputs for valves or actuators
   * Sensor outputs for joint position, velocity, and force

4. **Control Layer (Expandable)**

   * Placeholder blocks for classical or advanced controllers
   * Designed for closed-loop simulation

---

## Folder Structure

```
MySimscapeProject/
│
├── models/
│   ├── main_simscape_model.slx      # Top-level Simulink model
│   ├── multibody/                   # Mechanical structure
│   ├── hydraulics/                  # Hydraulic subsystem
│
├── data/
│   ├── parameters.m                 # System parameters
│   ├── joint_limits.m               # Joint and workspace limits
│
├── urdf_cad/
│   ├── model.urdf                   # Exported URDF
│   └── meshes/                      # STL or mesh files
│
├── scripts/
│   ├── init_model.m                 # Model initialization
│   ├── run_simulation.m             # Simulation launcher
│
├── results/
│   └── simulation_outputs/          # Logged results
│
└── README.md
```

---

## Requirements

* MATLAB (R2022b or newer recommended)
* Simulink
* Simscape
* Simscape Multibody
* (Optional) Simscape Fluids

---

## Getting Started

1. Clone or download the project folder
2. Open MATLAB and set the project root as the working directory
3. Run the initialization script:

   ```matlab
   init_model
   ```
4. Open the main model:

   ```matlab
   open_system('Mulag_FME_500_with_cylinder.slxc')
   ```
5. Start the simulation. Upon running the model, a **Simscape Multibody Mechanics Explorer** window will automatically open.
6. Use the Mechanics Explorer to **visualize the plant model**, observe joint motions, link interactions, and overall system behavior in 3D.

The Mechanics Explorer serves as the primary tool for validating kinematics, joint limits, and motion realism during simulation.

---

## Simulation Capabilities

* Joint-level motion visualization
* Hydraulic pressure and flow analysis
* Force and torque monitoring
* Workspace and reach validation
* Trajectory feasibility testing

---

## Design Philosophy

* **Physical realism first**: dynamics are modeled explicitly, not approximated
* **Modularity**: mechanical, hydraulic, and control layers are decoupled
* **Research-ready**: suitable for experimentation and controller development
* **Extensible**: easy to add sensors, controllers, or disturbances

---

## Future Extensions

* Closed-loop MPC or DeePC control
* Real-time execution support
* Parameter identification from experimental data
* Fault injection and robustness analysis
* Hardware-in-the-loop (HIL) integration

---

## Author

Kevin Kudakwashe Murera

---

## License

This project is intended for academic and research use. Licensing can be adapted depending on distribution requirements.

---

## Notes

* Ensure joint frames are correctly aligned before importing URDF models
* Validate hydraulic parameters to avoid numerical stiffness
* Use smaller step sizes for high-fidelity simulations

---

Feel free to extend or adapt this project for your own research or applications.
