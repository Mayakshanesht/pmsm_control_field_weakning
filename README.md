# PMSM Control System

This repository contains a comprehensive implementation of Permanent Magnet Synchronous Motor (PMSM) control systems, including both non-salient and salient pole motor control strategies with field weakening capabilities.

Youtube Playlist: https://www.youtube.com/playlist?list=PLFjFRmQyEKQTvLAP-C8akt9MBvG3oWwLQ

## Project Structure

```
PMSM/
├── PMSM_model/           # PMSM motor model implementation
│   ├── PMSM.slx         # Simulink model of the PMSM motor
│   └── PMSM_machine.m   # MATLAB script for PMSM motor parameters
├── PMSM_control/        # Control logic for non-salient PMSM
│   ├── PMSM_control.m   # Main control script
│   └── PMSM_control_model.slx  # Simulink control model
└── Field_oriented_controller/  # Field weakening control for salient PMSM
    └── ...
```

## Control Architecture

The PMSM control system implements a cascaded control structure with four main loops:

### 1. Torque/Speed Control Loop (Outer Loop)
- Takes reference torque or speed commands from the Vehicle Control Unit (VCU) or driver input
- Generates reference currents (𝑖𝑑∗ and 𝑖𝑞∗) in the dq-reference frame
- Implements field weakening strategy for operation above base speed

### 2. Current Control Loop
- Compares actual motor currents with reference currents
- Implements PI controllers or Box controller for d-axis and q-axis current regulation
- Outputs reference voltages (𝑢𝑑∗, 𝑢𝑞∗) in the dq-reference frame
- Includes decoupling terms for independent control of d and q axes

### 3. Voltage Modulation (SVPWM)
- Converts reference voltages from dq-frame to abc-frame
- Implements Space Vector PWM (SVPWM) technique
- Generates PWM signals for the three-phase inverter

### 4. Inverter + Motor + Sensors
- Three-phase inverter applies the PWM voltages to the motor
- PMSM produces torque based on the applied voltages
- Feedback signals:
  - Current sensors (phase currents) → current control loop
  - Position/speed sensors (Hall effect, encoder) → speed/torque control loop

## Key Features

### Non-salient PMSM Control
- Standard field-oriented control (FOC) implementation
- Speed and torque control modes
- Current limiting and protection

### Salient PMSM Control with Field Weakening
- Advanced field weakening control for extended speed range
- MTPA (Maximum Torque Per Ampere) operation
- Flux weakening control for high-speed operation

## Getting Started

### Prerequisites
- MATLAB/Simulink (R2020b or later)
- Simulink Control Design Toolbox
- Simscape Electrical (for motor simulation)

### Running the Simulation
1. Open MATLAB and navigate to the project directory
2. Run `PMSM_machine.m` to load motor parameters
3. Open `PMSM_control_model.slx` for non-salient control or the respective model in Field_oriented_controller for salient pole control
4. Configure simulation parameters as needed
5. Run the simulation

## Documentation

For detailed documentation on the control algorithms and implementation, please refer to the following files:
- `PMSM_model/PMSM_machine.m` - Motor parameters and specifications
- `PMSM_control/PMSM_control.m` - Control algorithm implementation
- `Field_oriented_controller/` - Advanced control strategies for salient pole motors

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments
- Based on standard field-oriented control principles
- Implements industry-standard PMSM control techniques
- Designed for educational and research purposes
