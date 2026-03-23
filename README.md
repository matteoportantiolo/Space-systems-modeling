# Space Systems Modeling

## Overview
This project presents the modeling and simulation of two key spacecraft subsystems:

- An active thermal control system based on deployable radiators
- A simplified Attitude Control System (ACS) for drag compensation

The work combines multi-domain physical modeling (thermal, mechanical, electrical) and control design, implemented using both causal (Matlab) and acausal (Modelica/Simscape) approaches.

## System Architecture

### Thermal Control System
The satellite thermal model is based on a lumped parameter approach with five nodes:
- Main body
- Two solar panels
- Two deployable radiators

Key features:
- Radiator emissivity varies with rotation angle
- Heat exchange includes:
  - Solar radiation
  - Deep space radiation
  - Conductive coupling between nodes
- Temperature regulation achieved through active control of radiator angle

### Radiator Actuation
- DC motor-driven hinge mechanism
- Electromechanical coupling:
  - Electrical circuit (R–L–back EMF)
  - Mechanical rotation dynamics
- Control input: proportional law based on body temperature

### Attitude Control System (ACS)
A simplified ACS is designed to compensate atmospheric drag:

Subsystems:
- Accelerometer (mass-spring-damper model)
- Voltage modulation (operational amplifier)
- Solenoidal valve (electromagnetic actuator)
- Ion thruster (mass flow → thrust generation)

The system dynamically adjusts thrust to match time-varying drag disturbances.

## Methods

### Causal Modeling (Matlab)
- Full nonlinear ODE system derived for both subsystems
- Multi-domain coupling:
  - Thermal
  - Mechanical
  - Electrical
- Stiff system solved using `ode15s`
- Control strategies:
  - Constant proportional gain
  - Adaptive gain tuning
- Parameter tuning for:
  - Thermal stability
  - Drag-thrust matching

### Acausal Modeling
Two environments were used:

#### Thermal System – Modelica (Dymola/OpenModelica)
- Block-based multi-physics modeling
- Custom components for:
  - Variable emissivity
  - Control logic
- Validation against Matlab results

#### ACS – Simscape
- Physical modeling using:
  - Mechanical translational components
  - Electrical circuits
  - Solenoid actuator
- Hybrid approach with embedded analytical models (thruster, drag)

## Results

### Thermal Control
- Stable regulation around target temperature (294.15 K)
- Oscillations reduced below 0.1% within required time
- Radiator angle converges to equilibrium configuration
- Strong agreement between causal and acausal models

### ACS Performance
- Effective compensation of atmospheric drag
- Steady-state thrust error on the order of 10⁻⁶ N
- Fast transient response (~15 s)
- Sensitivity to initial conditions highlighted

### Modeling Comparison
- Matlab (causal) and Modelica results are nearly identical
- Simscape model shows small discrepancies due to:
  - Solenoid modeling differences
  - Sensitivity to parameters

## Implementation
The project includes:
- Nonlinear ODE formulation of multi-domain systems
- Numerical integration of stiff dynamics
- Control system design and tuning
- Modelica and Simscape implementations
- Comparative validation between modeling approaches

## Key Concepts
- Lumped thermal modeling
- Radiative heat transfer
- Electromechanical systems
- DC motor dynamics
- Mass-spring-damper systems
- Ion propulsion modeling
- Stiff ODE systems
- Causal vs acausal modeling

## Author
Matteo Portantiolo  
MSc Space Engineering – GNC
