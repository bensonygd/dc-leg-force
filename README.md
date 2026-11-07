# DC Motor Single-Leg Simulation

A simple MATLAB and Simulink model of a current-controlled DC motor driving one rigid leg on a vertically moving carriage. The foot slides on a horizontal ground plane, and Simscape Multibody provides the ground support force directly from its contact model.

This repository contains the physical simulation, model-building code, and editable parameters for a study supporting **Construction and Deployment of a Torque-Controlled Modular Robot Motor Testing System**.

## Files

```text
simulation/
  leg_parameters.m       Editable mechanical, electrical, and simulation parameters
  build_leg_model.m      Build or rebuild the Simulink model
  dc_leg.slx              Ready-to-run model with embedded parameters
  run_simulation.m        Run the model, plot signals, and export raw data
README.md
.gitignore
.gitattributes
```

## Requirements

Built and tested in **MATLAB R2026a** with Simulink, Simscape, and Simscape Multibody. Simscape Electrical, Motor Control Blockset, and a C/C++ compiler are not required; the DC electrical circuit is implemented with basic Simulink blocks.

## Run the supplied model

From the repository root:

```matlab
run('simulation/run_simulation.m')
```

The script runs the supplied model for 3 seconds and plots ground support force, leg angle, current, and carriage height. It creates a local `results/` folder containing:

- `measurements.csv`: time, current, angle, angular velocity, angular acceleration, carriage height, voltage, and electromagnetic torque.
- `ground_force.csv`: time and the ground contact block's force output.
- `parameters.json`: the parameter values embedded in the model that was actually simulated.
- `simulation.png`: a plot of the raw simulation signals.

Generated results and Simulink caches are excluded by `.gitignore`. The `.slx` can also be opened and run directly without an initialization script.

## Change parameters and rebuild

Edit `simulation/leg_parameters.m`, then run:

```matlab
addpath('simulation')
build_leg_model
run('simulation/run_simulation.m')
```

Rebuilding replaces `simulation/dc_leg.slx` and embeds the updated parameter structure. Editing the parameter file alone does not change an already-built model.

## Default experiment

All defaults are illustrative assumptions, not calibrated hardware measurements.

| Quantity | Default |
| --- | ---: |
| Total moving mass, including the leg | 10 kg |
| Leg mass / carriage mass | 0.5 kg / 9.5 kg |
| Leg length / square cross-section width | 0.16 m / 0.01 m |
| Joint-to-center-of-mass distance | 0.08 m |
| Leg inertia about the joint | 0.0042708333 kg m^2 |
| Initial angle / angular velocity | 60 degrees / 0 rad/s |
| Target electromagnetic torque | 8 N m |
| Assumed joint viscous damping | 11 N m s/rad |
| Motor torque / back-EMF constants | 1 N m/A / 1 V s/rad |
| Armature resistance / inductance | 1 ohm / 0.01 H |
| Average drive voltage limit | 24 V |
| Ground contact stiffness / damping | 100000 N/m / 500 N s/m |
| Simulation duration / logging interval | 3 s / 1 ms |

The current controller follows a constant electromagnetic torque target; available joint torque also depends on viscous damping. The relatively large assumed damping keeps the demonstration slow. There is no position target or stop at 90 degrees.

The initial contact spring is preloaded to support the moving mass. Initial motor current is set to the static support value and then follows the constant 8 A target.

## Model overview

```text
Target current -> current PI -> limited average voltage -> DC motor circuit
                                                            |
                                                            v
                                              motor torque -> rotating leg
                                                            |
                                                            v
                                        vertical carriage + foot-ground contact
```

The DC circuit uses `Le*dI/dt = V - R*I - Ke*speed` and `electromagnetic torque = Kt*I`. Motor current is an integrated electrical state. The mechanical motion is solved by Simscape Multibody.

World Z points upward. The joint rotates about world Y. The leg angle increases downward from horizontal, so 90 degrees is vertical. The foot can slide horizontally without friction while the carriage is constrained to vertical translation.

## Read the ground support force directly

Inside `Leg and ground`, the **Spatial Contact Force** block connects the foot point to the ground plane. Its built-in smooth spring-damper law produces the contact force. The `fnm` output passes through a **PS-Simulink Converter** and is logged as `F_ground`:

```matlab
out = sim('dc_leg', 'ReturnWorkspaceOutputs', 'on');
time_s = out.F_ground.Time;
ground_force_N = out.F_ground.Data;
```

For this single contact on horizontal ground, `fnm` is the magnitude of the upward support force. It is a simulated contact-model output, not a physical sensor measurement.

## Example behavior and model scope

The default run records 3001 samples, rotates the leg from 60 to approximately 121.51 degrees, and produces ground support of approximately 94.12 to 99.31 N.

The model assumes a rigid uniform leg, an ideal frictionless vertical guide, a frictionless sliding foot, one foot-ground contact, and prescribed viscous joint damping. Joint motion is sensed ideally. PWM switching, FOC, gearing, rotor inertia, and electrical losses beyond armature resistance are omitted. This is a simplified DC-drive simulation; it does not validate the thesis hardware or its actual three-phase motor controller.

## Documentation

- [Spatial Contact Force](https://www.mathworks.com/help/sm/ref/spatialcontactforce.html)
- [PS-Simulink Converter](https://www.mathworks.com/help/simscape/ref/pssimulinkconverter.html)
- [To Workspace](https://www.mathworks.com/help/simulink/slref/toworkspace.html)
- [DC motor equations](https://www.mathworks.com/help/simscape-electrical/ref/dcmotor.html)
