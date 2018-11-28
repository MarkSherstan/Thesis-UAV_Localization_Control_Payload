# Iterative Learning Control (ILC)

The following directory and all sub directories are part of MecE 788's term project.

## Modelling

Create a model for simulation in MATLAB / Simulink / Simscape environment.

## Data

Saved data to be referenced for the model.

## Testing

Code for data acquisition in C++ for arduino

* currentVoltage - Get current and voltage
* encoder - Output position data from an encoder
* forceSensor - Under development. Use FSR sensor to get force from clamps.
* main - Brings all programs together in one sketch

## Analysis

MATLAB code for reading in data for the model development, testing and checking stability, and implementing ILC.

## Use

Before using the twist model run:
```
main.m
```
as there are refined motor parameters and transfer functions required for the simulation.
