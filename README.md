# Fixed-Wing Glider Simulation

An example of a non-linear flight dynamics simulation for a unmanned aerial glider with a wingspan of 1.5m. The simulation is implemented with Matlab Simulink and uses FlightGear [[1]](#flightgear) for visualization purposes. 

In addition to existing Simulink examples from the Mathworks documentation, this implementation shows how to:

1. Compute required aerodynamic coefficient tables using Tornado [[2]](#tornado), an implementation of the [vortex lattice method](https://en.wikipedia.org/wiki/Vortex_lattice_method). 
2. Find the trimmed gliding state and deduce longitudinal and lateral linear time invariant systems ([LTI](https://en.wikipedia.org/wiki/Linear_time-invariant_theory)) for the trimmed state according to text book definitions such as the one given in [[3]](#caughey).

## Overview

Airframe
Rudder actions
Airfoil

### Airframe

### Tornado Setup

### Simulink Models

### FlightGear

## Applications

### Non-Linear Flight Simulation

### Longitudinal and Lateral LTI of Trimmed Gliding State

## Installation and Configuration

* Adjust the paths to your FlightGear installation in `runFlightGear.bat` and `runFlightGear.m` in `ExperimentalCarrierSimulink/utilities`.
* To run `mainComputeLTI.m`, check the configuration section to make the necessary adjustments to run this script in your environment and with the desired parameters.

## Running the Simulation

1. Open in Matlab the Simulink project ExperimentalCarrierSimulink.prj. This opens:
  * Plant model
  * ExperimentalCarrier model
  * ExperimentalCarrier_longitudinal model
  * ExperimentalCarrier_lateral model
  * FlightGear

# Appendix

## Test Flight Data

Data from test flight with physical model:

* Center of gravity: 92mm from leading edge of the wing.
* Weight: fully equipped with on-board computer (Tinkerforge) and 3-cell li-po and MVVS electric motor -> 1.56 kg
* Average velocity measured with GPS: ~45km/h.

# References

[1] <a name="flightgear"></a> [FlightGear](http://www.flightgear.org/).  
[2] <a name="tornado"></a> Melin, Tomas. [Tornado](http://tornado.redhammer.se/).  
[3] <a name="caughey"></a> Caughey, David A. [Introduction to Aircraft Stability and Control](https://courses.cit.cornell.edu/mae5070/Caughey_2011_04.pdf).  
