# Fixed-Wing Glider Simulation

This repository provides an example of a non-linear fixed-wing glider simulation with Matlab Simulink and FlightGear using tables of aerodynamic coefficients computed with Tornado [1](#abcd)., an implementation of the Vortex Lattice Method. 

Furthermore, the implementation also contains scripts to deduce the longitudinal and lateral linear time invariant systems (LTI) for the trimmed gliding state.

## Overview

Airframe
Rudder actions
Airfoil

## Applications


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

<a name="abcd"></a> test

[1] http://tornado.redhammer.se/
