# fixed-wing-sim

* Matlab implementation to simulate the non-linear dynamics of a fixed-wing glider. 
* Tools to extract linear time invariant (LTI) systems around equilibrium point.

## Installation and Configuration

* Adjust the paths to your FlightGear installation in `runFlightGear.bat` and `runFlightGear.m` in `ExperimentalCarrierSimulink/utilities`.
* To run `mainComputeLTI.m`, check the configuration section to make the necessary adjustments to run this script in your environment and with the desired parameters.

## Running the Simulation

1. Open in Matlab the Simulink project ExperimentalCarrierSimulink.prj. This opens:
..* Plant model
..* ExperimentalCarrier model
..* ExperimentalCarrier_longitudinal model
..* ExperimentalCarrier_lateral model
..* FlightGear

# Appendix

## Test Flight Data

Data from test flight with physical model:

* Center of gravity: 92mm from leading edge of the wing.
* Weight: fully equipped with on-board computer (Tinkerforge) and 3-cell li-po and MVVS electric motor -> 1.56 kg
* Average velocity measured with GPS: ~45km/h.
