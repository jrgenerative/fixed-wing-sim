# Simulation of a Fixed-Wing Unmanned Aerial Glider

An example of a non-linear flight simulation for a unmanned aerial glider with a wingspan of 1.5m. The simulation is implemented with Matlab Simulink and uses [FlightGear](http://www.flightgear.org) for visualization purposes. 

In addition to existing Simulink examples from the Mathworks documentation, this implementation shows how to:

1. Compute the required aerodynamic coefficient tables using [Tornado](http://tornado.redhammer.se/) an implementation of the [vortex lattice method](https://en.wikipedia.org/wiki/Vortex_lattice_method) (VLM). For more information on the Tornado implementation, see also [[1]](#tornado). 
2. Find the trimmed gliding state and deduce longitudinal and lateral linear time invariant systems ([LTI](https://en.wikipedia.org/wiki/Linear_time-invariant_theory)) for the trimmed state according to text book definitions such as the one described in [[2]](#caughey).

Simulation | Real Flight
----------| ------------
<img src="./figures/FlightGear03.png" width="400"> | <img src="./figures/Airframe02.png" width="400">
Visualization of the Simulink simulation with FlightGear | Test flight with the real airframe

Lateral LTI | Longitudinal LTI
-----------|-------------
<img src="./results/mainComputeLTIs/lateral.png" width="400"> | <img src="./results/mainComputeLTIs/longitudinal.png" width="400">
Characteristics of the corresponding lateral LTI system | Characteristics of the corresponding longitudinal LTI system

## Airframe

The airframe has a twin-boom fuselage and a wing with upward cranked tips. The total wing span is 1.5m and the take-off weight is 1.56kg (actual glider equiped with on-board computer and temporarily installed electric motor for testing / take-off). Center of gravity has been found to be at 92mm from the leading edge of the main wing. Via GPS measurements a gliding velocity of about 45km/h was confirmed (at roughly zero elevator deflection). The glider uses two actuators: elevator and rudder. The rudder is asymmetrically attached to the left of the two vertical stabilizers.

Below is the airframe as defined for the vortex lattice method computation with Tornado:

Wing partition layout | VLM discretization
---------|----------
 <img src="./results/mainComputeCoefficients/TornadoAirframe1.png" width="400"> | <img src="./results/mainComputeCoefficients/TornadoAirframe2.png" width="400">

Airfoil JR001 | Example pressure distribution computed by Tornado 
--------------|---------------------
<img src="./airfoil/JR001.png" width="400"> | <img src="./figures/pressure_distribution_visualization_tornado.png" width="400">

The airfoil JR001 features a planar pressure side which simplifies the build procedure and provides. The profile was designed to work well with low Reynold's numbers and to provide friendly stall characteristics. It wasn't designed with gliding performance in mind.

Further drawings related to the airframe can be found [here](./Tornado/aircraft/ExperimentalCarrier.svg) and [here](./figures/StabilityAxisReferenceForTrimmedGliding.svg). The Tornado definition of the airframe is [here](./Tornado/aircraft).

## Applications

The codebase and the provided Simulink models can be used to:

1. Compute aerodynamic properties and coefficients using the Tornado VLM implementation.
2. Run a non-linear flight simulation using previously computed coefficient matrices.
3. Extract the linear time-invariant systems for the trimmed gliding state.

### Computation of Aerodynamic Coefficients

neutral point

force, moments, damping, moments induced by rudder deflection.

### Non-Linear Flight Simulation

TODO

### Longitudinal and Lateral LTIs for Gliding Equilibrium

TODO

## Results

TODO

Get some plots principal Tornado results.

## Installation and Configuration

TODO

* Adjust the paths to your FlightGear installation in `runFlightGear.bat` and `runFlightGear.m` in `ExperimentalCarrierSimulink/utilities`.
* To run `mainComputeLTI.m`, check the configuration section to make the necessary adjustments to run this script in your environment and with the desired parameters.

## Running the Simulation

TODO

1. Open in Matlab the Simulink project ExperimentalCarrierSimulink.prj. This opens:
  * Plant model
  * ExperimentalCarrier model
  * ExperimentalCarrier_longitudinal model
  * ExperimentalCarrier_lateral model
  * FlightGear

# References
  
[1] <a name="melin"></a> Melin, Tomas. [Tornado, a vortex lattice MATLAB implementation for Linear Aerodynamic Wing applications](https://www.researchgate.net/profile/Tomas_Melin/publication/238671899_A_Vortex_Lattice_MATLAB_Implementation_for_Linear_Aerodynamic_Wing_Applications/links/0deec5302051604432000000.pdf), Masters thesis, Royal Institute of Technology (KTH),Sweden, December 2000.  
[2] <a name="caughey"></a> Caughey, David A. [Introduction to Aircraft Stability and Control](https://courses.cit.cornell.edu/mae5070/Caughey_2011_04.pdf).  
