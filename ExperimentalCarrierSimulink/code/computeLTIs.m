%

% @
% Copyright (C) 2017 Jonas Ruesch
%
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
% @

function [ltiLongitudinal, ... % the longitudinal linear time invariant system
    trimmedStateVariablesLongitudinal, ... % [u; w; q; theta] for the trimmed state
    ltiLateral, ... % the lateral linear time invariant system
    trimmedStateVariablesLateral, ... % [v; p; phi; r] for the trimmed state
    Cm_longitudinal, ... % array of Cm for each alpha of the alpha sweep
    Cn_lateral ... % array of Cn for each beta of the beta sweep
    ] = computeLTIs(...
    tornadoDirectory, ... % absolute path to the Tornado root directory
    nameTornadoAircraftGeometry, ... % the name of the Tornado geometry file
    nameLongitudinalModel, ... % the name of the Simulink long. model.
    nameLateralModel,... % the name of the Simulink lateral model.    
    alphaStart, alphaEnd, ... % range in degrees over which coefficients are computed for the longitudinal model
    numAlphas, ... % the number of alphas to sample
    betaStart, betaEnd, ... range in degrees over which coefficients are computed for the lateral model
    numBetas, ... the number of betas to sample 
    estimatedVelocity, ... % velocity in m/s for which aerodyn. coeffs. are computed
    velocityDeviationTol, ... % tolerated difference of the estimated velocity with regards to the found trimmed velocity.
    airDensity, ...  % kg/m^3 -> 1.225 at sea level
    elevatorFlapIndex, ... % Which is the elevator? Tornado flap index.
    rudderFlapIndex, ... % Which is the rudder? Tornado flap index.
    mass, ... % aircraft mass
    centerOfGravity, ... % center of gravity in Tornado aircraft reference frame (x-axis extends aft, thus usually positive value).
    InertiaTensor) % inertia tensor
% Finds the natural gliding state (trimmed gliding), linearizes the passed
% longitudinal and lateral models for that state.
% Assumptions:
% - aircraft geometry has 2 elevator and rudder flaps
% - aircraft geometry defined the first wing as the main wing
% - dihedral projection is neglected for total wingspan.
% - mean chord is geo.c(1)
% Returns the longitudinal and lateral linear time invariant systems.
% CAUTON: only considered inputs are elevator and rudder (no throttle, no
% ailerons).

% Define an initial Tornado state with given estimated velocity.
state.AS=      estimatedVelocity; %Airspeed m/s
state.alpha=   0.0;       %Angle of attack, radians
state.betha=   0;         %Angle of sideslip, radians
state.P=       0;         %Rollrate, rad/s
state.Q=       0;         %pitchrate, rad/s
state.R=       0;         %yawrate, rad/s
state.adot=    0;         %Alpha time derivative rad/s
state.bdot=    0;         %Betha time derivative rad/s
state.ALT=     0;         %Altitude, m
state.rho=     airDensity;%Desity, kg/m^3
state.pgcorr=  0;         %Apply prandtl glauert compressibility correction
     
% Get some properties from the aircraft geometry
cd(tornadoDirectory);
settings=config('startup');
% Load the aircraft
cd(settings.acdir);
load(nameTornadoAircraftGeometry); % aircraft geometry is now in variable 'geo'.
wingspan = 2*sum(geo.b(1,:)); % assume first wing is the main wing and symmetric (discard dihedral)
mean_chord = geo.c(1);
disp(['Wingspan computed from Tornado aircraft geometry: ' num2str(wingspan)]);
disp(['Mean chord read from Tornado aircraft geometry:   ' num2str(mean_chord)]);
cd(fileparts(which(mfilename))); % Back to the directory of this script.

% Compute coefficients for longitudinal LTI (alpha-sweep).
disp(' ');
disp('Computing alpha-sweep for longitudinal dynamics...');
[alpha, beta, ...
    CX, CY, CZ, Cl, Cm, Cn, ...% force and moment coefficients
    CX_d, CY_d, CZ_d, Cl_d, Cm_d, Cn_d, ... % rudder derivatives
     CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, ... % roll derivatives
     CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, ... % pitch derivatives
     CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R] ... % yaw derivatives
     =computeAerodynamicCoefficients(...
     nameTornadoAircraftGeometry, state, tornadoDirectory, ...
     alphaStart, alphaEnd, numAlphas, 0, 0, 1, centerOfGravity); % 1 beta at 0 degrees.

% Keep the Cm of the alpha sweep
Cm_longitudinal = Cm;

% Reformat rudder derivatives to simplify in Simulink model
CX_d_elevator = CX_d(:,:,elevatorFlapIndex);
CY_d_elevator = CY_d(:,:,elevatorFlapIndex);
CZ_d_elevator = CZ_d(:,:,elevatorFlapIndex);
Cl_d_elevator = Cl_d(:,:,elevatorFlapIndex);
Cm_d_elevator = Cm_d(:,:,elevatorFlapIndex);
Cn_d_elevator = Cn_d(:,:,elevatorFlapIndex);
CX_d_rudder = CX_d(:,:,rudderFlapIndex);
CY_d_rudder = CY_d(:,:,rudderFlapIndex);
CZ_d_rudder = CZ_d(:,:,rudderFlapIndex);
Cl_d_rudder = Cl_d(:,:,rudderFlapIndex);
Cm_d_rudder = Cm_d(:,:,rudderFlapIndex);
Cn_d_rudder = Cn_d(:,:,rudderFlapIndex);

% Load the longitudinal model
modelPath = ['../models/' nameLongitudinalModel];
load_system(modelPath); % loads model into memory / load_system
loadedModelName = bdroot;
hws = get_param(loadedModelName, 'modelworkspace'); % get handle to workspace

% Update all Simulink model workspace parameters
hws.assignin('alpha', alpha);
hws.assignin('beta', beta);
hws.assignin('centerOfGravity', -centerOfGravity); % convert to Simulink reference frame (x-axis extends forward, negate).
hws.assignin('Cl', Cl);
hws.assignin('Cl_d', Cl_d);
hws.assignin('Cl_d_elevator', Cl_d_elevator);
hws.assignin('Cl_d_rudder', Cl_d_rudder);
hws.assignin('Cl_P', Cl_P);
hws.assignin('Cl_Q', Cl_Q);
hws.assignin('Cl_R', Cl_R);
hws.assignin('Cm', Cm);
hws.assignin('Cm_d', Cm_d);
hws.assignin('Cm_d_elevator', Cm_d_elevator);
hws.assignin('Cm_d_rudder', Cm_d_rudder);
hws.assignin('Cm_P', Cm_P);
hws.assignin('Cm_Q', Cm_Q);
hws.assignin('Cm_R', Cm_R);
hws.assignin('Cn', Cn);
hws.assignin('Cn_d', Cn_d);
hws.assignin('Cn_d_elevator', Cn_d_elevator);
hws.assignin('Cn_d_rudder', Cn_d_rudder);
hws.assignin('Cn_P', Cn_P);
hws.assignin('Cn_Q', Cn_Q);
hws.assignin('Cn_R', Cn_R);
hws.assignin('CX', CX);
hws.assignin('CX_d', CX_d);
hws.assignin('CX_d_elevator', CX_d_elevator);
hws.assignin('CX_d_rudder', CX_d_rudder);
hws.assignin('CX_P', CX_P);
hws.assignin('CX_Q', CX_Q);
hws.assignin('CX_R', CX_R);
hws.assignin('CY', CY);
hws.assignin('CY_d', CY_d);
hws.assignin('CY_d_elevator', CY_d_elevator);
hws.assignin('CY_d_rudder', CY_d_rudder);
hws.assignin('CY_P', CY_P);
hws.assignin('CY_Q', CY_Q);
hws.assignin('CY_R', CY_R);
hws.assignin('CZ', CZ);
hws.assignin('CZ_d', CZ_d);
hws.assignin('CZ_d_elevator', CZ_d_elevator);
hws.assignin('CZ_d_rudder', CZ_d_rudder);
hws.assignin('CZ_P', CZ_P);
hws.assignin('CZ_Q', CZ_Q);
hws.assignin('CZ_R', CZ_R);
hws.assignin('InertiaTensor', InertiaTensor);
hws.assignin('estimatedVelocity', estimatedVelocity);
hws.assignin('mass', mass);
hws.assignin('mean_chord', mean_chord);
hws.assignin('spanwidth', wingspan);

% Find trimmed gliding state (operating point for LTI).

% Create a default operating point specification object
operatingPointSpec = operspec(nameLongitudinalModel);

% Fix elevator input to 0
operatingPointSpec.Inputs(1).Known = true;
operatingPointSpec.Inputs(1).u = 0;

% Find trimmed gliding state
disp(' ');
disp('Searching for trimmed operating point of longitudinal dynamics...');
operatingPointOptions = findopOptions('DisplayReport','off');
[trimmedOperatingPoint, opreport] = findop(nameLongitudinalModel, operatingPointSpec, operatingPointOptions);
disp(['Number of iterations: ' num2str(opreport.OptimizationOutput.iterations)]);
disp(opreport.TerminationString);
%disp(opreport.OptimizationOutput.message);

% Get angle of attack and velocity
trimmedTheta = trimmedOperatingPoint.States(1).x(1); % angle between body-x and earth-x
trimmedu = trimmedOperatingPoint.States(2).x(1);
trimmedw = trimmedOperatingPoint.States(2).x(2);
trimmedq = trimmedOperatingPoint.States(3).x(1); % should be zero.
trimmedVelocity = sqrt(trimmedu^2 + trimmedw^2);
trimmedAlpha = atan(trimmedw/trimmedu);
trimmedStateVariablesLongitudinal = [trimmedu; trimmedw; trimmedq; trimmedTheta];

% Compute velocities in earth reference frame
trimmedu_e =  trimmedu*cos(trimmedTheta) + trimmedw*(sin(trimmedTheta));
trimmedw_e = -trimmedu*sin(trimmedTheta) + trimmedw*(cos(trimmedTheta));

% Compute glide ratio
trimmedGlideRatio = trimmedu_e/trimmedw_e;

% Output result for longitudinal trimmed state
disp(' ');
disp('Trimmed longitudinal state: ');
disp('============================');
disp(['1) u     : velocity in x-body                       (m/s): ' num2str(trimmedu)]);
disp(['2) w     : velocity in z-body                       (m/s): ' num2str(trimmedw)]);
disp(['3) q     : rotation velocity y-body               (deg/s): ' num2str(trimmedq*180/pi)]);
disp(['4) theta : euler pitch (angle body-x to earth-x)    (deg): ' num2str(trimmedTheta*180/pi)]);
disp(' ');

disp('Other parameters in trimmed longitudinal state: ');
disp('================================================');
disp(['Alpha : angle of attack (velocity vector to earth-x) (deg): ' num2str(trimmedAlpha*180/pi)]);
disp(['Absolute velocity (longitudinal)                     (m/s): ' num2str(trimmedVelocity)]);
disp(['Velocity in x-earth                                  (m/s): ' num2str(trimmedu_e)]);
disp(['Velocity in z-earth                                  (m/s): ' num2str(trimmedw_e)]);
disp(['Glide ratio                                               : ' num2str(trimmedGlideRatio)]);
disp(' ');

% Check if trimmed state within acceptable range
if(trimmedAlpha*180/pi > alphaEnd || trimmedAlpha*180/pi < alphaStart)    
    disp(['Trimmed alpha is ' num2str(trimmedAlpha*180/pi)]);
    disp(['Minimum alpha is ' num2str(alphaStart)]);
    disp(['Maximum alpha is ' num2str(alphaEnd)]);
    error('Trimmed alpha lies outside the alpha sweep range for which aerodynamic coefficients have been computed!');
end
if(abs(trimmedVelocity - estimatedVelocity) > estimatedVelocity*velocityDeviationTol)    
    disp(['Trimmed velocity is ' num2str(trimmedVelocity) '.']);
    disp(['Estimated velocity is ' num2str(estimatedVelocity) '.']);
    error('Trimmed velocity differs more than 10% from the estimated velocity which has been used to compute the aerodynamic coefficients!');
end

% Options to linearize longitudinal model
linOptions = linearizeOptions('UseFullBlockNameLabels','on', 'UseBusSignalLabels','on');

% Specify the custom state ordering according to Caughey convention.
stateOrder = {'ExperimentalCarrier_longitudinal/Airframe/3DOF_longitudinal_no_position/U,w',...
    'ExperimentalCarrier_longitudinal/Airframe/3DOF_longitudinal_no_position/q',...
    'ExperimentalCarrier_longitudinal/Airframe/3DOF_longitudinal_no_position/Theta'};

% Linearize longitudinal model
ltiLongitudinal = linearize(nameLongitudinalModel, trimmedOperatingPoint, linOptions, 'StateOrder', stateOrder);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compute coefficients for lateral LTI (beta-sweep at trimmed alpha).
disp('Computing beta-sweep for lateral dynamics...');
[alpha, beta, ...
    CX, CY, CZ, Cl, Cm, Cn, ...% force and moment coefficients
    CX_d, CY_d, CZ_d, Cl_d, Cm_d, Cn_d, ... % rudder derivatives
     CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, ... % roll derivatives
     CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, ... % pitch derivatives
     CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R] ... % yaw derivatives
     =computeAerodynamicCoefficients(...
     nameTornadoAircraftGeometry, state, tornadoDirectory, ...
     trimmedAlpha*180/pi, trimmedAlpha*180/pi, 1, betaStart, betaEnd, numBetas, centerOfGravity); % 1 alpha at trimmedAlpha radian.

% Keep Cn for beta sweep
Cn_lateral = Cn;

% Reformat rudder derivatives to simplify in Simulink model
CX_d_elevator = CX_d(:,:,elevatorFlapIndex);
CY_d_elevator = CY_d(:,:,elevatorFlapIndex);
CZ_d_elevator = CZ_d(:,:,elevatorFlapIndex);
Cl_d_elevator = Cl_d(:,:,elevatorFlapIndex);
Cm_d_elevator = Cm_d(:,:,elevatorFlapIndex);
Cn_d_elevator = Cn_d(:,:,elevatorFlapIndex);
CX_d_rudder = CX_d(:,:,rudderFlapIndex);
CY_d_rudder = CY_d(:,:,rudderFlapIndex);
CZ_d_rudder = CZ_d(:,:,rudderFlapIndex);
Cl_d_rudder = Cl_d(:,:,rudderFlapIndex);
Cm_d_rudder = Cm_d(:,:,rudderFlapIndex);
Cn_d_rudder = Cn_d(:,:,rudderFlapIndex);

% Load the longitudinal model
modelPath = ['../models/' nameLateralModel];
load_system(modelPath); % loads model into memory / load_system
loadedModelName = bdroot;
hws = get_param(loadedModelName, 'modelworkspace'); % get handle to workspace

% Update all Simulink model workspace parameters
hws.assignin('alpha', alpha);
hws.assignin('beta', beta);
hws.assignin('centerOfGravity', -centerOfGravity); % convert to Simulink reference frame (x-axis extends forward, negate).
hws.assignin('Cl', Cl);
hws.assignin('Cl_d', Cl_d);
hws.assignin('Cl_d_elevator', Cl_d_elevator);
hws.assignin('Cl_d_rudder', Cl_d_rudder);
hws.assignin('Cl_P', Cl_P);
hws.assignin('Cl_Q', Cl_Q);
hws.assignin('Cl_R', Cl_R);
hws.assignin('Cm', Cm);
hws.assignin('Cm_d', Cm_d);
hws.assignin('Cm_d_elevator', Cm_d_elevator);
hws.assignin('Cm_d_rudder', Cm_d_rudder);
hws.assignin('Cm_P', Cm_P);
hws.assignin('Cm_Q', Cm_Q);
hws.assignin('Cm_R', Cm_R);
hws.assignin('Cn', Cn);
hws.assignin('Cn_d', Cn_d);
hws.assignin('Cn_d_elevator', Cn_d_elevator);
hws.assignin('Cn_d_rudder', Cn_d_rudder);
hws.assignin('Cn_P', Cn_P);
hws.assignin('Cn_Q', Cn_Q);
hws.assignin('Cn_R', Cn_R);
hws.assignin('CX', CX);
hws.assignin('CX_d', CX_d);
hws.assignin('CX_d_elevator', CX_d_elevator);
hws.assignin('CX_d_rudder', CX_d_rudder);
hws.assignin('CX_P', CX_P);
hws.assignin('CX_Q', CX_Q);
hws.assignin('CX_R', CX_R);
hws.assignin('CY', CY);
hws.assignin('CY_d', CY_d);
hws.assignin('CY_d_elevator', CY_d_elevator);
hws.assignin('CY_d_rudder', CY_d_rudder);
hws.assignin('CY_P', CY_P);
hws.assignin('CY_Q', CY_Q);
hws.assignin('CY_R', CY_R);
hws.assignin('CZ', CZ);
hws.assignin('CZ_d', CZ_d);
hws.assignin('CZ_d_elevator', CZ_d_elevator);
hws.assignin('CZ_d_rudder', CZ_d_rudder);
hws.assignin('CZ_P', CZ_P);
hws.assignin('CZ_Q', CZ_Q);
hws.assignin('CZ_R', CZ_R);
hws.assignin('InertiaTensor', InertiaTensor);
hws.assignin('estimatedVelocity', estimatedVelocity);
hws.assignin('mass', mass);
hws.assignin('mean_chord', mean_chord);
hws.assignin('p_ini', 0);
hws.assignin('phi_ini', 0);
hws.assignin('r_ini', 0);
hws.assignin('spanwidth', wingspan);
hws.assignin('theta_ini', trimmedTheta); % radian
hws.assignin('u_ini', trimmedu);
hws.assignin('v_ini', 0);
hws.assignin('w_ini', trimmedw);

% Find trimmed gliding state (operating point for LTI).

% Create a default operating point specification object
operatingPointSpec = operspec(nameLateralModel);

% Fix rudder input to 0
operatingPointSpec.Inputs(1).Known = true;
operatingPointSpec.Inputs(1).u = 0;

% Find trimmed gliding state
disp(' ');
disp('Searching for trimmed operating point of lateral dynamics...');
operatingPointOptions = findopOptions('DisplayReport','off');
[trimmedOperatingPoint, opreport] = findop(nameLateralModel, operatingPointSpec, operatingPointOptions);
disp(['Number of iterations: ' num2str(opreport.OptimizationOutput.iterations)]);
disp(opreport.TerminationString);
%disp(opreport.OptimizationOutput.message);

% Get angle of attack and velocity
trimmedv = trimmedOperatingPoint.States(4).x(1); % see state names <-> indices when debugging trimmedOperatingPoint variable
trimmedp = trimmedOperatingPoint.States(1).x(1);
trimmedPhi = trimmedOperatingPoint.States(2).x(1);
trimmedr = trimmedOperatingPoint.States(3).x(1); 
trimmedVelocity = sqrt(trimmedu^2 + trimmedv^2 + trimmedw^2);
trimmedBeta = atan(trimmedv/trimmedu);
trimmedStateVariablesLateral = [trimmedv; trimmedp; trimmedPhi; trimmedr];

% Output result for longitudinal trimmed state
disp(' ');
disp('Trimmed lateral state: ');
disp('============================');
disp(['1) v     : velocity in y-body           (m/s): ' num2str(trimmedv)]);
disp(['2) p     : rotation velocity x-body   (deg/s): ' num2str(trimmedp)]);
disp(['3) phi   : euler roll angle             (deg): ' num2str(trimmedPhi*180/pi)]);
disp(['4) r     : rotation velocity z-body    deg/s): ' num2str(trimmedr)]);
disp(' ');

disp('Other parameters in trimmed lateral state: ');
disp('================================================');
disp(['Beta     : sideslip                        (deg): ' num2str(trimmedBeta*180/pi)]);
disp(['Absolute velocity (longitudinal & lateral) (m/s): ' num2str(trimmedVelocity)]);
disp(' ');

% Check if trimmed state within acceptable range
if(trimmedBeta*180/pi > betaEnd || trimmedBeta*180/pi < betaStart)    
    disp(['Trimmed beta is ' num2str(trimmedBeta*180/pi)]);
    disp(['Minimum beta is ' num2str(betaStart)]);
    disp(['Maximum beta is ' num2str(betaEnd)]);
    error('Trimmed beta lies outside the beta sweep range for which aerodynamic coefficients have been computed!');
end

% Options to linearize longitudinal model
linOptions = linearizeOptions('UseFullBlockNameLabels','on', 'UseBusSignalLabels','on');

% Specify the custom state ordering according to Caughey convention
% (v,p,phi,r)
stateOrder = {'ExperimentalCarrier_lateral/Airframe/3DOF_lateral_no_position/v_integrator',...
    'ExperimentalCarrier_lateral/Airframe/3DOF_lateral_no_position/p_integrator',...
    'ExperimentalCarrier_lateral/Airframe/3DOF_lateral_no_position/phi_integrator',...
    'ExperimentalCarrier_lateral/Airframe/3DOF_lateral_no_position/r_integrator'};

% Linearize lateral model
ltiLateral = linearize(nameLateralModel, trimmedOperatingPoint, linOptions, 'StateOrder', stateOrder);

end

