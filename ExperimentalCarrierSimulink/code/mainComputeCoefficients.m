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

% The tornado installation directory.
% Must contain the aircraft geometry file and the status used below.
tornado_root_directory = 'F:\svn\dev\matlab\tornado\T135_export';

% The state:
% Velocity 12.5 m/s , ~45km/h (average velocity as measured in real flight)
% Velocity doesn't matter much for coefficient computation. 
% Coefficients are largely velocity independent (tested)).
state.AS=      12.5;      %Airspeed m/s
state.alpha=   0.0;       %Angle of attack, radians
state.betha=   0;         %Angle of sideslip, radians
state.P=       0;         %Rollrate, rad/s
state.Q=       0;         %pitchrate, rad/s
state.R=       0;         %yawrate, rad/s
state.adot=    0;         %Alpha time derivative rad/s
state.bdot=    0;         %Betha time derivative rad/s
state.ALT=     0;         %Altitude, m
state.rho=     1.225;     %Desity, kg/m^3
state.pgcorr=  0;         %Apply prandtl glauert compressibility correction

% The ExperimentalCarrier geometry.
aircraft_geometry_name = 'ExperimentalCarrier';

% Center of gravity
centerOfGravity = 0.092; % Tornado x-axis extends aft (!).

% The sweep ranges and sampling resolution for Simulink modelling
% alphaStart = -10;
% alphaEnd = 20;
% numAlphas = 31;
% betaStart = -10;
% betaEnd = 10;
% numBetas = 21;

% Quick Test
alphaStart = -10;
alphaEnd = 20;
numAlphas = 2;
betaStart = -10;
betaEnd = 10;
numBetas = 2;

% Coefficient computation
[alpha, beta, CX, CY, CZ, Cl, Cm, Cn, CX_d, CY_d, CZ_d, Cl_d, Cm_d, Cn_d, ...
     CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R]=computeAerodynamicCoefficients(...
     aircraft_geometry_name, state, tornado_root_directory, alphaStart, ...
     alphaEnd, numAlphas, betaStart, betaEnd, numBetas, centerOfGravity);

plotCoefficients(alpha, beta, CX, CY, CZ, Cl, Cm, Cn, CX_d, CY_d, CZ_d, Cl_d, Cm_d, Cn_d, ...
    CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R);

plotAircraftGeometry(aircraft_geometry_name, state, tornado_root_directory);

save;
