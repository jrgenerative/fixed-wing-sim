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

function [ac, ... % the aerodynamic center of the aircraft in meters measured from the Tornado frame of reference origin.
    staticMarginMAC, ... % the static margin of the aircraft with the given center of gravity as a factor of the mean chord.
    staticMarginInMeters]...
    = computeStaticMargin(...
nameTornadoAircraftGeometry, ...
estimatedVelocity, ...
airDensity, ...
tornadoDirectory, ...
centerOfGravity)

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
% Overwrite center of gravity with given value (!)
geo.CG = [centerOfGravity 0 0];
% Overwrite reference point for moment computation with center of gravity.
geo.ref_point = [centerOfGravity 0 0];

% Find static margin
% Since aerodynamic center for entire aircraft should correspond with
% neutral point, the static margin should add up (cg + margin = neutral
% point = aerodynamic center entire plane).
cd(tornadoDirectory);
out=fFindstaticmargin(geo,state);
staticMarginMAC = out.h(1);
mean_chord = geo.c(1);
staticMarginInMeters = out.h(1)*mean_chord;
ac = out.ac(1);

cd(fileparts(which(mfilename))); % Back to the directory of this script.

end

