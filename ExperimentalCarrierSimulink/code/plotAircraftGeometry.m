% Plot the aircraft's geometry setup.
% Example call: plotAircraftGeometry('ExperimentalCarrier', 'State_1.5alpha_12.5ms.mat', 'F:\svn\dev\matlab\tornado\T135_export');

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

function plotAircraftGeometry(aircraft_geometry_filename, state, tornado_root_directory)

currDir = pwd;

cd(tornado_root_directory);
settings=config('startup');

% Load the aircraft
cd(settings.acdir)
load(aircraft_geometry_filename); % aircraft geometry is now in variable 'geo'.

cd(tornado_root_directory)

% Generate the Lattice
latticetype=0; % Tornado lattice type
[lattice,ref]=fLattice_setup2(geo,state,latticetype);

%  Plot the geometry
geometryplot(lattice,geo,ref);
%bodyplot(body);

% back to the original directory
cd(currDir);





