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

dirOfScript = fileparts(mfilename('fullpath'));
airfoilCoords = importdata([dirOfScript '/../../airfoil/JR001_ORIG.dat']);
airfoilCoords = airfoilCoords(:, [1 2]);

upperX = airfoilCoords(1:44,1); 
upperY = airfoilCoords(1:44,2);
lowerX = airfoilCoords(44:78,1);
lowerY = airfoilCoords(44:78,2);

% need to reverse lower coords to run from nose to tail
lowerX = lowerX(35:-1:1); 
lowerY = lowerY(35:-1:1);

% Since there is a different number of x locations on lower and upper
% curve, interpolate. Interpolate the one with less x-locations.
lowerY = interp1(lowerX, lowerY, upperX);
lowerX = upperX;

% Need to scale to [0...1]
upperX = upperX./100;
upperY = upperY./100;
lowerX = lowerX./100;
lowerY = lowerY./100;

hold on;
plot(lowerX, lowerY, 'r');
plot(upperX, upperY, 'b');
hold off;

pwd
fileID = fopen([dirOfScript '/../../airfoil/JR001_TORNADO.DAT'],'w' );
fprintf(fileID, '%s\n', '% (JR001 Airfoil)');
fprintf(fileID, '%s\n', '44.  44.');
fprintf(fileID, '%s\n', '%  x/c  y/c upper'); 
formatSpec = '%3.4f % 3.4f\n';
fprintf(fileID, formatSpec, [upperX upperY]');
fprintf(fileID, '%s\n', '%  x/c  y/c lower ');
fprintf(fileID, formatSpec, [lowerX lowerY]');
fclose(fileID);

