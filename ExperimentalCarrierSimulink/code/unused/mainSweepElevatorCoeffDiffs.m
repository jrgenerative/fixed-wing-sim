% I think this script was written before understanding that rudder derivatives
% are readily available from Tornado (?).
% Compute the first order derivative of all coefficients with respect to
% elevator (horizontal stabilizer rudder) deflection at different alphas. Yields
% derivative of coefficients per radian.
% Tornado\T135_export has to be in the path
% See also solverloop5.m line 312 on how to to a sweep over rudder deflections.

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
% 
% @

currDir = pwd;
cd(fileparts(which(mfilename)));
scriptDir = pwd;

% Load the geometry and the initial state
load('../aircraft/ExperimentalCarrier.mat')
load('../aircraft/State_1.5alpha_12.5ms.mat')

% Must run in tornado root directory for loading airfoil profile later
cd('F:\svn\dev\matlab\tornado\T135_export')

% Alpha range
alphaStart = -10/180*pi;
alphaEnd = 20/180*pi;
numAlphas = 31; % must be > 1

resultsPos = [];
resultsNeg = [];
settings=config('startup');        

%lattictype=1; %Standard VLM
lattictype=0;%Tornado freestream following wake VLM

alphaStep = (alphaEnd - alphaStart)/(numAlphas-1);
alpha = zeros(numAlphas,1);
CXdr = zeros(numAlphas, 1);
CYdr = zeros(numAlphas, 1);
CZdr = zeros(numAlphas, 1);
Cldr = zeros(numAlphas, 1);
Cmdr = zeros(numAlphas, 1);
Cndr = zeros(numAlphas, 1);

[n,m]=find(geo.flapped');
resetdelta=geo.flap_vector;
positiveFlap = 1*pi/180; % in radian
negativeFlap = -1*pi/180; % in radian
flapDiff = positiveFlap - negativeFlap;
rudder = 1; % the first rudder is on the horizontal stabilizer in ExperimentalCarrier geometry.

%% loop over alphas
for alphaIndex=1:numAlphas
    
    % Modify state
    alphaCurrent = alphaStart + (alphaIndex-1)*alphaStep;
    state.alpha = alphaCurrent;
    alpha(alphaIndex) = alphaCurrent;
    state.betha = 0.0;
    
    %% Positive Flap 
    % Modify geometry
    geo.flap_vector(m(rudder),n(rudder))=positiveFlap;
    % Regenerate lattice.
    [lattice,ref]=fLattice_setup2(geo,state,lattictype);
    % Compute the solution
    [resultsPos]=solver9(resultsPos,state,geo,lattice,ref);
    [resultsPos]=coeff_create3(resultsPos,lattice,state,ref,geo);
    
    %% Negative Flap
    % Modify geometry
    geo.flap_vector(m(rudder),n(rudder))=negativeFlap;
    % Regenerate lattice.
    [lattice,ref]=fLattice_setup2(geo,state,lattictype);
    % Compute the solution
    [resultsNeg]=solver9(resultsNeg,state,geo,lattice,ref);
    [resultsNeg]=coeff_create3(resultsNeg,lattice,state,ref,geo);
    
    %% Derivative
    
    CXdr(alphaIndex, 1) = (resultsPos.CX - resultsNeg.CX)/flapDiff;
    CYdr(alphaIndex, 1) = (resultsPos.CY - resultsNeg.CY)/flapDiff;
    CZdr(alphaIndex, 1) = (resultsPos.CZ - resultsNeg.CZ)/flapDiff;
    Cldr(alphaIndex, 1) = (resultsPos.Cl - resultsNeg.Cl)/flapDiff;
    Cmdr(alphaIndex, 1) = (resultsPos.Cm - resultsNeg.Cm)/flapDiff;
    Cndr(alphaIndex, 1) = (resultsPos.Cn - resultsNeg.Cn)/flapDiff;
    
    %% Output 
    disp(['alpha ' num2str(alphaCurrent*180/pi) ' deg']);
end

geo.flap_vector=resetdelta;

cd(currDir);

save

alphaDegrees = alpha*180/pi;

figure(23);
subplot(2,3,1);
plot(alphaDegrees,CXdr);
title('CXdr (longitudinal)');
xlabel('alpha');
zlabel('longitudinal coefficient derivative');

subplot(2,3,2);
plot(alphaDegrees,CYdr);
title('CYdr (lateral)');
xlabel('alpha');
zlabel('lateral coefficient derivative');

subplot(2,3,3);
plot(alphaDegrees, CZdr);
title('CZdr (lift)');
xlabel('alpha');
zlabel('lift force coefficient derivative');

subplot(2,3,4);
plot(alphaDegrees, Cldr);
title('Cldr (roll)');
xlabel('alpha');
zlabel('roll moment coefficient derivative');

subplot(2,3,5);
plot(alphaDegrees, Cmdr);
title('Cmdr (pitch)');
xlabel('alpha');
zlabel('pitch moment coefficient derivative');

subplot(2,3,6);
plot(alphaDegrees, Cndr);
title('Cndr (yaw)');
xlabel('alpha');
zlabel('yaw moment coefficient derivative');


    