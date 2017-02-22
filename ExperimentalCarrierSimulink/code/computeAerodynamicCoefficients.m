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

function [alpha, beta, ...                  % sampled values
    CX,   CY,   CZ,   Cl,   Cm,   Cn,   ... % datum coefficients
    CX_d, CY_d, CZ_d, Cl_d, Cm_d, Cn_d, ... % rudder moments for each rudder (3d matrices with rudder index in last dimension)
    CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, ... % damping when rotating around X
    CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, ... % damping when rotating around Y
    CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R  ... % damping when rotating around Z
    ]=computeAerodynamicCoefficients(...
    aircraft_geometry_filename, state, tornado_root_directory, ...
    alphaStart, alphaEnd, numAlphas, betaStart, betaEnd, numBetas, ...
    centerOfGravity)
% Compute coefficient matrices as required by a aeroblock simulink
% simulation project [CX, CY, CZ, Cl, Cm, Cn]. The function sweeps over alpha
% (angle of attack) and beta (heading angle). 
% aircraft_geometry_filename is the name of the aircraft geometry file you
% want to use, e.g. 'ExperimentalCarrier', or 'B747-400'.
% state is the data struct containing an initial state for the coefficient
% computation. This merily requires to set a velocity. A state can
% be defined for example as:
% state.AS=      12.5;      %Airspeed m/s
% state.alpha=   0.0;       %Angle of attack, radians
% state.betha=   0;         %Angle of sideslip, radians
% state.P=       0;         %Rollrate, rad/s
% state.Q=       0;         %pitchrate, rad/s
% state.R=       0;         %yawrate, rad/s
% state.adot=    0;         %Alpha time derivative rad/s
% state.bdot=    0;         %Betha time derivative rad/s
% state.ALT=     0;         %Altitude, m
% state.rho=     1.225;     %Desity, kg/m^3
% state.pgcorr=  0;         %Apply prandtl glauert compressibility correction
% tornado_root_directory is the root directory of the Tornado installation.
% You need to copy your aircraft geometry and the respective status into
% the appropriate directories 'aircraft' and 'state'.
% alphaStart is the angle of attack in degrees at which to start sampling (e.g. -10).
% alphaEnd is the angle of attack in degrees at which to stop when sampling (e.g. 20).
% numAlphas is the number of alphas to sample (e.g. 31).
% betaStart is the heading angle in degrees at which to start sampling
% (e.g. -10).
% betaEnd is the heading angle in degrees at which to stop sampling (e.g.
% 10)
% numBetas is the number of betas to sample (e.g. 21).
% centerOfGravity is the center of gravity in meters from the origin of the
% aircraft geometry reference frame. This value is used to overwrite the 
% center of gravity and the moment reference point specified in the Tornado
% aircraft geometry.
% Output is: 
% alpha vector of sampled alpha values
% beta vector of sampled beta values
% CX, CY, CZ, coefficients of forces in X, Y, and Z direction (body ref.).
% These are 2d matrices of the format C(alpha, beta).
% Cl, Cm, Cn, coefficients of moments along x, y, and z axis. These are 2d
% matrices of the format C(alpha, beta).
% CX_d, CY_d, CZ_d, Cl_d, Cm_d, Cn_d coefficient moment derivatives for 
% rudder deflection (in degrees or radian?). These are 3d matrices of the
% format C(alpha, beta,rudder_index). Where the rudder index refers to a
% rudder in the sequence as rudders have been defined in the aircraft geometry.
% CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, damping when rotating around X.
% CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, damping when rotating around Y.
% CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R, damping when rotating around Z.
% Example call:
% [alpha, beta, CX, CY, CZ, Cl, Cm, Cn, Cl_d, Cm_d, Cn_d, CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R]=computeCoefficientsForSimulinkSimulation('ExperimentalCarrier', 'State_1.5alpha_12.5ms.mat', 'F:\svn\dev\matlab\tornado\T135_export', -10, 20, 31, -10, 10, 21);

% Alpha range
alphaStart = alphaStart/180*pi;
alphaEnd = alphaEnd/180*pi;

% Beta range
betaStart = betaStart/180*pi;
betaEnd = betaEnd/180*pi;

currDir = pwd; % store the current directory

results = [];
cd(tornado_root_directory);
settings=config('startup');

% Load the aircraft
cd(settings.acdir)
load(aircraft_geometry_filename); % aircraft geometry is now in variable 'geo'.

% Overwrite center of gravity with given value (!)
geo.CG = [centerOfGravity 0 0];

% Overwrite reference point for moment computation with center of gravity.
geo.ref_point = [centerOfGravity 0 0];

numRudders = nnz(geo.flapped); % number of non-zero elements
% Back to the root directory.
cd(settings.hdir)          

%lattictype=1; %Standard VLM
lattictype=0;%Tornado freestream following wake VLM

% Allocate variables
if numAlphas > 1
    alphaStep = (alphaEnd - alphaStart)/(numAlphas-1);
else
    alphaStep = 0;
end
if numBetas > 1
    betaStep = (betaEnd - betaStart)/(numBetas-1);
else
    betaStep = 0;
end
alpha = zeros(numAlphas,1);
beta = zeros(numBetas,1);
CX = zeros(numAlphas, numBetas);
CY = zeros(numAlphas, numBetas);
CZ = zeros(numAlphas, numBetas);
Cl = zeros(numAlphas, numBetas);
Cm = zeros(numAlphas, numBetas);
Cn = zeros(numAlphas, numBetas);
CX_d = zeros(numAlphas, numBetas, numRudders);
CY_d = zeros(numAlphas, numBetas, numRudders);
CZ_d = zeros(numAlphas, numBetas, numRudders);
Cl_d = zeros(numAlphas, numBetas, numRudders);
Cm_d = zeros(numAlphas, numBetas, numRudders);
Cn_d = zeros(numAlphas, numBetas, numRudders);
CX_P = zeros(numAlphas, numBetas);
CY_P = zeros(numAlphas, numBetas);
CZ_P = zeros(numAlphas, numBetas);
Cl_P = zeros(numAlphas, numBetas);
Cm_P = zeros(numAlphas, numBetas);
Cn_P = zeros(numAlphas, numBetas);
CX_Q = zeros(numAlphas, numBetas);
CY_Q = zeros(numAlphas, numBetas);
CZ_Q = zeros(numAlphas, numBetas);
Cl_Q = zeros(numAlphas, numBetas);
Cm_Q = zeros(numAlphas, numBetas);
Cn_Q = zeros(numAlphas, numBetas);
CX_R = zeros(numAlphas, numBetas);
CY_R = zeros(numAlphas, numBetas);
CZ_R = zeros(numAlphas, numBetas);
Cl_R = zeros(numAlphas, numBetas);
Cm_R = zeros(numAlphas, numBetas);
Cn_R = zeros(numAlphas, numBetas);

% loop over alphas
for alphaIndex=1:numAlphas
    
    alphaCurrent = alphaStart + (alphaIndex-1)*alphaStep;
    state.alpha = alphaCurrent;
    alpha(alphaIndex) = alphaCurrent;
    
    % loop over betas
    for betaIndex=1:numBetas
        
        betaCurrent = betaStart + (betaIndex-1)*betaStep;
        state.betha = betaCurrent;
        beta(betaIndex) = betaCurrent;
        
        % Regenerate lattice.
        [lattice,ref]=fLattice_setup2(geo,state,lattictype);

        % Compute the solution with standard one-side derivative
        [results]=solver9(results,state,geo,lattice,ref);
        [results]=coeff_create3(results,lattice,state,ref,geo);
        
%         % Compute the solution with symmetric derivatives
%         [results]=solver9_symmetric(results,state,geo,lattice,ref);
%         [results]=coeff_create3_symmetric(results,lattice,state,ref,geo);

        % Datum coefficients
        CX(alphaIndex, betaIndex) = results.CX; % force in X
        CY(alphaIndex, betaIndex) = results.CY; % force in Y
        CZ(alphaIndex, betaIndex) = results.CZ; % force in Z
        Cl(alphaIndex, betaIndex) = results.Cl; % roll moment
        Cm(alphaIndex, betaIndex) = results.Cm; % pitch moment
        Cn(alphaIndex, betaIndex) = results.Cn; % yaw moment
        
        % Actuator derivatives
        % Coefficient derivatives for rudder movements. 
        % results.C<x>_d is a vector containing rudder 
        % derivatives for each rudder in the sequence as rudders have been
        % defined.
        CX_d(alphaIndex, betaIndex, :) = results.CX_d; % X force change
        CY_d(alphaIndex, betaIndex, :) = results.CY_d; % Y force change
        CZ_d(alphaIndex, betaIndex, :) = results.CZ_d; % Z force change
        Cl_d(alphaIndex, betaIndex, :) = results.Cl_d; %roll moment change
        Cm_d(alphaIndex, betaIndex, :) = results.Cm_d; %pitch moment change
        Cn_d(alphaIndex, betaIndex, :) = results.Cn_d; %yaw moment change
                
        % Damping coefficients
        
        % derivatives by roll rotation velocity
        CX_P(alphaIndex, betaIndex) = results.CX_P; % in X when rotating around X
        CY_P(alphaIndex, betaIndex) = results.CY_P; % ...
        CZ_P(alphaIndex, betaIndex) = results.CZ_P;
        Cl_P(alphaIndex, betaIndex) = results.Cl_P;
        Cm_P(alphaIndex, betaIndex) = results.Cm_P;
        Cn_P(alphaIndex, betaIndex) = results.Cn_P;        
        
         % derivatives by pitch rotation velocity
        CX_Q(alphaIndex, betaIndex) = results.CX_Q; % in X when rotating around Y
        CY_Q(alphaIndex, betaIndex) = results.CY_Q; % ...
        CZ_Q(alphaIndex, betaIndex) = results.CZ_Q;
        Cl_Q(alphaIndex, betaIndex) = results.Cl_Q;
        Cm_Q(alphaIndex, betaIndex) = results.Cm_Q;
        Cn_Q(alphaIndex, betaIndex) = results.Cn_Q;        
        
         % derivatives by yaw rotation velocity
        CX_R(alphaIndex, betaIndex) = results.CX_R; % in X when rotating around Z
        CY_R(alphaIndex, betaIndex) = results.CY_R; % ...
        CZ_R(alphaIndex, betaIndex) = results.CZ_R;
        Cl_R(alphaIndex, betaIndex) = results.Cl_R;
        Cm_R(alphaIndex, betaIndex) = results.Cm_R;
        Cn_R(alphaIndex, betaIndex) = results.Cn_R;                  
        
        disp(['alpha ' num2str(alphaCurrent*180/pi) ' deg, beta ' num2str(betaCurrent*180/pi) ' deg']);
    end
end

cd(currDir); % change back to the originally current directory.

    
    
    