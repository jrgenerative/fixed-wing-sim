% Compute and plot the coefficients as a function of flap deflection.
% 1) Plot the coefficient derivative on flap deflection computed by Tornado
% 2) Plot the coefficient derivative estimated manually (finite difference).
%
% This script has been written to investigate question 1 of the two questions
% below. (Question remains partly unanswered / unfixed).
%
% The following points remain unclear in the Tornado implementation:
%
% 1) To compute the coefficient derivatives by flap deflection (all results.C<x>_d variables), mustn't the downwash (aerodynamic influence coefficients (AIC) on the left-hand side) be recomputed since the flap deflection changes the lattice geometry and this influences AIC (for AIC see also https://en.wikipedia.org/wiki/Vortex_lattice_method)? AIC is computed in solver9.m (function[dw,DW]=fastdw(lattice)).
%
% 2) To compute the coefficient derivatives by alpha and beta (all results.C<x>_a and results.C<x>_b variables), mustn't the lattice be updated (see 1)) and thus, the downwash (AIC) be recomputed since a change in alpha and beta changes the freestream following wake? The change in wake you can see if changing alpha, recomputing lattice and replotting geometry plots in an interactive Tornado session.

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

% ==================================================================================

% The tornado installation directory.
% Must contain the aircraft geometry file and the status used below.
tornado_root_directory = 'F:\svn\dev\matlab\tornado\T135_export';

% The state
state_name = 'State_1.5alpha_12.5ms.mat';

% The aircraft geometry.
aircraft_geometry_name = 'ExperimentalCarrier';

% store the current directory
currDir = pwd; 

results = [];
resultsDelta = [];
cd(tornado_root_directory);
settings=config('startup');

% Load the aircraft
cd(settings.acdir)
load(aircraft_geometry_name); % aircraft geometry is now in variable 'geo'.
numRudders = nnz(geo.flapped); % number of non-zero elements

% Load the state
cd(settings.sdir)
load(state_name); % state information is now in variable 'state'.

% Back to the Tornado root directory.
cd(settings.hdir)          

%lattictype=1; %Standard VLM
lattictype=0; %Tornado freestream following wake VLM

% =======================================================================

numFlaps=sum(sum(geo.flapped)); %Number of trailing edge control effectors
[n,m]=find(geo.flapped');
deltaDeriv = config('delta'); % delta to compute derivative as used in setboundary5.m line 139. 

for rudder=1:numFlaps
    
    figure(16+rudder);
    
    % Remember original flap deflection
    bkpFlapDeflect = geo.flap_vector(m(rudder),n(rudder));
    
    deflectRange = -0.4:0.1:0.4; % radian
    numMeasurements = size(deflectRange, 2);
    CX = zeros(numMeasurements,1);
    CY = zeros(numMeasurements,1);
    CZ = zeros(numMeasurements,1);
    Cl = zeros(numMeasurements,1);
    Cm = zeros(numMeasurements,1);
    Cn = zeros(numMeasurements,1);
    CX_d = zeros(numMeasurements,numFlaps);
    CY_d = zeros(numMeasurements,numFlaps);
    CZ_d = zeros(numMeasurements,numFlaps);
    Cl_d = zeros(numMeasurements,numFlaps);
    Cm_d = zeros(numMeasurements,numFlaps);
    Cn_d = zeros(numMeasurements,numFlaps);
    CX_d_comp = zeros(numMeasurements,1);
    CY_d_comp = zeros(numMeasurements,1);
    CZ_d_comp = zeros(numMeasurements,1);
    Cl_d_comp = zeros(numMeasurements,1);
    Cm_d_comp = zeros(numMeasurements,1);
    Cn_d_comp = zeros(numMeasurements,1);
    
    counter = 0;
    for deflect=deflectRange % radian
        
        counter = counter+1;
        
        % Set the flap deflection
        geo.flap_vector(m(rudder),n(rudder))=deflect;
        
        % Regenerate lattice
        [lattice,ref]=fLattice_setup2(geo,state,0);
  
        % Compute the solution with standard one-side derivative
        results=solver9(results,state,geo,lattice,ref);
        results=coeff_create3(results,lattice,state,ref,geo);
        
        % Datum coefficients
        CX(counter,1) = results.CX; % force in X
        CY(counter,1) = results.CY; % force in Y
        CZ(counter,1) = results.CZ; % force in Z
        Cl(counter,1) = results.Cl; % roll moment
        Cm(counter,1) = results.Cm; % pitch moment
        Cn(counter,1) = results.Cn; % yaw moment
        
        % Tornado rudder derivatives
        CX_d(counter,:) = results.CX_d; % change in force in X on rudder deflection
        CY_d(counter,:) = results.CY_d; % change in force in Y on rudder deflection
        CZ_d(counter,:) = results.CZ_d; % ...
        Cl_d(counter,:) = results.Cl_d; % change in roll moment on rudder deflection
        Cm_d(counter,:) = results.Cm_d; % ... pitch moment
        Cn_d(counter,:) = results.Cn_d; % ... yaw moment
            
        % Manual rudder derivative computation
        
        % Set the flap deflection + delta
        geo.flap_vector(m(rudder),n(rudder))=deflect + deltaDeriv;
        
        % Regenerate lattice (this might be the difference to the standard derivatives?)
        [lattice,ref]=fLattice_setup2(geo,state,0);
  
        % Compute the solution 
        resultsDelta=solver9(resultsDelta,state,geo,lattice,ref);
        resultsDelta=coeff_create3(resultsDelta,lattice,state,ref,geo);
        
        % Derivatives for comparison
        CX_d_comp(counter,1) = (resultsDelta.CX-results.CX)./deltaDeriv; 
        CY_d_comp(counter,1) = (resultsDelta.CY-results.CY)./deltaDeriv; 
        CZ_d_comp(counter,1) = (resultsDelta.CZ-results.CZ)./deltaDeriv; 
        Cl_d_comp(counter,1) = (resultsDelta.Cl-results.Cl)./deltaDeriv;
        Cm_d_comp(counter,1) = (resultsDelta.Cm-results.Cm)./deltaDeriv;
        Cn_d_comp(counter,1) = (resultsDelta.Cn-results.Cn)./deltaDeriv;
        
        disp(['Rudder: ' num2str(rudder) ' iteration ' num2str(counter) ' of ' num2str(numMeasurements)]);
    end
    
    subplot(2,3,1);
    hold on;
    plot(deflectRange, CX, '-r');
    plot(deflectRange, CX_d(:,rudder), '-b');
    plot(deflectRange, CX_d_comp, '-g');
    legend('Coefficient value', 'Tornado deflection derivative', 'Alter. deflection derivative');
    title('CX (longitudinal)');
    xlabel('deflection (rad)');
    ylabel('CX');
    hold off;
    
    subplot(2,3,2);
    hold on;
    plot(deflectRange, CY, '-r');
    plot(deflectRange, CY_d(:,rudder), '-b');
    plot(deflectRange, CY_d_comp, '-g');
    legend('Coefficient value', 'Tornado deflection derivative', 'Alter. deflection derivative');
    title('CY (lateral)');
    xlabel('deflection (rad)');
    ylabel('CY');
    hold off;
    
    subplot(2,3,3);
    hold on;
    plot(deflectRange, CZ, '-r');
    plot(deflectRange, CZ_d(:,rudder), '-b');
    plot(deflectRange, CZ_d_comp, '-g');
    legend('Coefficient value', 'Tornado deflection derivative', 'Alter. deflection derivative');
    title('CZ (lift)');
    xlabel('deflection (rad)');
    ylabel('CZ');
    hold off;
    
    subplot(2,3,4);
    hold on;
    plot(deflectRange, Cl, '-r');
    plot(deflectRange, Cl_d(:,rudder), '-b');
    plot(deflectRange, Cl_d_comp, '-g');
    legend('Coefficient value', 'Tornado deflection derivative', 'Alter. deflection derivative');
    title('Cl (roll)');
    xlabel('deflection (rad)');
    ylabel('Cl');
    hold off;
    
    subplot(2,3,5);
    hold on;
    plot(deflectRange, Cm, '-r');
    plot(deflectRange, Cm_d(:,rudder), '-b');
    plot(deflectRange, Cm_d_comp, '-g');
    legend('Coefficient value', 'Tornado deflection derivative', 'Alter. deflection derivative');
    title('Cm (pitch)');
    xlabel('deflection (rad)');
    ylabel('Cm');
    hold off;
    
    subplot(2,3,6);
    hold on;
    plot(deflectRange, Cn, '-r');
    plot(deflectRange, Cn_d(:,rudder), '-b');
    plot(deflectRange, Cn_d_comp, '-g');
    legend('Coefficient value', 'Tornado deflection derivative', 'Alter. deflection derivative');
    title('Cn (yaw)');
    xlabel('deflection (rad)');
    ylabel('Cn');
    hold off;

    
    % Reset original flap deflection for this flap.
    geo.flap_vector(m(rudder),n(rudder)) = bkpFlapDeflect;
    % Regenerate lattice
    [lattice,ref]=fLattice_setup2(geo,state,0);
    
end


cd(currDir); % change back to the originally current directory.
