% CAUTION: this did not yield the desired result. The LTI matrix is slightly
% off compared to the one obtained via Simulink longitudinal model lineari-
% zation. A non-stable phugoid mode was obtained, even though it is known
% that phugoid is stable (ExperimentalCarrier aircraft).
% TODO: there might be a missing translation of aerodynamic coefficients
% from body reference to stability reference (although an initial attempt
% to transform didn't improve the LTI).
%
% Compute linear time invariant systems (longitudinal and lateral for a
% given equilibrium state.
% 
% Input: 
% A Tornado state which corresponds to a true equilibrium / trimmed flight
% condition in the non-linear Simulink simulation. Such a state includes
% (for a glider in a straight trimmed glide): 
% u: velocity in body x-axis direction (u_b_trim)
% w: velocity in body z-axis direction (w_b_trim)
% 
% 1) Compute A and B matrices for longitudinal linear time invariant
% system:
% State x = [u w q theta]'; % [forward vel, vertical vel, pitch vel, pitch ang]'
% eta = [delta_e delta_thr]';
% x_dot = A*x + B*eta;
%
% 2) Compute A and B matrices for lateral linear time invariant system:
% System:
% State x = [u w q theta]'; % [forward vel, vertical vel, pitch vel, pitch ang]'
% eta = [delta_e delta_thr]';
% x_dot = A*x + B*eta;
% 
% References:
% [1] Caughey A. David. Introduction to Aircraft Stability and Control,
% Course Notes for M&AE 5070, Cornell University, Ithaca, New York. 2011.
% [2] How P. Jonathan, Frazzoli Emilio, Chowdhary Girish. Linear Flight
% Control Techniques for Unmanned Aerial Vehicles, Chapter 1. August 2012.
%
% TODO: rotate coefficients according to stability reference frame.
% TODO: find the trimmed state with Tornado

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

% Input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Wing span
b = 1.5;

% Mean aerodynamic chord
mean_chord = 0.25;

% body u_b velocity for trimmed state (known from Simulink trimmed state)
% This is not state.AS (air speed). 
u_b_trim = 11.569; 

% body w_b velocity for trimmed state (known from Simulink trimmed state)
w_b_trim = 1.049; 

% vehicle mass
m = 1.56; % ExperimentalCarrier weight

% projected wing area 
S = 0.25*0.75 + 2*0.25*0.375*cos(20/180*pi); 

% Angle between body x-axis and earth surface. This is known from the 
% trimmed gliding state theta (6dof euler block) of the Simulink simulation
% -> 0.0619*180/pi=3.55 degrees
theta_trim = 0.0619; 

% Angle of attack for trimmed state
alpha_trim = atan(w_b_trim/u_b_trim);

% The trimmed velocity.
% This velocity is equal to airspeed (state.AS) and its vector is the x-axis of the
% stability aligned axis system.
u_0 = sqrt(u_b_trim^2 + w_b_trim^2); 

% Euler angle between earth surface and equilibrium reference frame x-axis.
% See ExperimentalCarrier\matlab\ExperimentalCarrierSimulink\
% StabilityAxisReferenceForTrimmedGliding.svg and Fig. 4.3 in [1].
theta_0 = theta_trim - alpha_trim; 

% density of air at sea level
rho = 1.225; 

% Gravity (in [1] often 32.174 appears, this is 9.81 m/s^2 in ft/s^2)
g_0 = 9.80665; % sea level (an increase in altitude from sea level to 9,000 metres (30,000 ft) causes a weight decrease of about 0.29%.)

% dynamic pressure;
Q = rho * (u_0^2) / 2;

% Inertia tensor
% Has I_xz, I_zx, and I_yz, I_zy = 0 because of aircraft symmetry.
% Note, this tensor must be computed with respect to the stability axis
% (which are typically not principal axes), and thus I_xz, I_zx are not zero.
% [I_x  I_xy I_xz]
% |I_yx I_y  I_yz|
% [I_zx I_zy I_z ]
I_y = 1; % TODO: compute this as integrate_m(x^2 + z^2)dm
I_x = 1; % TODO: compute this
I_z = 1; % TODO: compute this

% The Tornado aircraft geometry definition.
aircraft_geometry_name = 'ExperimentalCarrier';

% The tornado installation directory.
% Must contain the aircraft geometry file and the status used below.
tornado_root_directory = 'F:\svn\dev\matlab\tornado\T135_export';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define the state 
state.AS = u_0; %Airspeed m/s
state.alpha = alpha_trim; %Angle of attack, radians
state.betha = 0; %Angle of sideslip, radians
state.P = 0; %Rollrate, rad/s
state.Q = 0; %pitchrate, rad/s
state.R = 0; %yawrate, rad/s
state.adot = 0; %Alpha time derivative rad/s
state.bdot = 0; %Betha time derivative rad/s
state.ALT = 0; %Altitude, m
state.rho = rho; %Desity, kg/m^3
state.pgcorr = 0; % [0 = don't, 1 = do] apply prandtl glauert compressibility correction

% store the current directory
currDir = pwd;

% Go to Tornado directory
cd(tornado_root_directory);

% The output variable;
results = [];

% Load Tornado environment / settings
settings=config('startup');

% Load the aircraft
cd(settings.acdir)
load(aircraft_geometry_name); % aircraft geometry is now in variable 'geo'.
%geo.CG = [0.06, 0, 0];
%geo.ref_point = [0.06, 0, 0];
numRudders = nnz(geo.flapped); % number of non-zero elements

% Back to the root directory.
cd(settings.hdir)       

lattictype=0;%Tornado freestream following wake VLM

% Regenerate lattice.
[lattice,ref]=fLattice_setup2(geo,state,lattictype);

% Compute the solution
[results]=solver9(results,state,geo,lattice,ref);
[results]=coeff_create3(results,lattice,state,ref,geo);

% % To compute C_Z_alpha_dot and C_m_alpha_dot, we compute here again a
% % solution with a deltaAlpha and compute these coefficients as a
% % difference approximation using C_Z_alpha and C_m_alpha.
% resultsDAlpha = [];
% deltaAlphaRad = 0.005; % ~0.3 deg.
% origAlphaRad = state.alpha;
% state.alpha = origAlphaRad + deltaAlphaRad;
% 
% % Regenerate lattice.
% [lattice,ref]=fLattice_setup2(geo,state,lattictype);
% 
% % Compute the solution
% [resultsDAlpha]=solver9(resultsDAlpha,state,geo,lattice,ref);
% [resultsDAlpha]=coeff_create3(resultsDAlpha,lattice,state,ref,geo);
% 
% % Restore original alpha
% state.alpha = origAlphaRad;
% [lattice,ref]=fLattice_setup2(geo,state,lattictype); % just to refert lattice in case using below at some point.
% 
% %%%%%%%%%%%%%%%

% % To compute C_Z_u and C_X_u and C_m_u, we compute here again a
% % solution with a deltaU (velocity) and compute these coefficients as a
% % difference approximation using C_Z, C_X, and C_m.
% resultsDU = [];
% deltaU = 0.1; % m/s
% origU = state.AS;
% state.AS = origU + deltaU;
% 
% % Regenerate lattice.
% [lattice,ref]=fLattice_setup2(geo,state,lattictype);
% 
% % Compute the solution
% [resultsDU]=solver9(resultsDU,state,geo,lattice,ref);
% [resultsDU]=coeff_create3(resultsDU,lattice,state,ref,geo);
% 
% % Restore original velocity (air-speed)
% state.AS = origU;
% 
% %%%%%%%%%%%%%%%

% projected tail wing area 
%S_t = 0.14*0.5;

% Aspect ratio of the wing
%AR = b^2/S; % Eq. 2.3 and Eq. 2.8 [1]

% Wing efficiency factor (lifting line theory: e = 1 for elliptical lift
% distribution
%e_factor = 0.9; % a guess (some articles report 0.96 for inviscid and AR = 6, others say lower for viscid measurements) http://drydenwt.usc.edu/documents/LowReAero/SMac_JA_10.pdf

% Reynolds number density * length * velocity / kinematic viscocity
%Re = rho * 0.25 * 11.6 / (1.568*10^(-5));

%M = 0.034; % Mach number for this state.
%C_D_M = 0; % savely assumed to be 0.0 for our conditions. See also Table 5.3 in [1] for the Boeing 747 example.
%C_m_M = 0; % savely assumed to be 0.0 for our conditions. See also Eq. 5.49 in [1].

% kappa
%kappa = 1.25; % a rough guess, 1 == tail is right after the wing, 2 == tail is far from the wing.

% epsilon (reduction in angle of attack for the vertical stabilizer because of main wing down-wash
%epsilon = kappa * C_L_0 / (pi * e_factor * AR); % Eq. 2.34 [1]

% epsilon_alpha
%epsilon_alpha = (kappa / (pi * e_factor * AR)) * C_L_alpha ; % Eq. 2.35 [1]

% l_t is the distance between aerodynamic center of the wing and the stabilizer.
%l_t = 0.75 * 0.25 + 0.4 + 0.25 * 0.14; % 0.25 main wing width, 0.14 tail wing width, 0.4 distance between main and tail wings.

% eta is the tail efficiency factor,
%eta = 0.9; % Reportedly ok for low tails (1.0 for high tails) Eq. 3.3 [1]

% V_H is the tail volume parameter
%V_H = (l_t * S_t) / (mean_chord * S); % Given below Eq. 3.2 [1]

% Translate from Tornado
% ================================================================

% Signs. Standing hypothesis regarding Tornado to Matlab/[1] axis system
% conversion: alpha, beta are equal. X- and Z-axis are switched. Y-axis
% remains the same. And, if a variable which switches sign is derived
% by a variable which also switches sign this results in (-1)*(-1) = 1.
% This rule leads to the same sign conversion as empirically proven to work
% in Simulink simulation with Tornado-computed coefficients.

% Dimensionless derivatives question: Are Tornado derivatives, correctly 
% computed for this application with 'dimensionless' derivatives? 
% E.g. dimensionless pitch rate of % q^hat =  (mean_chord * pitch_rate)/2*V,
% where V == velocity (true air speed of the vehicle).
% See also [1] Eq. 3.34 and Fig. 3.3, and above Eq. 4.77.
% Answer: yes it seems clear that Tornado computes the correct dimensionless coefficient
% derivatives: can be seen in coeff_create3.m on line 133, 134.

% For longitudinal coefficients
C_X_0 = results.CX; % Don't invert, it is correct that CX is negative (even though it should be inverted due to x-axis flip from Tornado to [1] definition).
C_Z_0 = (-1)*results.CZ; % Invert, because of z-axis flip moving from Tornado to Caughy reference frame.

% For longitudinal coefficients
C_X_alpha = (-1)*results.CX_a;
C_Z_alpha = (-1)*results.CZ_a;
% Only required if want to use Z_w_dot != 0
% C_Z_alpha_dot = (-1)*(resultsDAlpha.CZ_a - results.CZ_a)/deltaAlphaRad; % computed as derivative approximation. Could also use [1] estimate given in Eq. 4.71
C_m_alpha = results.Cm_a;
% Only required if want to use M_w_dot != 0
% C_m_alpha_dot = (resultsDAlpha.Cm_a - results.Cm_a)/deltaAlphaRad; % computed as derivative approximation. Could also use the [1] estimate in Eq. 4.73

% For longitudinal coefficients
% The following coefficients could be approximated using resultsDU,
% however, coefficients are nearly velocity independent and the
% approximated derivative is actually very close to 0 (order e-16). Thus,
% leaving it at 0.0. Boeing 747
% example in [1] doesn't list *_u coefficients and leaves them at 0
% too.
C_X_u = 0.0; % (resultsDU.CX - results.CX)/deltaU; % computed as derivative approximation. TODO need to check sign.
C_Z_u = 0.0; % (resultsDU.CZ - results.CZ)/deltaU; % computed as derivative approximation. TODO need to check sign.
C_m_u = 0.0; % (resultsDU.Cm - results.Cm)/deltaU; % computed as derivative approximation. TODO need to check sign.

% For longitudinal coefficients
C_m_q = results.Cm_Q; 
C_Z_q = (-1)*results.CZ_Q;

% For lateral coefficients
C_Y_beta = results.CY_b;
C_l_beta = (-1)*results.Cl_b;
C_n_beta = (-1)*results.Cn_b;

% For lateral coefficients
C_Y_p = (-1)*results.CY_P; 
C_l_p = (-1)*(-1)*results.Cl_P; 
C_n_p = (-1)*(-1)*results.Cn_P; 

% For lateral coefficients
C_Y_r = (-1)*results.CY_R;
C_l_r = (-1)*(-1)*results.Cl_R;
C_n_r = (-1)*(-1)*results.Cn_R;

% For flap deflection
C_X_delta_e = 0; % results.CX_d(1); % we leave it at zero, since decided earlier that also not relevant for non-linear simulation (see also ExperimentalCarrier.slx model in actuator coefficient sub-system).
C_X_delta_thr = 0; % no throttle available
C_Y_delta_r = results.CY_d(2);
C_Z_delta_e = results.CZ_d(1);
C_Z_delta_thr = 0; % no throttle available
C_m_delta_e = results.Cm_d(1); % elevator index = 1, defined in Tornado geometry.
C_l_delta_a = 0; % no ailerons available, roll due to aileron deflection
C_n_delta_r = (-1)*(-1)*results.Cn_d(2); % rudder index = 2, defined in Tornado geometry.
C_n_delta_a = 0; % adverse yaw due to aileron deflection
C_l_delta_r = 0; % (-1)*(-1)*results.Cl_d(2); % we leave it at zero, since decided earlier that should derivative at 0 deflection isn't very representative (be 0 but isn't according to Tornado because of asymmetric geometryo not relevant for non-linear simulation (see also ExperimentalCarrier.slx model in actuator coefficient sub-system). % rudder index = 2, defined in Tornado geometry.

% ================================================================

% Longitudinal stability derivatives
% ==============================================
% Compare also [2] Table 1.2.

X_u = (Q*S)/(m*u_0) * (2*C_X_0 + C_X_u); % [1] Eq. 4.47: M*C_D_M with C_D_M = 0 -> 0
Z_u = (Q*S)/(m*u_0) * (2*C_Z_0 + C_Z_u); % [1] Eq. 4.53: (M^2/(1-M^2)) * C_L_0 = 0.006 ~ 0.0
M_u = (Q*S*mean_chord)/(I_y*u_0)*C_m_u; % ((Q*S*c_avg) / (I_y * u_0)) * M*C_m_M % [1] Eq. 4.55

X_w = (Q*S)/(m*u_0) * C_X_alpha; % [1] Table 4.1  
Z_w = (Q*S)/(m*u_0) * C_Z_alpha; % [1] Table 4.1  
M_w = (Q*S*mean_chord)/(I_y * u_0) * C_m_alpha; % [1] Table 4.1 

X_w_dot = 0; % [1] Table 4.1
% Is 0.00x -> discard (also discarded in [2])
Z_w_dot = 0; % (Q*S*mean_chord)/(2*m*u_0^2)*C_Z_alpha_dot; % [1] Table 4.1 
% Is 0.00x -> discard (also discarded in [2])
M_w_dot = 0; % (Q*S*mean_chord^2)/(2*I_y*u_0^2)*C_m_alpha_dot; % [1] Table 4.1

X_q = 0; % [1] Table 4.1
Z_q = (Q*S*mean_chord)/(2*m*u_0)    *C_Z_q; % [1] Table 4.1
M_q = (Q*S*mean_chord^2)/(2*I_y*u_0)*C_m_q; % [1] Table 4.1

% Control derivatives
X_delta_e = (Q*S)/(m*u_0) * C_X_delta_e; % x-force due to elevator deflection, TODO is this the correct way to go to X_delta_e?
X_delta_thr = 0; % x-force due to throttle, TODO not sure how to go from C_X_delta_thr to X_delta_thr.
Z_delta_e = (Q*S)/(m*u_0) * C_Z_delta_e; % z-force due to elevator, TODO is this the correct way to go to Z_delta_e?
Z_delta_thr = 0; % z-force due to throttle, TODO not sure how to go from C_Z_delta_thr to Z_delta_thr.
M_delta_e = (Q*S*mean_chord)/(I_y) * C_m_delta_e; % pitch moment due to elevator deflection % [1] Eq. 4.131, 
M_delta_thr = 0; % pitch moment due to throttle change,  TODO not sure how to go from C_M_delta_thr to M_delta_thr. 

% Lateral/directional stability derivatives
% ==================================================
% Compare also [2] Table 1.2.

Y_v = (Q*S)/(m*u_0)    *C_Y_beta; % [1] Table 4.2
L_v = (Q*S*b)/(I_x*u_0)*C_l_beta; % [1] Table 4.2
N_v = (Q*S*b)/(I_z*u_0)*C_n_beta; % [1] Table 4.2

Y_p = (Q*S*b)/(2*m*u_0)    *C_Y_p; % [1] Table 4.2
L_p = (Q*S*b^2)/(2*I_x*u_0)*C_l_p; % [1] Table 4.2
N_p = (Q*S*b^2)/(2*I_z*u_0)*C_n_p; % [1] Table 4.2

Y_r = (Q*S*b)/(2*m*u_0)    *C_Y_r; % [1] Table 4.2
L_r = (Q*S*b^2)/(2*I_x*u_0)*C_l_r; % [1] Table 4.2
N_r = (Q*S*b^2)/(2*I_z*u_0)*C_n_r; % [1] Table 4.2

% Control derivatives
Y_delta_r = (Q*S)/(m*u_0)*C_Y_delta_r; % y-force due to rudder deflection, TODO is this the correct way to go to Y_delta_r?
L_delta_r = (Q*S*b)/(I_x)*C_l_delta_r; % roll moment due to rudder deflection % [1] Eq. 4.125, roll due to rudder deflection
L_delta_a = (Q*S*b)/(I_x)*C_l_delta_a; % roll moment due to aileron deflection % [1] Eq 4.132, 
N_delta_r = (Q*S*b)/(I_z)*C_n_delta_r; % yawing moment due to rudder deflection % [1] Eq. 4.133 
N_delta_a = (Q*S*b)/(I_z)*C_n_delta_a; % adverse yaw due to aileron deflection % [1] Eq. 4.134, 

% LTI matrices longitudinal 
% ==================================================

% System:
% x = [u w q theta]'; % [forward vel, vertical vel, pitch vel, pitch ang]'
% eta = [delta_e delta_thr]';
% x_dot = A*x + B*eta;

% A according to [1] Eq. 5.47
% Note, can use Eq. 5.47 instad of Eq. 5.44, because Z_w_dot has been found
% to be very small (see also comment on Z_w_dot above).
% Also discard terms with M_w_dot as done in [2] (since M_w_dot very small).
% Could neglect Z_q as well as done in [1] Eq. 5.47, but Z_q for low u_0
% not so small ~ 10% (Z_q is used in [2] as well).
A_lon = [X_u,   X_w,   0        ,    -g_0*cos(theta_0);
         Z_u,   Z_w,   u_0 + Z_q,    -g_0*sin(theta_0);
         M_u,   M_w,   M_q      ,    0                ;
         0  ,   0  ,   1        ,    0                ];
 
% B according to [1] Eq. 5.47 and discarding M_w_dot as 0.
B_lon = [X_delta_e, X_delta_thr ;
         Z_delta_e, Z_delta_thr ;
         M_delta_e, M_delta_thr ;
         0        , 0           ];

% LTI matrices lateral/directional
% ===================================================

% System:
% x = [v p phi r]'; % [lateral vel, roll vel, roll ang, yaw vel]'
% eta = [delta_r delta_a]';
% x_dot = A*x + B*eta;

% A according to [1] Eq. 5.88 (the common approximate form).
A_lat = [Y_v, Y_p, g_0*cos(theta_0), Y_r - u_0;
         L_v, L_p, 0               , L_r      ;
         0  , 1  , 0               , 0        ;
         N_v, N_p, 0               , N_r      ];
 
B_lat = [Y_delta_r, 0        ;  % influence in lateral velocity
         L_delta_r, L_delta_a;
         0        , 0        ;
         N_delta_r, N_delta_a]; % influence in yaw angular velocity
 
% Save workspace
% ===================================================

cd(currDir);

save;

eig(A_lon)
eig(A_lat)
ssLon = ss(A_lon, B_lon, diag([1 1 1 1]), zeros(4,2))
ssLat = ss(A_lat, B_lat, diag([1 1 1 1]), zeros(4,2))
% linearSystemAnalyzer(ssLon);
% linearSystemAnalyzer(ssLat);


