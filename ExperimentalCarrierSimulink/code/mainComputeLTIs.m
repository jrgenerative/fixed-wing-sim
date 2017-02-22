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

% Configuration
% ===================================================================================
tornadoDirectory = 'F:\svn\dev\matlab\tornado\T135_export'; % where is Tornado installed?
nameTornadoAircraftGeometry = 'ExperimentalCarrier'; % which Tornado model to use?
estimatedVelocity = 12; % 12; % the velocity at which aerodynamic coefficients are computed.
velocityDeviationTol = 0.5; % how much the found trimmed velocity can differ from the estimated velocity to accept the solution.
airDensity = 1.225;
elevatorFlapIndex = 1; % which is the elevator flap in the Tornado model?
rudderFlapIndex = 2; % which is the rudder flap in the Tornado model?
centerOfGravity = 0.092; % in meters from zero of Tornado aircraft model reference frame. Tornado x-axis extends aft (!) -> positive.
mass = 1.56; % aircraft mass in kg (overwrites the value set in the Tornado model).
InertiaTensor = [1 0 0; 0 1 0; 0 0 1]; % inertia tensor
% The alpha-sweep configuration
alphaStart = 4; %2;
alphaEnd = 6; % 6;
numAlphas = 5;
% The beta-sweep configuration
betaStart = -1;
betaEnd = 1;
numBetas = 5;
computeStaticMargin = 0; % [0/1]
% ===================================================================================

nameLongitudinalModel = 'ExperimentalCarrier_longitudinal';
nameLateralModel = 'ExperimentalCarrier_lateral';

disp(' ');

% Compute longitudinal and lateral LTIs for trimmed gliding.    
[ltiLongitudinal, trimmedStateVariablesLongitudinal, ltiLateral, trimmedStateVariablesLateral, Cm_longitudinal, Cn_lateral] = computeLTIs(...
    tornadoDirectory, ... % absolute path to the Tornado root directory
    nameTornadoAircraftGeometry, ... % the name of the Tornado geometry file
    nameLongitudinalModel, ... % the name of the Simulink long. model.
    nameLateralModel,... % the name of the Simulink lateral model.    
    alphaStart, alphaEnd, ... % range in degrees over which coefficients are computed for the longitudinal model
    numAlphas, ... % the number of alphas to sample
    betaStart, betaEnd, ... range in degrees over which coefficients are computed for the lateral model
    numBetas, ... the number of betas to sample 
    estimatedVelocity, ... % velocity in m/s for which aerodyn. coeffs. are computed
    velocityDeviationTol, ... % factor [0 ... 1].
    airDensity, ... % kg/m^3 -> 1.225 at sea level
    elevatorFlapIndex, ...
    rudderFlapIndex, ...
    mass, ...
    centerOfGravity, ...
    InertiaTensor); % inertia tensor
trimmedAlphaRad = atan(trimmedStateVariablesLongitudinal(2)/trimmedStateVariablesLongitudinal(1)); % atan(w/u);
trimmedBetaRad = atan(trimmedStateVariablesLateral(1)/trimmedStateVariablesLongitudinal(1)); % atan(w/v);

% Compute static margin of the aircraft.
if(computeStaticMargin)
    disp('Computing static margin of longitudinal dynamics...');
    [ac, staticMarginMAC, staticMarginInMeters] = computeStaticMargin(nameTornadoAircraftGeometry, ...
        estimatedVelocity, airDensity, tornadoDirectory, centerOfGravity);
    disp(['Static margin as MAC factor  : ' num2str(staticMarginMAC)]);
    disp(['Static margin (sm)      (m)  : ' num2str(staticMarginInMeters)]);
    disp(['Aerodynamic center (ac) (m)  : ' num2str(ac)]);
    disp(['ac - sm = center of gravity  : ' num2str(ac - staticMarginInMeters)]);
end

% Longitudinal
% ========================================================================
% disp('The longitudinal LTI:');
% ltiLongitudinal

% sort complex numbers
polesLongitudinal = cplxpair(pole(ltiLongitudinal));  % sort first complex pairs (each with negative imag. part first, then pure real (ascending?)).

% Poles Map
figure();
h = subplot(2,2,1);
hold on;
plot(h, real(polesLongitudinal(1)), imag(polesLongitudinal(1)), 'rx');
plot(h, real(polesLongitudinal(2)), imag(polesLongitudinal(2)), 'bx');
plot(h, real(polesLongitudinal(3)), imag(polesLongitudinal(3)), 'ro');
plot(h, real(polesLongitudinal(4)), imag(polesLongitudinal(4)), 'bo');
hold off;
legend('phugoid mode 1', 'phugoid mode 2', 'short period mode 1', 'short period mode 2');
title('Longitudinal Poles Map');
xlabel('re');
ylabel('im');

% Step response
% Remember: for a trimmed glide, LTI state variables are all 0. Thus, the plot 
% shows the resulting deviation of state variables from the trimmed state.
% Also note: with the current model (very little air resistance -> very
% good gliding ratio, the non-linear matlab model shows that it is sensible
% that w (the velocity in body-z) is becoming negative (moving 'upwards' in
% body-z).
h = subplot(2,2,2);
elevatorDeflectionStep = 2*pi/180;
stepOpt = stepDataOptions('StepAmplitude', elevatorDeflectionStep); % step response for elevatorDeflectionStep radian elevator deflection
stepplot(h, ltiLongitudinal, stepOpt);
title('Longitudinal Step Response');
hAllAxes = findobj(gcf,'type','axes');
hLeg = findobj(hAllAxes,'tag','legend');
hAxes = setdiff(hAllAxes,hLeg); % All axes which are not
for k=1:length(hAxes)
   if ~isempty(strfind(hAxes(k).YLabel.String, '(1)'))
       hAxes(k).Title.String = ['State change for an elevator step of ' num2str(elevatorDeflectionStep*180/pi) ' degrees'];
       hAxes(k).YLabel.String = 'u (m/s)'; % velocity in body x-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(2)'))
       hAxes(k).YLabel.String = 'w (m/s)'; % velocity in body z-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(3)'))
       hAxes(k).YLabel.String = 'q (rad/s)'; % rotation velocity around y-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(4)'))
       hAxes(k).YLabel.String = 'theta (rad)'; % angle body-x vs earth-x
   end
end

% Impulse response
h = subplot(2,2,3);
impulseplot(h, ltiLongitudinal);
title('Longitudinal Impulse Response');
hAllAxes = findobj(gcf,'type','axes');
hLeg = findobj(hAllAxes,'tag','legend');
hAxes = setdiff(hAllAxes,hLeg); % All axes which are not
for k=1:length(hAxes)
   if ~isempty(strfind(hAxes(k).YLabel.String, '(1)'))
       hAxes(k).Title.String = 'State change for an elevator dirac impulse';
       hAxes(k).YLabel.String = 'u (m/s)'; % velocity in body x-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(2)'))
       hAxes(k).YLabel.String = 'w (m/s)'; % velocity in body z-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(3)'))
       hAxes(k).YLabel.String = 'q (rad/s)'; % rotation velocity around y-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(4)'))
       hAxes(k).YLabel.String = 'theta (rad)'; % angle body-x vs earth-x
   end
end

% Plot pitch moment curve
h = subplot(2,2,4);
hold on;
plot(h, [alphaStart:(alphaEnd-alphaStart)/(numAlphas-1):alphaEnd], Cm_longitudinal, 'r-');
plot(h, trimmedAlphaRad*180/pi, 0, 'bx');
hold off;
legend('Cm', ['trimmed alpha = ' num2str(trimmedAlphaRad*180/pi) ' (deg)']);
title('Pitch moment');
xlabel('alpha');
ylabel('Cm');

% Compute damping, frequency, period from poles. See also p83 in Caughey.

% Short period poles
polesShortPeriod = polesLongitudinal(3:4); % choose last ones from sorted complex numbers
polesShortPeriodReal = real(polesShortPeriod);
polesShortPeriodImag = imag(polesShortPeriod);

Damping_sp = sqrt(1./(1+(polesShortPeriodImag./polesShortPeriodReal).^2)); % Damping ratio
Frequency_sp = -polesShortPeriodReal./Damping_sp; % Undampted, natural frequency per second
Period_sp = 2*pi./(Frequency_sp.*sqrt(1-Damping_sp.^2)); % seconds
Cycles_half_sp = ((log(2))/(2*pi))*sqrt(1-Damping_sp.^2)./Damping_sp; % Number of cycles to damp to half the amplitude

% Output result for longitudinal trimmed state
disp('Short Period Mode Properties (longitudinal): ');
disp('                                                 Pole 1      Pole 2');
disp('==============================================================================');
disp(sprintf(['Damping ratio                                 : ' num2str(Damping_sp(:)')]));
disp(sprintf(['Undampted, natural frequency             (1/s): ' num2str(Frequency_sp(:)')]));
disp(sprintf(['Period                                     (s): ' num2str(Period_sp(:)')]));
disp(sprintf(['Num. cycles to damp to half the amplitude     : ' num2str(Cycles_half_sp(:)')]));
disp(' ');

% Phugoid poles
polesPhugoid = polesLongitudinal(1:2); % choose first ones from sorted complex numbers
polesPhugoidReal = real(polesPhugoid);
polesPhugoidImag = imag(polesPhugoid);

Damping_ph = sqrt(1./(1+(polesPhugoidImag./polesPhugoidReal).^2)); % Damping ratio
Frequency_ph = -polesPhugoidReal./Damping_ph; % Undampted, natural frequency per second
Period_ph = 2*pi./(Frequency_ph.*sqrt(1-Damping_ph.^2)); % seconds
Cycles_half_ph = ((log(2))/(2*pi))*sqrt(1-Damping_ph.^2)./Damping_ph; % Number of cycles to damp to half the amplitude

% Output result for longitudinal trimmed state
disp('Phugoid Mode Properties (longitudinal): ');
disp('                                                 Pole 1      Pole 2');
disp('==============================================================================');
disp(sprintf(['Damping ratio                                 : ' num2str(Damping_ph(:)')]));
disp(sprintf(['Undampted, natural frequency             (1/s): ' num2str(Frequency_ph(:)')]));
disp(sprintf(['Period                                     (s): ' num2str(Period_ph(:)')]));
disp(sprintf(['Num. cycles to damp to half the amplitude     : ' num2str(Cycles_half_ph(:)')]));
disp(' ');

% Lateral
% ========================================================================

%disp('The lateral LTI:');
%ltiLateral

% sort complex numbers
polesLateral = cplxpair(pole(ltiLateral)); % sort first complex pairs (each with negative imag. part first, then pure real (ascending?)).

% Poles Map
figure();
h = subplot(2,2,1);
hold on;
plot(h, real(polesLateral(1)), imag(polesLateral(1)), 'rx');
plot(h, real(polesLateral(2)), imag(polesLateral(2)), 'bx');
plot(h, real(polesLateral(3)), imag(polesLateral(3)), 'ro');
plot(h, real(polesLateral(4)), imag(polesLateral(4)), 'bo');
hold off;
legend('dutch roll mode 1', 'dutch roll mode 2', 'rolling mode', 'spiral mode');
title('Lateral Poles Map');
xlabel('re');
ylabel('im');

% Step response
% Remember: for a trimmed glide, LTI state variables are all 0. Thus, the plot 
% shows the resulting deviation of state variables from the trimmed state.
h = subplot(2,2,2);
rudderDeflectionStep = 2*pi/180;
stepOpt = stepDataOptions('StepAmplitude', rudderDeflectionStep); % step response for elevatorDeflectionStep radian elevator deflection
stepplot(h, ltiLateral, stepOpt);
title('Lateral Step Response');
hAllAxes = findobj(gcf,'type','axes');
hLeg = findobj(hAllAxes,'tag','legend');
hAxes = setdiff(hAllAxes,hLeg); % All axes which are not
for k=1:length(hAxes)
   if ~isempty(strfind(hAxes(k).YLabel.String, '(1)'))
       hAxes(k).Title.String = ['State change for a rudder step of ' num2str(rudderDeflectionStep*180/pi) ' degrees'];
       hAxes(k).YLabel.String = 'v (m/s)'; % velocity in body y-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(2)'))
       hAxes(k).YLabel.String = 'p (rad/s)'; % rotation velocity around x-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(3)'))
       hAxes(k).YLabel.String = 'phi (rad)'; % roll angle (euler phi)
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(4)'))
       hAxes(k).YLabel.String = 'r (rad/s)'; % rotation velocity around z-axis
   end
end

% Impulse response
h = subplot(2,2,3);
impulseplot(h, ltiLateral);
title('Lateral Impulse Response');
hAllAxes = findobj(gcf,'type','axes');
hLeg = findobj(hAllAxes,'tag','legend');
hAxes = setdiff(hAllAxes,hLeg); % All axes which are not
for k=1:length(hAxes)
   if ~isempty(strfind(hAxes(k).YLabel.String, '(1)'))
       hAxes(k).Title.String = 'State change for a rudder dirac impulse';
      hAxes(k).YLabel.String = 'v (m/s)'; % velocity in body y-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(2)'))
       hAxes(k).YLabel.String = 'p (rad/s)'; % rotation velocity around x-axis
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(3)'))
       hAxes(k).YLabel.String = 'phi (rad)'; % roll angle (euler phi)
   elseif ~isempty(strfind(hAxes(k).YLabel.String, '(4)'))
       hAxes(k).YLabel.String = 'r (rad/s)'; % rotation velocity around z-axis
   end
end

% Plot yaw moment curve
h = subplot(2,2,4);
hold on;
plot(h, [betaStart:(betaEnd-betaStart)/(numBetas-1):betaEnd], Cn_lateral, 'r-');
plot(h, trimmedBetaRad*180/pi, 0, 'bx');
hold off;
legend('Cn', ['trimmed beta = ' num2str(trimmedBetaRad*180/pi) ' (deg)']);
title('Yaw moment');
xlabel('beta');
ylabel('Cn');

% Compute damping, frequency, period from poles. See also p83 in Caughey.

% Roll pole
poleRoll = polesLateral(3);
poleRollReal = real(poleRoll);
poleRollImag = imag(poleRoll);

Damping_roll = sqrt(1./(1+(poleRollImag./poleRollReal).^2)); % Damping ratio
Frequency_roll = -poleRollReal./Damping_roll; % Undampted, natural frequency per second
Period_roll = 2*pi./(Frequency_roll.*sqrt(1-Damping_roll.^2)); % seconds
Cycles_half_roll = ((log(2))/(2*pi))*sqrt(1-Damping_roll.^2)./Damping_roll; % Number of cycles to damp to half the amplitude

disp('Roll Mode Properties (lateral): ');
disp('                                                 Pole 1');
disp('==============================================================================');
disp(sprintf(['Damping ratio                                 : ' num2str(Damping_roll(:)')]));
disp(sprintf(['Undampted, natural frequency             (1/s): ' num2str(Frequency_roll(:)')]));
disp(sprintf(['Period                                     (s): ' num2str(Period_roll(:)')]));
disp(sprintf(['Num. cycles to damp to half the amplitude     : ' num2str(Cycles_half_roll(:)')]));
disp(' ');

% Spiral pole
poleSpiral = polesLateral(4);
poleSpiralReal = real(poleSpiral);
poleSpiralImag = imag(poleSpiral);

Damping_spiral = sqrt(1./(1+(poleSpiralImag./poleSpiralReal).^2)); % Damping ratio
Frequency_spiral = -poleSpiralReal./Damping_spiral; % Undampted, natural frequency per second
Period_spiral = 2*pi./(Frequency_spiral.*sqrt(1-Damping_spiral.^2)); % seconds
Cycles_half_spiral = ((log(2))/(2*pi))*sqrt(1-Damping_spiral.^2)./Damping_spiral; % Number of cycles to damp to half the amplitude

disp('Spiral Mode Properties (lateral): ');
disp('                                                 Pole 1');
disp('==============================================================================');
disp(sprintf(['Damping ratio                                 : ' num2str(Damping_spiral(:)')]));
disp(sprintf(['Undampted, natural frequency             (1/s): ' num2str(Frequency_spiral(:)')]));
disp(sprintf(['Period                                     (s): ' num2str(Period_spiral(:)')]));
disp(sprintf(['Num. cycles to damp to half the amplitude     : ' num2str(Cycles_half_spiral(:)')]));
disp(' ');

% Dutch Roll poles
polesDutchRoll = polesLateral(1:2); % choose first ones from sorted complex numbers
polesDutchRollReal = real(polesDutchRoll);
polesDutchRollImag = imag(polesDutchRoll);

Damping_dutch = sqrt(1./(1+(polesDutchRollImag./polesDutchRollReal).^2)); % Damping ratio
Frequency_dutch = -polesDutchRollReal./Damping_dutch; % Undampted, natural frequency per second
Period_dutch = 2*pi./(Frequency_dutch.*sqrt(1-Damping_dutch.^2)); % seconds
Cycles_half_dutch = ((log(2))/(2*pi))*sqrt(1-Damping_dutch.^2)./Damping_dutch; % Number of cycles to damp to half the amplitude

% Output result for longitudinal trimmed state
disp('Dutch Roll Mode Properties (lateral): ');
disp('                                                 Pole 1      Pole 2');
disp('==============================================================================');
disp(sprintf(['Damping ratio                                 : ' num2str(Damping_dutch(:)')]));
disp(sprintf(['Undampted, natural frequency             (1/s): ' num2str(Frequency_dutch(:)')]));
disp(sprintf(['Period                                     (s): ' num2str(Period_dutch(:)')]));
disp(sprintf(['Num. cycles to damp to half the amplitude     : ' num2str(Cycles_half_dutch(:)')]));
disp(' ');

% =========================================================================

% Observability
disp(['Number of unobservable states (longitudinal)  : ' num2str(length(ltiLongitudinal.a) - rank(obsv(ltiLongitudinal)))]);
disp(['Number of unobservable states (lateral)       : ' num2str(length(ltiLateral.a) - rank(obsv(ltiLateral)))]);
disp(' ');

% Controllability
disp(['Number of uncontrollable states (longitudinal): ' num2str(length(ltiLongitudinal.a) - rank(ctrb(ltiLongitudinal)))]);
disp(['Number of uncontrollable states (lateral)     : ' num2str(length(ltiLateral.a) - rank(ctrb(ltiLateral)))]);
disp(' ');

% TODO
% ========================================================================

% 1) Run this whole thing for different center of gravity settings and plot
% Cm curve and poles.

