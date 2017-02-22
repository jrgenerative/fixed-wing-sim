% Plot the 6 main coefficients vs slpha and beta.
% Input arguments are in the format of the output arguments of
% computeAerodynamicCoefficients.

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

function plotCoefficients(alpha, beta, CX, CY, CZ, Cl, Cm, Cn, CX_d, CY_d, CZ_d, Cl_d, Cm_d, Cn_d, CX_P, CY_P, CZ_P, Cl_P, Cm_P, Cn_P, CX_Q, CY_Q, CZ_Q, Cl_Q, Cm_Q, Cn_Q, CX_R, CY_R, CZ_R, Cl_R, Cm_R, Cn_R)

[X, Y] = meshgrid(alpha*180/pi, beta*180/pi);

figure(21);
subplot(2,3,1);
surf(X,Y,CX');
title('CX (longitudinal)');
xlabel('alpha');
ylabel('beta');
zlabel('longitudinal coefficient');

subplot(2,3,2);
surf(X,Y,CY');
title('CY (lateral)');
xlabel('alpha');
ylabel('beta');
zlabel('lateral coefficient');

subplot(2,3,3);
surf(X, Y, CZ');
title('CZ (lift)');
xlabel('alpha');
ylabel('beta');
zlabel('lift force coefficient');

subplot(2,3,4);
surf(X,Y,Cl');
title('Cl (roll)');
xlabel('alpha');
ylabel('beta');
zlabel('roll moment coefficient');

subplot(2,3,5);
surf(X,Y,Cm');
title('Cm (pitch)');
xlabel('alpha');
ylabel('beta');
zlabel('pitch moment coefficient');

subplot(2,3,6);
surf(X,Y,Cn');
title('Cn (yaw)');
xlabel('alpha');
ylabel('beta');
zlabel('yaw moment coefficient');

numRudders = size(Cl_d, 3);

for i = 1:numRudders
    
    figure(30+i);
    
    subplot(2,3,1);
    surf(X,Y,CX_d(:,:,i)');
    title(['Rudder ' num2str(i) ' CX_d (longitudinal derivative)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel('longitudinal coefficient derivative');

    subplot(2,3,2);
    surf(X,Y,CY_d(:,:,i)');
    title(['Rudder ' num2str(i) ' CY_d (lateral derivative)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel('lateral coefficient derivative');

    subplot(2,3,3);
    surf(X, Y, CZ_d(:,:,i)');
    title(['Rudder ' num2str(i) ' CZ_d (lift derivative)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel('lift force coefficient derivative');
    
    subplot(2,3,4);
    surf(X,Y,Cl_d(:,:,i)');
    title(['Rudder ' num2str(i) ' Cl_d (roll derivative)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel('roll moment coefficient derivative');

    subplot(2,3,5);
    surf(X,Y,Cm_d(:,:,i)');
    title(['Rudder ' num2str(i) ' Cm_d (pitch derivative)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel('pitch moment coefficient derivative');

    subplot(2,3,6);
    surf(X,Y,Cn_d(:,:,i)');
    title(['Rudder ' num2str(i) ' Cn_d (yaw derivative)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel('yaw moment coefficient derivative');
   
end

CX_PQR = zeros(size(X,2), size(Y,1), 3);
CX_PQR(:,:,1) = CX_P;
CX_PQR(:,:,2) = CX_Q;
CX_PQR(:,:,3) = CX_R;

CY_PQR = zeros(size(X,2), size(Y,1), 3);
CY_PQR(:,:,1) = CY_P;
CY_PQR(:,:,2) = CY_Q;
CY_PQR(:,:,3) = CY_R;

CZ_PQR = zeros(size(X,2), size(Y,1), 3);
CZ_PQR(:,:,1) = CZ_P;
CZ_PQR(:,:,2) = CZ_Q;
CZ_PQR(:,:,3) = CZ_R;

Cl_PQR = zeros(size(X,2), size(Y,1), 3);
Cl_PQR(:,:,1) = Cl_P;
Cl_PQR(:,:,2) = Cl_Q;
Cl_PQR(:,:,3) = Cl_R;

Cm_PQR = zeros(size(X,2), size(Y,1), 3);
Cm_PQR(:,:,1) = Cm_P;
Cm_PQR(:,:,2) = Cm_Q;
Cm_PQR(:,:,3) = Cm_R;

Cn_PQR = zeros(size(X,2), size(Y,1), 3);
Cn_PQR(:,:,1) = Cn_P;
Cn_PQR(:,:,2) = Cn_Q;
Cn_PQR(:,:,3) = Cn_R;

for i=1:3
   
    if i == 1
        axis = 'X';
        variable = 'P';
    elseif i == 2
       axis = 'Y';
       variable = 'Q';
    elseif i == 3
       axis = 'Z';
       variable = 'R';
    end
    
    figure(40+i);
    subplot(2,3,1);
    surf(X,Y, CX_PQR(:,:,i)');
    title(['CX_' variable ' (longitudinal)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel(['longitudinal damping force coefficient rotating around ' axis]);

    subplot(2,3,2);
    surf(X,Y,CY_PQR(:,:,i)');
    title(['CY_' variable ' (lateral)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel(['lateral damping force coefficient rotating around ' axis]);

    subplot(2,3,3);
    surf(X, Y, CZ_PQR(:,:,i)');
    title(['CZ_' variable ' (lift)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel(['lift damping force coefficient rotating around ' axis]);

    subplot(2,3,4);
    surf(X,Y,Cl_PQR(:,:,i)');
    title(['Cl_' variable ' (roll)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel(['roll damping moment coefficient rotating around ' axis]);

    subplot(2,3,5);
    surf(X,Y,Cm_PQR(:,:,i)');
    title(['Cm_' variable ' (pitch)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel(['pitch moment coefficient rotating around ' axis]);

    subplot(2,3,6);
    surf(X,Y,Cn_PQR(:,:,i)');
    title(['Cn_' variable ' (yaw)']);
    xlabel('alpha');
    ylabel('beta');
    zlabel(['yaw moment coefficient rotating around ' axis]);
    
end

