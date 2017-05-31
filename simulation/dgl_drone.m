%% Explanations:
% inputs:
%     t     = timestamp
%     x     = current state x(t)
%     rpms  = 1x4 vector with rotor speed
%     drpms = 1x4 vector with rotor acceleration

% Naming/indexing convention
% 1 = front
% 2 = left
% 3 = rear
% 4 = right

% state vector 
%     x = [
%         x
%         y
%         z
%         v_x
%         v_y
%         v_z
%         q_0
%         q_1
%         q_2
%         q_3
%         omega_1
%         omega_2
%         omega_3
%     ]

function dx = dgl_drone(t,x,q,rpms,drpms,F_Dist,M_Dist)
%% Preparation

global data;

% Result vector
dx = zeros(size(x));

% Get linear and angular velocities in Body System
vel = x(4:6);
vel_horz = vel(1:2);
omega_B = x(11:13);

% Get rotor angular vel and acc (Stellgrößen)
Omega_R = sum(data.dirs .* rpms);
dOmega_R = sum(data.dirs .* drpms);

% Unity vectors
ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];

%% Forces and torques

% Schubkräfte (abhängig von Geometrie und Drehzahl)
% T(i) = BladeLift(r*omega-vel_horz) + BladeLift(r*omega+vel_horz)
T = data.cT * rpms; %data.cT*data.rho*data.A1*data.r^2

% Gesamtschubkraft (Summer der vertik. Kräfte)
F_T = [ 0      ; ...
        0      ; ...
        -sum(T) ];
    
% Horizontalkraft (der Rotoren wegen horiz. vel)
F_Hi = zeros(3,4); % F_Hi = BladeDrag(r*omega-vel_horz) + BladeDrag(r*omega+vel_horz)
F_H = sum(F_Hi,2);

% Wind Resistance
F_W = [-data.cW*data.rho/2*data.A2 * vel_horz*norm(vel_horz); 0];

% Gravity
F_G = [0; 0; data.m*data.g];

% Drehmoment aus Schubkräften (abhängig von Geometrie)
M_T = [ data.l*(T(2) - T(4)); ...
        data.l*(T(1) - T(3)); ...
        0              ];

% Bremsmoment (Widerstand entgegen der Rotordrehung)
M_Di = zeros(3,4); % M_Di = const * omega^2;
M_D = sum(M_Di,2);

% Rollmoment (Kippmomente der Rotoren wegen horiz. vel)
M_Ri = zeros(3,4); % M_Ri = const * vec_vel * omega(^2?)
M_R = sum(M_Ri,2);

% Hängmoment (aus den Horizontalkräften der Rotoren)
M_H = [ data.h * ey'*F_H                                           ;...
        data.h * ex'*F_H                                           ;...
        data.l * ( F_Hi(2,1) - F_Hi(2,3) + F_Hi(1,2) - F_Hi(1,4) )]; % 

% Total force
F_ges = F_T + F_H + F_W + q.rotateFromWorldToBody(F_G + F_Dist*[0;0;1]);
    
% Total torque
M_ges = M_T + M_D + M_R + M_H + q.rotateFromWorldToBody(M_Dist*[0;1;0]);

%% Equations of motion

% % Calculate Euler angles from quaternion
% phi   = atan2(2*(q.a*q.i+q.j*q.k),q.a^2-q.i^2-q.j^2+q.k^2);
% theta = asin(2*(q.a*q.j-q.k*q.i));
% psi   = atan2(2*(q.a*q.k+q.i*q.j),q.a^2+q.i^2-q.j^2-q.k^2);
% 
% % Calculate Linear Velocities in Inertial System to determine new position
% % in inertial system
% vel_E = [cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1]...
%           *[cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)]...
%           *[1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)]...
%           *vel;
% 
% % Velocity in Inertial System from quaternion and body velocity
% dx(1:3) = [q.a^2+q.i^2-q.j^2-q.k^2, 2*(q.i*q.j-q.a*q.k), 2*(q.a*q.j+q.i*q.k);...
%            2*(q.i*q.j+q.a*q.k), q.a^2-q.i^2+q.j^2-q.k^2, 2*(q.j*q.k-q.a*q.i);...
%            2*(q.i*q.k-q.a*q.j), 2*(q.j*q.k+q.a*q.i), q.a^2-q.i^2-q.j^2+q.k^2]...
%            *vel;

% Velocity in Inertial System from quaternion and body velocity
dx(1:3) = q.rotateFromBodyToWorld(vel);
newVel = dx(1:3);

% Linear Accelerations in Body System
dx(4:6) = F_ges/data.m - 2*cross(omega_B, vel);
newAcc = dx(4:6);

% Angular Accelerations
M_R_acc = [ 0; 0; - data.I_R(3,3) * dOmega_R ];
M_gyro = cross( omega_B, data.I_Q*omega_B );
M_R_gyro = cross( omega_B, [0;0;data.I_R(3,3) * Omega_R] );
dx(11:13) = diag(1./diag(data.I_Q)) * ( M_ges + M_R_acc + M_gyro + M_R_gyro );
newDo = dx(11:13);

dummy = 1;
