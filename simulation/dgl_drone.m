%% Explanations:
% inputs:
%     t     = timestamp
%     x     = current state x(t)
%     rpms  = 1x4 vector with rotor rpm

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
%         phi_1
%         phi_2
%         phi_3
%         v_x
%         v_y
%         v_z
%         omega_1
%         omega_2
%         omega_3
%         q_0
%         q_1
%         q_2
%         q_3
%     ]

function dx = dgl_drone(t,x,rpms,drpms)
%% Parameters

global data;

%% Forces and torques

% Unity vectors
ex = [1; 0; 0];
ey = [0; 1; 0];
ez = [0; 0; 1];

% Schubkräfte (abhängig von Geometrie und Drehzahl)
% T(i) = BladeLift(r*omega-vel_horz) + BladeLift(r*omega+vel_horz)
T = data.cT*data.rho*data.A1*data.r^2 * rpms;

% Gesamtschubkraft (Summer der vertik. Kräfte)
F_T = [ 0      ; ...
        0      ; ...
        -sum(T) ];
    
% Horizontalkraft (der Rotoren wegen horiz. vel)
F_Hi = zeros(3,4); % F_Hi = BladeDrag(r*omega-vel_horz) + BladeDrag(r*omega+vel_horz)
F_H = sum(F_Hi,2);

% Wind Resistance
vel_horz = x(7:8);
F_W = [-data.cW*data.rho/2*data.A2 * vel_horz*norm(vel_horz); 0];

% Gravity
F_G = [0;
         0;
         data.m*data.g];

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
q = quaternion(x(13:16));
q.normalize()
F_ges = F_T + F_H + F_W + q.rotate(F_G);
    
% Total torque
M_ges = M_T + M_D + M_R + M_H;

%% Equations of motion

if t > 0
    dummy =1;
end

dx = zeros(16,1);

% Linear and Angular velocities in Body System
omega_B = x(10:12);
vel = x(7:9);

% Rotor rpms and angular acc
Omega_R = sum(data.dirs .* rpms);
dOmega_R = sum(data.dirs .* drpms);

% Calculate Euler angles from quaternion
phi   = atan2(2*(q.a*q.i+q.j*q.k),q.a^2-q.i^2-q.j^2+q.k^2);
theta = asin(2*(q.a*q.j-q.k*q.i));
psi   = atan2(2*(q.a*q.k+q.i*q.j),q.a^2+q.i^2-q.j^2-q.k^2);

% Calculate Linear Velocities in Inertial System to determine new position
% in inertial system
vel_E = [cos(psi),-sin(psi),0;sin(psi),cos(psi),0;0,0,1]...
          *[cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)]...
          *[1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)]...
          *x(7:9);

% Linear and Angular velocity in Body System
dx(1:6) = x(7:12);
      
% Linear Accelerations in Body System
dx(7:9) = F_ges - cross( omega_B, data.m * vel );

% Angular Accelerations
M_R_acc = [ 0                     ;...
            0                     ;...
            - data.I_R(3,3) * dOmega_R ];
M_gyro = cross( omega_B, data.I_Q*omega_B );
M_R_gyro = cross( omega_B, [0;0;data.I_R(3,3) * Omega_R] );
dx(10:12) = diag(1./diag(data.I_Q)) * ( M_ges + M_R_acc + M_gyro + M_R_gyro );

% Angular Position Quaternion
qneu = qmultiply(q,quaternion(0,omega_B));
dx(13:16) = 0.5*qneu.getVector();