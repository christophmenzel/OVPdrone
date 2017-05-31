global data;

% Drone
data.m = 2; % kg - total drone mass
data.l = 0.5; % m - arm length
data.h = 0.05; % m - height of rotor COG above drone COG
data.r = 0.1; % m - rotor radius
data.cT = 1; % unitless - lift coefficient
data.cW = 0; % drag coefficient
data.A1 = 1;
data.A2 = 1;
data.dirs = [1; -1; 1; -1];
data.I_x = 0.01;
data.I_y = 0.01;
data.I_z = 0.01;
data.I_Rx = 0.001;
data.I_Ry = 0.001;
data.I_Rz = 0.001;

% Intertia
I_B = diag([data.I_x,data.I_y,data.I_z]);
I_R_B = [4*data.h^2+2*data.l^2 0 0;...
    0 4*data.h^2+2*data.l^2 0;...
    0 0      4*data.l^2];
data.I_R = diag([data.I_Rx,data.I_Ry,data.I_Rz]);
data.I_Q = I_B + I_R_B + 4*data.I_R;

% Environment
data.g = 0; %9.81;
data.rho = 1;