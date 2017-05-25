global data;

% Drone
data.m = 1;
data.l = 1;
data.h = 1;
data.r = 1;
data.cT = 1;
data.cW = 1;
data.A1 = 1;
data.A2 = 1;
data.dirs = [1 -1 1 -1];
data.I_x = 1;
data.I_y = 1;
data.I_z = 1;
data.I_Rx = 1;
data.I_Ry = 1;
data.I_Rz = 1;

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

% Anfangsbedingungen
t0 = 0;
t1 = 10;
dt = 0.01;

pos = [0,0,0]';
vel = [0,0,0]';
ang = [0,0,0]';
rot = [1,1,0]';

x0 = [pos; vel; ang; rot; 1; 0; 0; 0];

u = 1*[0 0 0 0];
du = zeros(1,4);

% Main Loop
% figure(1)
% hold on
% for t = t0:dt:t1
%     [t,x] = ode45(@(t,x) dgl_drone(t,x,u,du),[t t+dt],x(end,:));
%     plot(t,x(:,3),'b',t,x(:,9),'r',t,x(:,13),'k');
% end

[t,x] = ode45(@(t,x) dgl_drone(t,x,u,du),[t0 t1],x0);

figure(1)
clf
% Linear Position and velocity
subplot(3,2,1)
plot(t,x(:,1),'b',t,x(:,7),'r');
subplot(3,2,3)
plot(t,x(:,2),'b',t,x(:,8),'r');
subplot(3,2,5)
plot(t,x(:,3),'b',t,x(:,9),'r');
% Angular position and velocity
subplot(3,2,2)
plot(t,x(:,4),'b',t,x(:,10),'r');
subplot(3,2,4)
plot(t,x(:,5),'b',t,x(:,11),'r');
subplot(3,2,6)
plot(t,x(:,6),'b',t,x(:,12),'r');

figure(2)
subplot(4,1,1)
plot(t,x(:,13));
subplot(4,1,2)
plot(t,x(:,14));
subplot(4,1,3)
plot(t,x(:,15));
subplot(4,1,4)
plot(t,x(:,16));