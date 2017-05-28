clear all

global data;

% Drone
data.m = 1;
data.l = 1;
data.h = 1;
data.r = 1;
data.cT = 1;
data.cW = 0;
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
data.g = 9.81; %9.81;
data.rho = 1;

% Anfangsbedingungen
t0 = 0;
t1 = 10;
dt = 0.01;

pos = [0,0,0]';
vel = [0,0,0]';
ang = [0,0,0]';
rot = [0,0,0]';

x0 = [pos; vel; 1; 0; 0; 0; rot];

u = 2.5*[1 1 1 1];
du = zeros(1,4);

% % Calculating body states over time
% x_run = x0;
% tt = t0:dt:t1;
% x = zeros(length(tt,size(x,1)));
% for j = 1:length(tt)
%     t = tt(j);
%     [dv, dq] = get_acc_and_dq(x_run,u);
%     x = update_vel_and_q(x,dv,dq);
%     [dx, dO] = get_(x_run,u,du);
%     x(j,:) = 
% end

[t,x] = ode45(@(t,x) dgl_drone(t,x,u,du),[t0 t1],x0);

% Re-Calculate global states
for k = 1:length(t)
    q = quaternion(x(k,7:10));
    vel_E(1:3,k) = q.rotate(x(k,4:6));
    phi(k)   = atan2(2*(q.a*q.i+q.j*q.k),q.a^2-q.i^2-q.j^2+q.k^2);
    theta(k) = asin(2*(q.a*q.j-q.k*q.i));
    psi(k)   = atan2(2*(q.a*q.k+q.i*q.j),q.a^2+q.i^2-q.j^2-q.k^2);
    omega_E(1:3,k) = q.rotate(x(k,11:13));
end

% Plot body coordinates
figure(1)
rows = 4;
cols = 3;
clf;
% Linear position and velocity
subplot(rows,cols,1)
plot(t,x(:,4));
ylabel('vx');
subplot(rows,cols,4)
plot(t,x(:,5));
ylabel('vy');
subplot(rows,cols,7)
plot(t,x(:,6));
ylabel('vz');
subplot(rows,cols,2)
plot(t,x(:,11));
ylabel('dOx');
subplot(rows,cols,5)
plot(t,x(:,12));
ylabel('dOy');
subplot(rows,cols,8)
plot(t,x(:,13));
ylabel('dOz');
% Angular position and velocity
subplot(rows,cols,3)
plot(t,x(:,7));
ylabel('q.a');
subplot(rows,cols,6)
plot(t,x(:,8));
ylabel('q.i');
subplot(rows,cols,9)
plot(t,x(:,9));
ylabel('q.j');
subplot(rows,cols,12)
plot(t,x(:,10));
ylabel('q.k');


% Plot world coordinates
figure(2)
rows = 6;
cols = 2;
clf;
% Linear position and velocity
subplot(rows,cols,1)
plot(t,x(:,1));
ylabel('x');
subplot(rows,cols,3)
plot(t,x(:,2));
ylabel('y');
subplot(rows,cols,5)
plot(t,x(:,3));
ylabel('z');
subplot(rows,cols,7)
plot(t,vel_E(1,:));
ylabel('vx');
subplot(rows,cols,9)
plot(t,vel_E(2,:));
ylabel('vy');
subplot(rows,cols,11)
plot(t,vel_E(3,:));
ylabel('vz');
% Angular position and velocity
subplot(rows,cols,2)
plot(t,phi);
ylabel('phi');
subplot(rows,cols,4)
plot(t,theta);
ylabel('theta');
subplot(rows,cols,6)
plot(t,psi);
ylabel('psi');
subplot(rows,cols,8)
plot(t,omega_E(1,:));
ylabel('dphi');
subplot(rows,cols,10)
plot(t,omega_E(2,:));
ylabel('dtheta');
subplot(rows,cols,12)
plot(t,omega_E(3,:));
ylabel('dpsi');