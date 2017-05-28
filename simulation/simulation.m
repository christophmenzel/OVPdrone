clear all

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
data.dirs = [1 -1 1 -1];
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

% Anfangsbedingungen
t0 = 0;
t1 = 1;
dt = 0.01;

pos = [0,0,0]';
vel = [0,0,0]';
ang = [0,0,0]';
rot = [0,0,0]';

x0 = [pos; vel; 1; 0; 0; 0; rot];

u = 1*[0 1 0 0];
du = zeros(1,4);

[t,x] = ode45(@(t,x) dgl_drone(t,x,u,du),[t0:dt:t1],x0);

% Re-Calculate global states
for k = 1:length(t)
    q = quaternion(x(k,7:10));
    vel_E(1:3,k) = q.rotateToWorld(x(k,4:6)');
    phi(k)   = atan2(2*(q.a*q.i+q.j*q.k),q.a^2-q.i^2-q.j^2+q.k^2);
    theta(k) = asin(2*(q.a*q.j-q.k*q.i));
    psi(k)   = atan2(2*(q.a*q.k+q.i*q.j),q.a^2+q.i^2-q.j^2-q.k^2);
    omega_E(1:3,k) = q.rotateToWorld(x(k,11:13)');
end

% Plot 3D Model
valprev = x(1,:);
for j = 1:length(t)
    val = x(j,:)';
    q = quaternion(val(7:10));
    figure(3)
    clf;
    hold on;
    arms = data.l*[0 1 0 0 -1 0 0; 0 0 -1 0 0 0 1; 0 0 0 0 0 0 0];
    arms_rot = q.rotateToWorld(arms);
    plot3(arms_rot(1,1:2)+val(1),arms_rot(2,1:2)+val(2),-1*(arms_rot(3,1:2)+val(3)),'r');
    plot3(arms_rot(1,3:7)+val(1),arms_rot(2,3:7)+val(2),-1*(arms_rot(3,3:7)+val(3)),'k');
    plot3([val(1) val(1)],[val(2) val(2)],-1*[0 val(3)],'b--o');
    plot3(x(1:j,1),x(1:j,2),-1*x(1:j,3),'r');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    view(-30,30)
    axis equal
    xlim([val(1)-1 val(1)+1]);
    ylim([val(2)-1 val(2)+1]);
    zlim([-1*val(3)-1 -1*val(3)+1]);
    drawnow
    pause(dt);
    valprev = val;
end

plotGraphs = true;
if plotGraphs
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
    
end