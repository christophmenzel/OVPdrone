clear all

% Create Drone and Environment
physicalSettings;

% Simulation Settings
t0 = 0;
t1 = 1;
dt = 0.01;
t_ges = [t0:dt:t1];

% Mission Settings
h_soll = -1;
g_soll = [0;0;1];

% Controller Settings
Kh_P = -0.5 * ones(4,1);
Kh_I = -3   * ones(4,1);
Kh_D = -10  * ones(4,1);
Pq = 1  * [0  1 0; -1 0 0; 0 -1 0; 1  0 0];
Pw = -1 * [0 1 0; -1 0 0; 0 -1 0; 1 0 0];

% Initial State
pos = [0,0,0]';
vel = [0,0,0]';
ang = [0,0,0]';
rot = [0,0,0]';
x0 = [pos; vel; 1; 0; 0; 0; rot];
alpha0 = 0*pi/180;
q0 = quaternion(cos(alpha0/2),sin(alpha0/2)*[0; 0; 1]);

% Control Variables
u0 = [0; 0; 0; 0];
du = [0; 0; 0; 0];

% Disturbances
FD = @(t) interp1([0 t1/2 t1/2+dt t1/2+5*dt t1/2+6*dt t1],[0 0 0 0  0 0],t);
MD = @(t) interp1([0 t1/2 t1/2+dt t1/2+5*dt t1/2+6*dt t1],[0 0 0 0  0 0],t);

% Start Simulation
initializeMainLoop;
for tt = t_ges
    
    % Update the linear state of the physical drone
    dx = dgl_drone(tt, x_run, q_run, u_run, du, 0, 0);
    % Use explicit Euler Method for linear state
    x_run([1:6 11:13]) = x_run([1:6 11:13]) + dt*dx([1:6 11:13]);
    
    % Update the angular state of the physical drone
    omega_B = x_run(11:13);
    alpha = dt*norm(omega_B);
    % Rotate the orientation axis with the current body rotation
    if norm(x_run(11:13)) ~= 0
        normfac = norm(x_run(11:13));
    else
        normfac = 1;
    end
    dq = quaternion(cos(alpha/2),x_run(11:13)/normfac*sin(alpha/2));
    q_run = qmultiply(q_run,dq);
    if q_run.a < 0
        q_run = quaternion(-q_run.a,-q_run.i,-q_run.j,-q_run.k);
    end
    q_run = q_run.normalize();
    
    % Calculate the control errors
    err_h = (h_soll - x_run(3));
    err_h_d = (err_h - err_h_prev)/dt;
    err_h_prev = err_h;
    err_h_int = err_h_int + dt*err_h; 
    ax_err = cross( g_soll, q_run.rotateFromWorldToBody(g_soll) );
    
    % Calculate the rotor rpms and their derivative
    u_old = u_run;
    u_h = Kh_P*err_h;
    u_h_int = Kh_I*err_h_int;
    u_h_d = Kh_D * err_h_d;
    u_ax_err = Pq * ax_err;
    u_omega = Pw * x_run(11:13);
    u_run = u_h + u_h_int + u_h_d +  u_ax_err + u_omega;
    du = (u_run - u_old)/dt;
    
    u_run = 0 * u_run;
    du = 0 * du;
    
    % Save data for plotting
    t(end+1,1) = tt;
    x(end+1,:) = x_run';
    q(end+1) = q_run;
    err.h(end+1) = err_h;
    err.h_int(end+1) = err_h_int;
    err.h_d(end+1) = err_h_d;
    u.u_ges(end+1,1:4) = u_run';
    u.u_h(end+1,1:4) = u_h';
    u.u_h_int(end+1,1:4) = u_h_int';
    u.u_h_d(end+1,1:4) = u_h_d';
    u.u_ax_err(end+1,1:4) = u_ax_err';
    u.u_omega(end+1,1:4) = u_omega';
    
    dummy = 1;
end

% Calculate result states in world frame
for k = 1:length(t)
    q_mom = q(k);
    q1(k) = q_mom.a;
    q2(k) = q_mom.i;
    q3(k) = q_mom.j;
    q4(k) = q_mom.k;
    vel_E(1:3,k) = q_mom.rotateFromBodyToWorld(x(k,4:6)');
    phi(k)   = atan2(2*(q_mom.a*q_mom.i+q_mom.j*q_mom.k),q_mom.a^2-q_mom.i^2-q_mom.j^2+q_mom.k^2);
    theta(k) = asin(2*(q_mom.a*q_mom.j-q_mom.k*q_mom.i));
    psi(k)   = atan2(2*(q_mom.a*q_mom.k+q_mom.i*q_mom.j),q_mom.a^2+q_mom.i^2-q_mom.j^2-q_mom.k^2);
    omega_E(1:3,k) = q_mom.rotateFromBodyToWorld(x(k,11:13)');
end

%% Plot 3D Model
plotMovie = true;
if plotMovie
    for j = 1:length(t)
        val = x(j,:)';
        q_mom = q(j);
        ax = q_mom.getVector();
        if ax(1) > 0
            alpha = 2*acos(ax(1));
            rotAx = ax(2:4);
        else
            alpha = 2*acos(-ax(1));
            rotAx = -ax(2:4);
        end
        figure(5)
        clf;
        hold on;
        arms = data.l*[0 1 0 0 -1 0 0; 0 0 -1 0 0 0 1; 0 0 0 0 0 0 0];
        arms_rot = q_mom.rotateFromBodyToWorld(arms);
        vel_rot = q_mom.rotateFromBodyToWorld(val(4:6));
        plotting(arms_rot(:,1:2)+val(1:3),'r');
        plotting(arms_rot(:,3:7)+val(1:3),'b');
        lot = [val(1) val(1); val(2) val(2); 0 val(3)];
        plotting(lot,'b--o');
        plotting(x(1:j,1:3)','r');
        vel_vec = [0 vel_rot(1); 0 vel_rot(2); 0 vel_rot(3)]/norm(vel_rot);
        plotting(0.5*vel_vec+val(1:3),'g-o');
        texting(vel_rot/(2*norm(vel_rot))+val(1:3),num2str(norm(vel_rot)));
        rotAxis = [0 rotAx(1); 0 rotAx(2); 0 rotAx(3)];
        plotting(rotAxis+val(1:3),'m-o');
        texting(rotAx+val(1:3),num2str(alpha*180/pi));
        title(['V = ' num2str(u.u_ges(j,1)) ' / L = ' num2str(u.u_ges(j,2)) ' / H = ' num2str(u.u_ges(j,3)) ' / R = ' num2str(u.u_ges(j,4))]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view(-30,30)
        axis equal
        xlim([val(1)-1 val(1)+1]);
        ylim([-val(2)-1 -val(2)+1]);
        zlim([-val(3)-1 -val(3)+1]);
        drawnow
    end
end

%% Plot graphs
plotGraphs = true;
if plotGraphs
    
    % Plot body frame data
    figure(1)
    plotBodyFrameData;
    
    % Plot world frame data
    figure(2)
    plotWorldFrameData;
    
    % Plot control errors and variables
    figure(3)
    plotErrorsAndControls;
    
end