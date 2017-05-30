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