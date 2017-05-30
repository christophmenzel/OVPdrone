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
    plot(t,q1,'b',t,2*acos(q1),'r');
    ylabel('q.a');
    subplot(rows,cols,6)
    plot(t,q2);
    ylabel('q.i');
    subplot(rows,cols,9)
    plot(t,q3);
    ylabel('q.j');
    subplot(rows,cols,12)
    plot(t,q4);
    ylabel('q.k');