rows = 3;
cols = 3;
subplot(rows,cols,1)
plot(t,err.h);
ylabel('err h');
subplot(rows,cols,2)
plot(t,err.h_int);
ylabel('err h int');
subplot(rows,cols,3)
plot(t,err.h_d);
ylabel('err h d');
subplot(rows,cols,4)
plot(t,u.u_h);
ylabel('u h');
subplot(rows,cols,5)
plot(t,u.u_h_int);
ylabel('u h int');
subplot(rows,cols,6)
plot(t,u.u_h_d);
ylabel('u h d');
subplot(rows,cols,7)
plot(t,u.u_ax_err);
ylabel('u ax err');
subplot(rows,cols,8)
plot(t,u.u_omega);
ylabel('u omega');