E = eye(3);
alpha = 45 *pi/180;
q1 = quaternion(cos(alpha/2),[0;1;0]*sin(alpha/2));
q2 = quaternion(cos(alpha/2),[1;0;0]*sin(alpha/2));
q12 = qmultiply(q2,q1);
q21 = qmultiply(q1,q2);
A = q1.rotateToBody(E);
B = q1.rotateToWorld(E);

figure(4)
clf 
subplot(1,2,1)
hold on
plot3([0 1],[0 0],[0 0],'b--');
plot3([0 0],[0 1],[0 0],'r--');
plot3([0 0],[0 0],[0 1],'m--');
plot3([0 A(1,1)],[0 A(1,2)],[0 A(1,3)],'b-o');
plot3([0 A(2,1)],[0 A(2,2)],[0 A(2,3)],'r-o');
plot3([0 A(3,1)],[0 A(3,2)],[0 A(3,3)],'m-o');
xlabel('x');
ylabel('y');
zlabel('z');
view(-30,30);
axis equal
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
zlim([-1.2 1.2]);
subplot(1,2,2)
hold on
plot3([0 1],[0 0],[0 0],'b--');
plot3([0 0],[0 1],[0 0],'r--');
plot3([0 0],[0 0],[0 1],'m--');
plot3([0 B(1,1)],[0 B(1,2)],[0 B(1,3)],'b-s');
plot3([0 B(2,1)],[0 B(2,2)],[0 B(2,3)],'r-s');
plot3([0 B(3,1)],[0 B(3,2)],[0 B(3,3)],'m-s');
xlabel('x');
ylabel('y');
zlabel('z');
view(-30,30);
axis equal
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
zlim([-1.2 1.2]);