t = linspace(0,2*pi,100);
%t = t(randperm(length(t)));
H = 18;
f1 = 1.25;
f2 = 0.8;
a = 20 *pi/180;
bias = [7.5; 1.5];
ell = [f1*H*cos(t); f2*H*sin(t)];
A = [cos(a) -sin(a); sin(a) cos(a)];

data = A*ell + 1*randn(size(ell)) + bias;

xM = (max(data(1,:))-min(data(1,:)))/2+min(data(1,:));
yM = (max(data(2,:))-min(data(2,:)))/2+min(data(2,:));

dataM = [data(1,:)-xM; data(2,:)-yM];

xMax = max(abs(dataM(1,:)));
yMax = max(abs(dataM(2,:)));

r = sqrt(dataM(1,:).^2+dataM(2,:).^2);
irMin = find(r == min(r));
irMax = find(r == max(r));

ashort = atan(dataM(2,irMin)/dataM(1,irMin));
along = atan(dataM(2,irMax)/dataM(1,irMax));
as = ashort;
al = along;
if ashort > pi/2
    ashort = ashort - pi;
elseif ashort < -pi/2
    ashort = ashort + pi;
end
distShort = [pi/2-ashort, 0-ashort, -pi/2-ashort];
if along > pi/2
    along = along - pi;
elseif along < -pi/2
    along = along + pi;
end
distLong = [pi/2-along, 0-along, -pi/2-along];

aturn = (distShort(abs(distShort) == min(abs(distShort)))+distLong(abs(distLong) == min(abs(distLong))))/2;
B = [cos(aturn) -sin(aturn); sin(aturn) cos(aturn)];
dataT = B*dataM;

xMaxT = max(abs(dataT(1,:)));
yMaxT = max(abs(dataT(2,:)));

h1 = mean(abs(dataT(1,abs(dataT(2,:)) < yMaxT/5)))
h2 = mean(abs(dataT(2,abs(dataT(1,:)) < xMaxT/5)))

%pOpt = fminunc(@(p) J(p,dataT),[1 1 1 0 0 1]);

%H1 = -sqrt(2*(pOpt(1)*pOpt(5)^2+pOpt(3)*pOpt(4)^2-pOpt(2)*pOpt(4)*pOpt(5)+(pOpt(2)^2-4*pOpt(1)*pOpt(3))*pOpt(6))*(pOpt(1)+pOpt(3)+sqrt((pOpt(1)-pOpt(3))^2+pOpt(2)^2)))/(pOpt(2)^2-4*pOpt(1)*pOpt(3))
%H2 = -sqrt(2*(pOpt(1)*pOpt(5)^2+pOpt(3)*pOpt(4)^2-pOpt(2)*pOpt(4)*pOpt(5)+(pOpt(2)^2-4*pOpt(1)*pOpt(3))*pOpt(6))*(pOpt(1)+pOpt(3)-sqrt((pOpt(1)-pOpt(3))^2+pOpt(2)^2)))/(pOpt(2)^2-4*pOpt(1)*pOpt(3))

disp(['Korrekturfaktoren: ' num2str(H/h1) ' (' num2str(1/f1) ') / ' num2str(H/h2) ' (' num2str(1/f2) ')' ])
disp(['Bias: ' num2str(xM) ' (' num2str(bias(1)) ') / ' num2str(yM) ' (' num2str(bias(2)) ')' ])

figure(1)
clf
subplot(2,2,1)
hold on
plot(data(1,:),data(2,:),'bo')
plot(18*cos(t),18*sin(t),'r')
axis equal
subplot(2,2,2)
hold on
plot(dataM(1,:),dataM(2,:),'ro')
plot([0 r(irMin)*cos(as)],[0 r(irMin)*sin(as)],'m')
plot(dataM(1,irMin),dataM(2,irMin),'xm')
plot([0 r(irMax)*cos(al)],[0 r(irMax)*sin(al)],'g')
plot(dataM(1,irMax),dataM(2,irMax),'xg')
axis equal
subplot(2,2,3)
plot(dataT(1,:),dataT(2,:),'ko')
axis equal
subplot(2,2,4)
hold on
plot(18/h1*(data(1,:)-xM),18/h2*(data(2,:)-yM),'ko')
plot(18*cos(t),18*sin(t),'r')
axis equal


function val = J(p,data)
    val = 0;
    for k=1:size(data,1)
        x = data(1,k);
        y = data(2,k);
        val = val + (p(1)*x^2+p(2)*x*y+p(3)*y^2+p(4)*x+p(5)*y+p(6)); 
    end;
end