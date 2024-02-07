function doty_output=Robotnet(t,y)

global q0 obstacle obstacle_range_2 dq0 omega beta neta zeta T r upperq lowerq upperdq lowerdq initPos robot first subTime gamma noise

tj = clock;
q = y(1:6);
%w = y(7:15);
w = y(7:18);
dq = w(1:6);
%JHat = reshape(y(16:33),[3,6]);
JHat = reshape(y(19:36),[3,6]);
JCHat = reshape(y(37:54),[3,6]);

%======= Desired path, velocity=======================
rx = initPos(1) + r*cos(2*pi*(sin(0.5*pi*t/T)).^2)-r;
ry = initPos(2) + r*cos(pi/6)*sin(2*pi*(sin(0.5*pi*t/T)).^2);
rz = initPos(3) + r*sin(pi/6)*sin(2*pi*(sin(0.5*pi*t/T)).^2);

drx= -(2*r*pi^2*cos((pi*t)/(2*T))*sin((pi*t)/(2*T))*sin(2*pi*sin((pi*t)/(2*T))^2))/T;
dry= (3^(1/2)*r*pi^2*cos((pi*t)/(2*T))*sin((pi*t)/(2*T))*cos(2*pi*sin((pi*t)/(2*T))^2))/T;
drz= (r*pi^2*cos((pi*t)/(2*T))*sin((pi*t)/(2*T))*cos(2*pi*sin((pi*t)/(2*T))^2))/T;


rd=[rx;ry;rz];
drd=[drx;dry;drz];
%======= Desired path, velocity=======================

%======================= obstacle avoid ===========================
ra = kinovaJacoJ2N6S300position(robot,q,7);
jointPos = zeros(7,3);
%jointPos(1,:) = tform2trvec(getTransform(robot,qs','j2n6s300_link_1'));
for j = 1:6
    jointPos(j,:) = kinovaJacoJ2N6S300position(robot,q,j);
end
jointPos(7,:) = ra;
C = zeros(6,3);
R = zeros(6,3);
distance = zeros(6,1);
for i = 1:6
    [temp1,temp2,temp3] = GetCPosition(obstacle,jointPos(i,:)',jointPos(i+1,:)');
    C(i,:) = temp1;
    R(i,:) = temp2;
    distance(i) = temp3;
end

[min_distance,l] = min(distance);

if min_distance > obstacle_range_2 && first == 0
    first = 1;
    dq0 = [0;0;0;0;0;0];
end

if min_distance <= obstacle_range_2 && first == 1
%if min_distance == obstacle_range_2
    %abs(min_distance - obstacle_range_2)
    dq0 = dq;
    first = 0;
end

mu = omega * (q - q0);
vmax = min(beta * (upperq - q), upperdq);
vmin = max(beta * (lowerq - q), lowerdq);
PInf = 1e10 * ones(3,1);
MInf = -PInf;
pdnn_up = [vmax;PInf;PInf];
pdnn_lp = [vmin;MInf;zeros(3,1)];
%pdnn_dw = zeros(12,1);

A = zeros(3,6);
b = zeros(3,1);

subTime = subTime + etime(clock, tj);
if min_distance < obstacle_range_2
    %JC = getJC(robot,l,C(l,:),jointPos,q);
    
    %JC = geometricJacobian(robot,[q;0;0;0],'j2n6s300_link_3');
    %JC = JC(4:6,1:6);
    A = newJC(R(l,:),JCHat);
    %sgnx = sgn(obstacle(1)-C(l,1));
    %sgny = sgn(obstacle(2)-C(l,2));
    %sgnz = sgn(obstacle(3)-C(l,3));
    %A = [sgnx*JC(1,:);sgny*JC(2,:);sgnz*JC(3,:)];
    %abs(min_distance)
    %dq0
    b = s(min_distance) * max(A*dq0,0)
    %b = s(min_distance) * [A(1,:)*dq0;A(2,:)*dq0;A(3,:)*dq0]
%else
%   Q = [eye(6,6),-JHat';
%    JHat,zeros(3,3)];
%    P = [mu;-drd];
%    pdnn_m = w(1:9) - (Q * w(1:9) + P);
%    pdnn_n = F(pdnn_lp,pdnn_m,pdnn_up) - w(1:9);
%    pdnn_dw(1:9,1) = neta * (eye(9,9)+Q') * pdnn_n;
end
    
Q = [eye(6,6),-JHat',A';
    JHat,zeros(3,3),zeros(3,3);
    -A,zeros(3,3),zeros(3,3)];
new_drd = drd+gamma*(rd - ra');
P = [mu;-new_drd;b];
% P = [mu;-drd;b];
pdnn_m = w - (Q * w + P);
pdnn_n = F(pdnn_lp,pdnn_m,pdnn_up) - w;
pdnn_dw = neta * (eye(12,12)+Q') * pdnn_n;

%======================= obstacle avoid ===========================

%=======================
ddq = pdnn_dw(1:6);
%=======================

%========== Simulate the robot =============

[J,DJ]=kinovaJacoJ2N6S300jdj(robot,q,dq);
dra = J*dq+noise*sin(t/T*8*pi);
% dra = J*dq;
ddra = DJ*dq+J*ddq;

JC = getJC(robot,l,C(l,:),jointPos,q);

delta_t = 1e-16;
q_new = q + dq * delta_t;
new_jointPos = zeros(7,3);
for j = 1:7
    new_jointPos(j,:) = kinovaJacoJ2N6S300position(robot,q_new,j);
end
JC_new = getJC(robot,l,C(l,:),new_jointPos,q_new);
DJC = (JC_new - JC)./delta_t;

drac = JC*dq+noise*sin(t/T*8*pi);
% drac = JC*dq;
ddrac = DJC*dq+JC*ddq;
%========== Simulate the robot =============

%============= Estimate Jacobian matrix ========
DJHat = (ddra-JHat*ddq+zeta*(dra-JHat*dq)) * pinv(dq);
DJCHat = (ddrac-JCHat*ddq+zeta*(drac-JCHat*dq)) * pinv(dq);
%============= Estimate Jacobian matrix ========

doty_output=[dq;pdnn_dw;reshape(DJHat,[18,1]);reshape(DJCHat,[18,1])];


t
end