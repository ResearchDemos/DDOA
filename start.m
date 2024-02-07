clear all;
clc
format long;
addpath('func');
global q0 obstacle obstacle_range_1 obstacle_range_2 dq0 omega beta neta zeta T r upperq lowerq upperdq lowerdq initPos first subTime gamma noise
global robot

%=========== kinovaJacoJ2N6S300 ==============
robot = loadrobot('kinovaJacoJ2N6S300','DataFormat','column','Gravity',[0 0 -9.81]);
%=========== kinovaJacoJ2N6S300  ==============

%initial joint angle
q0 = [-1.659;4.474;1.665;4.195;1.740;2.309];
first = 1;
%initial obstacle position
obstacle = [-0.075,-0.48,0.16]';
obstacle_range_1 = 0.065;
obstacle_range_2 = 0.09;
% obstacle_range_1 = 0;
% obstacle_range_2 = 0;
dq0 = [0;0;0;0;0;0];
%initial parameters
omega = 0.1;
qlimit = 1; 
dqlimit = 1;
beta = 0.25;
neta = 100;
zeta = 1; 
gamma = 200;                    
subTime = 0;
noise = 8e-7;
% noise = 0;

T = 10; %task duration
r = 0.04; %scale of the path

%joint limit
upperq = qlimit * [2*pi;2*pi;2*pi;2*pi;2*pi;2*pi];
lowerq = -upperq;

%joint velocity limit
upperdq = dqlimit * [inf;inf;inf;inf;inf;inf];
lowerdq = -upperdq;

%w0 = zeros(9,1);
w0 = zeros(12,1);
init = [q0;w0];

%======= Initialize Jacobian matrix =============
[JHat,~] = kinovaJacoJ2N6S300jdj(robot,q0,w0(1:6));

ra = kinovaJacoJ2N6S300position(robot,q0,7);
jointPos = zeros(7,3);
%jointPos(1,:) = tform2trvec(getTransform(robot,qs','j2n6s300_link_1'));
for j = 1:6
    jointPos(j,:) = kinovaJacoJ2N6S300position(robot,q0,j);
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
JC = getJC(robot,l,C(l,:),jointPos,q0);

%======= Initialize Jacobian matrix =============

% initial position of the end effector 
initPos = zeros(3,1);
initPos(:,1) = kinovaJacoJ2N6S300position(robot,q0,7);

% options=odeset('RelTol',1e-6,'AbsTol',1e-8*ones(17,1));
options=odeset('RelTol',1e-3,'AbsTol',1e-3*ones(54,1));
% options=odeset('RelTol',1e-5,'AbsTol',1e-5*ones(54,1));

init = [init;reshape(JHat,[18,1]);reshape(JC,[18,1])];

[t,y]=ode45('Robotnet',(0:0.01:T),init,options);

y=y(1:length(t),1:54);
size(t)
size(y)
save (['INITdata'], 't', 'y');
save (['parameter'], 'T', 'neta', 'zeta', 'r', 'omega', 'upperq', 'lowerq', 'upperdq', 'lowerdq', 'beta', 'q0', 'initPos');