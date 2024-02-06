%%
% clear all;
% format long;
load data/INITdata.mat;
load data/parameter.mat;
addpath('func');

%=========== kinovaJacoJ2N6S300  ==============
robot = loadrobot('kinovaJacoJ2N6S300','DataFormat','column','Gravity',[0 0 -9.81]);
%=========== kinovaJacoJ2N6S300 ==============

obstacle = [-0.075,-0.48,0.16]';
obstacle_range_1 = 0.065;
obstacle_range_2 = 0.09;
inteval = 1;
number = fix(length(t)/inteval);
pos = zeros(number,3);
linkPos = zeros(5,3,number);
Error = zeros(number,3);
index = 0;

% desired path
rdx = initPos(1) + r*cos(2*pi*(sin(0.5*pi*t/T)).^2)-r;
rdy = initPos(2) + r*cos(pi/6)*sin(2*pi*(sin(0.5*pi*t/T)).^2);
rdz = initPos(3) + r*sin(pi/6)*sin(2*pi*(sin(0.5*pi*t/T)).^2);

% theta=t*6*pi/T;
% rdx=r*(cos(theta)+cos((2/3)*theta))-2*r+initPos(1);
% rdy=r*(sin(theta)-sin((2/3)*theta))+initPos(2);
% rdz=initPos(3)*ones(number);

% actual trajectory
for i = 1:length(t)
    q = y(i,1:6)';
    index = index + 1;
    %末端
    pos(index,1:3) = kinovaJacoJ2N6S300position(robot,q,7);
    %误差
    Error(index,:) = [pos(index,1)-rdx(i), pos(index,2)-rdy(i), pos(index,3)-rdz(i)];
end

% 轨迹
figure;
plot3(rdx, rdy, rdz, 'b','linewidth', 2);grid on;
hold on;
plot3(pos(1:10:number,1), pos(1:10:number,2),pos(1:10:number,3), 'r:*','linewidth', 1);
hold off;
axis equal
legend('Desired path', 'Actual trajectory', 'Location', 'best', 'FontName', 'times new Roman', 'fontsize', 18);
set(gca,'FontSize', 18);
xlabel('X (m)', 'FontName', 'times new Roman','fontsize',20);
ylabel('Y (m)', 'FontName', 'times new Roman','fontsize',20);
zlabel('Z (m)', 'FontName', 'times new Roman','fontsize',20);

index = 0;
jointPos = zeros(7,3,number);
D = zeros(number,3);
for i = 1:length(t)
    q = y(i,1:6)';
    index = index + 1;
    %末端
    ra = kinovaJacoJ2N6S300position(robot,q,7);
    
    for j = 1:6
        jointPos(j,:,index) = kinovaJacoJ2N6S300position(robot,q,j);
    end
    jointPos(7,:,index) = ra;
    C = zeros(6,3);
    R = zeros(6,3);
    distance = zeros(6,1);
    for i = 1:6
        [temp1,temp2,temp3] = GetCPosition(obstacle,jointPos(i,:,index)',jointPos(i+1,:,index)');
        C(i,:) = temp1;
        R(i,:) = temp2;
        distance(i) = temp3;
    end

    [min_distance,l] = min(distance);
    D(index,:) = min_distance;
end

figure;
plot(t,D(:,2), 'r-','linewidth', 1);
hold on;
plot(t,obstacle_range_1*ones(number,1), 'b-','linewidth', 1);
plot(t,obstacle_range_2*ones(number,1), 'b--','linewidth', 1);
legend('Distance', 'Limit distance', 'Warning distance', 'Location', 'best', 'FontName', 'times new Roman', 'fontsize', 24);
xlabel('Time (s)', 'FontName', 'times new Roman','fontsize',24);
ylabel('Distance (m)', 'FontName', 'times new Roman','fontsize',24);
axis([0,10,0.055,0.13]);
set(gca,'FontSize', 22);


