clear;
format long;
% load data_original/INITdata.mat;
% load data_original/parameter.mat;
load INITdata.mat;
load parameter.mat;
addpath('func');

global obstacle
obstacle = [-0.075,-0.48,0.16]';
robot = loadrobot('kinovaJacoJ2N6S300','DataFormat','column','Gravity',[0 0 -9.81]);

inteval = 1;
number = length(t);
pos = zeros(number,3);
jointPos = zeros(6,3,number);
Error = zeros(number,3);
index = 0;


% desired path
rdx = initPos(1) + r*cos(2*pi*(sin(0.5*pi*t/T)).^2)-r;
rdy = initPos(2) + r*cos(pi/6)*sin(2*pi*(sin(0.5*pi*t/T)).^2);
rdz = initPos(3) + r*sin(pi/6)*sin(2*pi*(sin(0.5*pi*t/T)).^2);
[steps, ~] = size(t);
% Actual trajectory
for i = 1:length(t)
    q = y(i,1:6)';
    qs = zeros(steps,9);
    qs(i,1:6) = y(i,1:6);
    
    jointPos(1,:,i) = tform2trvec(getTransform(robot,qs(i,:)','j2n6s300_link_1'));
    jointPos(2,:,i) = tform2trvec(getTransform(robot,qs(i,:)','j2n6s300_link_2'));
    jointPos(3,:,i) = tform2trvec(getTransform(robot,qs(i,:)','j2n6s300_link_3'));
    jointPos(4,:,i) = tform2trvec(getTransform(robot,qs(i,:)','j2n6s300_link_4'));
    jointPos(5,:,i) = tform2trvec(getTransform(robot,qs(i,:)','j2n6s300_link_5'));
    jointPos(6,:,i) = tform2trvec(getTransform(robot,qs(i,:)','j2n6s300_link_6'));
    pos(i, 1:3) = tform2trvec(getTransform(robot,qs(i,:)','j2n6s300_end_effector'));
    
    Error(i,:) = [pos(i,1)-rdx(i), pos(i,2)-rdy(i), pos(i,3)-rdz(i)];
end


figure;

pd = plot3(pos(:,1),pos(:,2),pos(:,3), 'LineWidth',2);hold on;
for j=1:5:length(t)
    basejoint1=line('xdata',[jointPos(1,1,j);jointPos(2,1,j)],'ydata',[jointPos(1,2,j);jointPos(2,2,j)],'zdata',[jointPos(1,3,j);jointPos(2,3,j)],'color', 'yellow');
    joints12=line('xdata',[jointPos(2,1,j);jointPos(3,1,j)],'ydata',[jointPos(2,2,j);jointPos(3,2,j)],'zdata',[jointPos(2,3,j);jointPos(3,3,j)],'color', 'red');
    joints23=line('xdata',[jointPos(3,1,j);jointPos(4,1,j)],'ydata',[jointPos(3,2,j);jointPos(4,2,j)],'zdata',[jointPos(3,3,j);jointPos(4,3,j)],'color', 'cyan');
    joints34=line('xdata',[jointPos(4,1,j);jointPos(5,1,j)],'ydata',[jointPos(4,2,j);jointPos(5,2,j)],'zdata',[jointPos(4,3,j);jointPos(5,3,j)],'color', 'magenta');
    joints45=line('xdata',[jointPos(5,1,j);jointPos(6,1,j)],'ydata',[jointPos(5,2,j);jointPos(6,2,j)],'zdata',[jointPos(5,3,j);jointPos(6,3,j)],'color', 'blue');
    joints56=line('xdata',[jointPos(6,1,j);pos(j,1)],'ydata',[jointPos(6,2,j);pos(j,2)],'zdata',[jointPos(6,3,j);pos(j,3)],'color', 'green');
%     drawnow
end

grid on;
%scatter3(obstacle(1),obstacle(2),obstacle(3),'filled');
[x1,y1,z1]=sphere(30);
x1 = obstacle(1) + 0.09 * x1;
y1 = obstacle(2) + 0.09 * y1;
z1 = obstacle(3) + 0.09 * z1;
surf(x1,y1,z1);
axis equal;
alpha(0.1);
shading interp;

[x2,y2,z2]=sphere(30);
x2 = obstacle(1) + 0.065 * x2;
y2 = obstacle(2) + 0.065 * y2;
z2 = obstacle(3) + 0.065 * z2;
surf(x2,y2,z2);
axis equal;
alpha(0.4);
% shading interp;

hold off;
legend(pd,'Target trajectory', 'best', 'FontName', 'times new Roman', 'fontsize', 24);
set(gca,'FontSize', 24,'FontName','times new Roman');
xlabel('X (m)', 'FontName', 'times new Roman','fontsize',24);
ylabel('Y (m)', 'FontName', 'times new Roman','fontsize',24);
zlabel('Z (m)', 'FontName', 'times new Roman','fontsize',24);