clear;
clc;
close all;
%对目标抽象表示
syms L1 L2 L3 theta1 theta2 theta3 nx ny nz ox oy oz ax ay az px py pz
angle = pi/180;%弧度转角度
radian = 180/pi;%角度转弧度

%R = Link([theta,d,a,alpha])%关节角，连杆偏置，连杆长度，连杆转角
L1 = 84;
L2 = 73.5;
L3 = 140.8;
q1_limit = [-pi/4,pi/4];
q2_limit = [-pi/2,pi/2];
q3_limit = [-pi/2,pi/2];

%连杆创建
L(1) = Link([0,0,0,0],'modified');L(1).qlim = q1_limit;
L(2) = Link([0,0,L1,pi/2.0],'modified');L(2).qlim = q2_limit;
L(3) = Link([0,0,L2,0],'modified');L(3).qlim = q3_limit;L(3).offset = -pi/2;
% L(4) = Link([0,0,L3,0],'modified');

%使用Seriallink类函数把我们上面使用Link函数建立的连杆连成一个整体，生成一个串联机械臂模型
four_Link = SerialLink([L(1),L(2),L(3)],'name','SixLegRobtor');
% four_Link = SerialLink([L(1),L(2),L(3),L(4)],'name','SixLegRobtor');
four_Link.base = transl(0,0,0);%定义机器人基坐标
four_Link.tool = transl(L3,0,0);
% four_Link.plot([0 0 0]);
%显示机器人模型

% four_Link.plot([0,0,0,0],'nobase','noshadow');
%使用display显示出我们建立的这个机械臂模型的信息
four_Link.display
% 示教模式，驱动机器人运动
% four_Link.teach
%蒙特卡洛法求工作空间
step = 1000;%随机步长

theta1 = q1_limit(1) + diff(q1_limit)*rand(step,1);
theta2 = q2_limit(1) + diff(q2_limit)*rand(step,1);
theta3 = q3_limit(1) + diff(q3_limit)*rand(step,1);

pos = {1,step};
for i=1:step
    pos{i} = four_Link.fkine([theta1(i),theta2(i),theta3(i)]);
end

figure 
four_Link.plot([0 0 0],'jointdiam',1)
axis equal;
hold on;
figure;
for i = 1:step
    plot3(pos{i}.t(1),pos{i}.t(2),pos{i}.t(3),'r.');
    hold on;
end
xlabel('X'), ylabel('Y'), zlabel('Z')
title('Reachable workspace')
hold on
grid off



