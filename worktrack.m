clear;
clc;
close all;
%对目标抽象表示
syms L1 L2 L3 theta1 theta2 theta3 nx ny nz ox oy oz ax ay az px py pz
angle = pi/180;%弧度转角度
radian = 180/pi;%角度转弧度
%机身长宽
Length = 225.68;
Width  = 86;
H = 20;
midLength = 242.68;

L1 = 84;
L2 = 73.5;
L3 = 140.8;
%关节角度限制
q1_limit = [-pi/4,pi/4];
q2_limit = [-pi,pi];
q3_limit = [-pi,pi];

%连杆创建
%R = Link([theta,d,a,alpha])%关节角，连杆偏置，连杆长度，连杆转角
L(1) = Link([0,0,0,0],'modified');L(1).qlim = q1_limit;
L(2) = Link([0,0,L1,pi/2,0],'modified');L(2).qlim = q2_limit;
L(3) = Link([0,0,L2,0],'modified');L(3).qlim = q3_limit;L(3).offset = -pi/2;

%使用Seriallink类函数把我们上面使用Link函数建立的连杆连成一个整体，生成一个串联机械臂模型
four_Link = SerialLink([L(1),L(2),L(3)],'name','SixLegRobtor');
four_Link.base = transl(0,0,0);%定义机器人基坐标
four_Link.tool = transl(L3,0,0);
base = four_Link.base;

three_Link = SerialLink([L(1),L(2),L(3)]);
three_Link.base = transl(0,0,0);%定义机器人基坐标
% three_Link.display
% three_Link.teach
two_Link = SerialLink([L(1),L(2)]);
two_Link.base = transl(0,0,0);%定义机器人基坐标


%定义轨迹起始点，中间点，终点，插值点数
p_start = [157.5,60,-140.8];
p_mid = [157.5,0,-81.8];
p_final = [157.5,-60,-140.8];
% p_start = [140.304, 0, -188.045];
% p_mid = [228.609, 85, -26.852];
% p_final = [185.962, 185.962, -26.852];
% %
% syms x1 x2 x3 y1 y2 y3 z1 z2 z3
% p_start = [x1,y1,z1];
% p_mid = [x2,y2,z2];
% p_final = [x3,y3,z3];
step = 9;

%计算所取圆弧在轨迹圆的半径
a = norm(p_final - p_mid);           %内接三角形边长a
b = norm(p_final - p_start);         %内接三角形边长b
c = norm(p_mid - p_start);           %内接三角形边长c
l = (a + b + c) / 2;   %内接三角形半周长
r = a*b*c/ 4 / sqrt(l*(l - a)*(l - b)*(l - c)); %轨迹圆半径
%求所取圆弧所在平面参数
solution = [p_start(1) p_start(2) p_start(3) ; p_mid(1) p_mid(2) p_mid(3) ; p_final(1) p_final(2) p_final(3) ] \ [1;1;1]; 
%A是一个方阵，A\B与inv(A)* B大致相同

%求取圆弧所在轨迹圆圆心

b1 = a*a * (b*b + c*c - a*a);
b2 = b*b * (a*a + c*c - b*b);
b3 = c*c * (a*a + b*b - c*c);
P1 = [p_start'  p_mid'  p_final'];
P2 = [b1; b2; b3];
P3 = P1 * P2;
center = P3 ./ (b1 + b2 + b3);
center = center';%转置
%
vector_start_big = p_start - center;                          %由圆心指向起点的向量
vector_start = (p_start - center) ./ norm(p_start - center);  %由圆心指向起点的单位向量
vector_final = (p_final - center) ./ norm(p_start - center);  %由圆心指向终点的单位向量
rotation_axis = cross(vector_start,vector_final);   %旋转轴

theta = acos(dot(vector_start , vector_final));     %弧度制的圆弧角度
%theta = rad2deg(theta);%角度制的圆弧角度
theta_per = theta / step;%角度制的每个轨迹点之间的角度

theta_current = 0;   %初始化当前路径点与起始点之间的角度
p_current = 0;       %初始化轨迹点位置
%
for t = 1 : step+1 
    
    matrix_current = rotation_matrix(rotation_axis,theta_current);  %旋转矩阵的计算
    vector_current = matrix_current * (vector_start_big');         %使向量绕旋转轴旋转
    p_current = center + vector_current';%轨迹点坐标 
    T_current=transl(p_current);                                   %起始点齐次变换矩阵
    Tc(:,:,t) = T_current;                                      %保存轨迹点为齐次变化矩阵
    theta_current = t * theta_per;                                 %角度变化

end
%分配空间存放支撑相，摆动相的坐标
Swing_Path = zeros(3,step+1);
Spport_Path = zeros(3,step+1);

%将计算结果存放到周期坐标数组中
for i = 1:step+1
    temp(:,:,i) = Tc(1:3,4,i);
    Swing_Path(1,i) = temp(1,1,i);
    Swing_Path(2,i) = temp(2,1,i);
    Swing_Path(3,i) = temp(3,1,i);

    Spport_Path(1,i) = temp(1,1,i);
    Spport_Path(2,i) = -temp(2,1,i);
    Spport_Path(3,i) = temp(3,1,1);
end

%开辟一个周期坐标存放空间
% path = zeros(2*step,3);
path = zeros(step,3);
for j = 1:step+1

    path(j,1)=Swing_Path(1,j);
    path(j,2)=Swing_Path(2,j);
    path(j,3)=Swing_Path(3,j);

    path(j+step+1,1) = Spport_Path(1,j);
    path(j+step+1,2) = Spport_Path(2,j);
    path(j+step+1,3) = Spport_Path(3,j);  
end

base = four_Link.base;
base = base.T;

via_angle = zeros(step,3);
Time = zeros(1,step+1);
real_path = zeros(step,3);

% via_angle = zeros(2*step,3);
% Time = zeros(1,2*step);
psition = path(1,:);
p0_pose = base*transl([psition(1,1),psition(1,2),psition(1,3)]);
p0_angle = four_Link.ikine(p0_pose,'mask',[1,1,1,0,0,0]);
% p0_angle = p0_angle;
for k = 1:step+1
    % if k > 1
    %    real_path(k-1,1) = path(k,1);
    %    real_path(k-1,2) = path(k,2);
    %    real_path(k-1,3) = path(k,3);
    % end
    real_path(k,1) = path(k,1);
    real_path(k,2) = path(k,2);
    real_path(k,3) = path(k,3);
end

% for t = 1:step
%     Time(1,k) = 0.1;
% end
% p0 = four_Link.ikine(psition,'mask',[1,1,1,0,0,0]);
for k = 1:step+1                                                 
    target_pose = base*transl([real_path(k,1),real_path(k,2),real_path(k,3)]);
    temp_angle = four_Link.ikine(target_pose,'mask',[1,1,1,0,0,0]);
    via_angle(k,1:3) = temp_angle;
end
tseg = ones(1, 10) * 0.1; % 每个段落的时间都是0.1秒
% for k = 1:step+1
%     Time(1,k) = 0.1; 
%     target_pose = base*transl([path(k,1),path(k,2),path(k,3)]);
%     temp_angle = four_Link.ikine(target_pose,'mask',[1,1,1,0,0,0]);
%     via_angle(k,1:3) = temp_angle;
% end

% [traj,vel,acc] = mstraj(via_angle,[],Time,temp_angle1,0.1,0.01);

[traj,vel,acc] = mstraj(via_angle,[],tseg,[0,0,0],0.1,0.1);

figure;
% subplot(3,2,1);
hold on; % 保持在同一张图上绘制
colors = lines(size(traj, 2)); % 为每个关节生成颜色
% for i = 1:size(traj, 2) % 遍历每个关节
%     plot(t(1:end-1), traj(:, i), 'Color', colors(i, :)); % 绘制第i个关节的速度
% end

dt = 0.1;
t = (0:(size(traj, 1) - 1)) * dt;
for i = 1:size(traj,2)
    plot(t(1:end-1),traj(1:10, i), 'Color', colors(i, :),'LineWidth',5)
end
hold off;
Htitle1 = title('Joint Angle vs. Time');
Htitle1.FontSize = 31;rrrrrrr
xlable1 = xlabel('Time (s)');
xlable1.FontSize = 21;
ylable1 = ylabel('angle (rad)');
ylable1.FontSize = 21;
leg1 = legend(arrayfun(@(n) sprintf('Joint %d', n), 1:size(traj, 2), 'UniformOutput', false));
leg1.FontSize = 20;

% figure;
% plot(traj);
% size(traj)
% 绘制每个关节的角度随时间变化的图像
% colors = lines(size(traj,3));
% for i = 1:size(traj,3)
%     plot()
% end

% for i = 1:size(traj, 2) % 遍历每个关节
%     figure; % 创建新的图形窗口
%     plot(traj(:, i)); % 绘制第i个关节的角度随时间变化
%     title(sprintf('Joint %d Angle vs. Time', i)); % 设置图像标题
%     xlabel('Time step'); % 设置横轴标签
%     ylabel('Joint angle (rad)'); % 设置纵轴标签
%     legend(sprintf('Joint %d', i)); % 设置图例
% end

%绘制每个关节速度
dt = 0.1;
vel = diff(traj)/dt;
% vel = [0;vel]
t = (0:(size(vel, 1))) * dt;

% 绘制每个关节的速度
figure;
% subplot(3,2,3);
hold on; % 保持在同一张图上绘制
colors = lines(size(vel, 2)); % 为每个关节生成颜色
for i = 1:size(vel, 2) % 遍历每个关节
    plot(t(1:end-2), vel(1:9, i), 'Color', colors(i, :),'LineWidth',5); % 绘制第i个关节的速度
end
hold off;
Htitle2 = title('Joint Velocities vs. Time');
Htitle2.FontSize = 31;
xlabel2 = xlabel('Time (s)');
xlable2.FontSize = 21;
ylable2 = ylabel('Joint velocity (rad/s)');
ylable2.FontSize = 21;
leg2 = legend(arrayfun(@(n) sprintf('Joint %d', n), 1:size(vel, 2), 'UniformOutput', false));
leg2.FontSize = 20;

% 假设 ddt 是采样时间间隔
% ddt = 0.1; % 例如，每0.05秒采样一次
% 计算加速度
acc = diff(vel) / dt;

% 由于再次使用 diff 函数，返回的矩阵比速度矩阵少一行，因此再次调整时间向量
t = (0:(size(acc, 1)- 1)) * dt;
    
% 绘制每个关节的加速度
figure;
% subplot(3,2,5);
hold on; % 保持在同一张图上绘制
colors = lines(size(acc, 2)); % 为每个关节生成颜色
for i = 1:size(acc, 2) % 遍历每个关节
    plot(t(1:end-1), acc(1:8, i), 'Color', colors(i, :),'LineWidth',5); % 绘制第i个关节的加速度
end
hold off;
Htitle3 = title('Joint Accelerations vs. Time');
Htitle3.FontSize = 31;
xlabel3 = xlabel('Time (s)');
xlabel3.FontSize = 21;
ylabel3=ylabel('Joint acceleration (rad/s^2)');
ylabel3.FontSize = 21;
leg3 = legend(arrayfun(@(n) sprintf('Joint %d', n), 1:size(acc, 2), 'UniformOutput', false));
leg3.FontSize = 20;
figure;
% subplot(3,2,[2,4,6])
four_Link.plot(traj);
% position_joint_2 = zeros()
position_x_1 = zeros(1,10);
position_y_1 = zeros(1,10);
position_z_1 = zeros(1,10);

position_x_2 = zeros(1,10);
position_y_2 = zeros(1,10);
position_z_2 = zeros(1,10);

position_x_3 = zeros(1,10);
position_y_3 = zeros(1,10);
position_z_3 = zeros(1,10);

position_joint_1 = zeros(10,3);
position_joint_2 = zeros(10,3);
position_joint_3 = zeros(10,3);
for i = 1:10
    theta1 = via_angle(i,1:2);
    position_matrix_1 = two_Link.fkine(theta1);
    position_matrix_1 = position_matrix_1.T;
    for j = 1:3
        position_joint_1(i,j) = position_matrix_1(j,4);
        position_x_1(1,i) = position_joint_1(i,1);
        position_y_1(1,i) = position_joint_1(i,2);
        position_z_1(1,i) = position_joint_1(i,3);
    end
    
end

for i = 1:10
    theta2 = via_angle(i,1:3);
    
    position_matrix_2 = three_Link.fkine(theta2);
    position_matrix_2 = position_matrix_2.T;
    for j = 1:3
        position_joint_2(i,j) = position_matrix_2(j,4);
        position_x_2(1,i) = position_joint_2(i,1);
        position_y_2(1,i) = position_joint_2(i,2);
        position_z_2(1,i) = position_joint_2(i,3);
    end
    
end
for i = 1:10
    theta3 = via_angle(i,1:3);
    
    position_matrix_3 = four_Link.fkine(theta3);
    position_matrix_3 = position_matrix_3.T;
    for j = 1:3
        position_joint_3(i,j) = position_matrix_3(j,4);
        position_x_3(1,i) = position_joint_3(i,1);
        position_y_3(1,i) = position_joint_3(i,2);
        position_z_3(1,i) = position_joint_3(i,3);
    end
    
end
figure
plot3(position_x_1,position_y_1,position_z_1,'LineWidth',5)
hold on
plot3(position_x_2,position_y_2,position_z_2,'LineWidth',5)
hold on
plot3(position_x_3,position_y_3,position_z_3,'LineWidth',5)
% legend("Joint1","Joint_2","Joint3")
% 连接每组点的对应点
for i = 1:length(position_x_1)
    plot3([position_x_1(i), position_x_2(i)], ...
         [position_y_1(i), position_y_2(i)], ...
         [position_z_1(i), position_z_2(i)], 'k--','Color','b','LineWidth',5); % 连接第一组和第二组的对应点
    plot3([position_x_2(i), position_x_3(i)], ...
         [position_y_2(i), position_y_3(i)], ...
         [position_z_2(i), position_z_3(i)], 'k--','Color',[1, 0.5, 0],'LineWidth',5); % 连接第二组和第三组的对应点
end
leg4 = legend("Joint1","Joint2","Joint3","Link1","Link2")
leg4.FontSize = 21;
HTitle = title("机器人关节运动三维图");
HTitle.FontSize = 51;
grid on;
hold off; % 释放图形

% figure;
% plot(traj);
% figure;
% plot(vel);
% figure;
% plot(acc);
% figure
% plot3(position_x,position_y,position_z,'.');
%%
% % 计算速度
% y_dot = diff(y) / (1/step);
% z_dot = diff(z) / (1/step);
% 
% % 计算加速度
% figure
% y_ddot = diff(y_dot) / (1/step);
% z_ddot = diff(z_dot) / (1/step);
% figure;
% subplot(2,2,1);
% plot(y);
% title('Y Position');
% xlabel('Time Step');
% ylabel('Position (mm)');

% subplot(2,2,2);
% plot(y_dot);
% title('Y Velocity');
% xlabel('Time Step');
% ylabel('Velocity (mm/s)');
% 
% subplot(2,2,3);
% plot(z);
% title('Z Position');
% xlabel('Time Step');
% ylabel('Position (mm)');
% 
% subplot(2,2,4);
% plot(z_dot);
% title('Z Velocity');
% xlabel('Time Step');
% ylabel('Velocity (mm/s)');
% 
% figure;
% subplot(2,1,1);
% plot(y_ddot);
% title('Y Acceleration');
% xlabel('Time Step');
% ylabel('Acceleration (mm/s^2)');
% 
% subplot(2,1,2);
% plot(z_ddot);
% title('Z Acceleration');
% xlabel('Time Step');
% ylabel('Acceleration (mm/s^2)');
% % 绘制轨迹
% figure;
% plot3(squeeze(Tc(1,4,:)), squeeze(Tc(2,4,:)), squeeze(Tc(3,4,:)), '-r', 'LineWidth', 2);
% grid on
% xlabel('X'), ylabel('Y'), zlabel('Z')
% title('Manipulator Trajectory')
% hold on
% 
% % 迭代绘制机械臂在轨迹上的运动
% figure;
% for t = 1:size(Tc,3)
%     four_Link.teach(four_Link.ikine(Tc(:,:,t),'mask',[1 1 1 0 0 0]));
%     drawnow
% end
% hold on;
% 
% %创建机器人
% Leg(1) = SerialLink(four_Link,'name','leg1','base',transl(-Width/2,Length/2,0)*trotz(135));
% Leg(2) = SerialLink(four_Link,'name','leg2','base',transl(-midLength/2,0,0)*trotz(180));
% Leg(3) = SerialLink(four_Link,'name','leg3','base',transl(-Width/2,-Length/2,0)*trotz(225));
% Leg(4) = SerialLink(four_Link,'name','leg4','base',transl(Width/2,Length/2,0)*trotz(45));
% Leg(5) = SerialLink(four_Link,'name','leg5','base',transl(midLength/2,0,0));
% Leg(6) = SerialLink(four_Link,'name','leg6','base',transl(Width/2,-Length/2,0)*trotz(-45));
% %%
% 
% A = ((x3.^2-x2.^2)+(y3.^2-y2.^2)+(z3.^2-z2.^2))*((x3.^2-x1.^2)+(y3.^2-y1.^2)+(z3.^2-z1.^2)+(x3.^2-x2.^2)+(y3.^2-y2.^2)+(z3.^2-z2.^2))
% expand(A)