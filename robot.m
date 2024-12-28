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
%R = Link([theta,d,a,alpha])%关节角，连杆偏置，连杆长度，连杆转角
L1 = 84;
L2 = 73.5;
L3 = 140.8;
q1_limit = [-pi,pi];
q2_limit = [-pi,pi];
q3_limit = [-pi,pi];

%连杆创建
L(1) = Link([0,0,0,0],'modified');L(1).qlim = q1_limit;
L(2) = Link([0,0,L1,pi/2,0],'modified');L(2).qlim = q2_limit;
L(3) = Link([0,0,L2,0],'modified');L(3).qlim = q3_limit;L(3).offset = -pi/2;
L(4) = Link([0,0,L3,0],'modified');

%使用Seriallink类函数把我们上面使用Link函数建立的连杆连成一个整体，生成一个串联机械臂模型
four_Link = SerialLink([L(1),L(2),L(3), L(4)],'name','SixLegRobtor');


%创建机器人
Leg(1) = SerialLink(four_Link,'name','leg1','base',transl(-Width/2,Length/2,0)*trotz(135));
Leg(2) = SerialLink(four_Link,'name','leg2','base',transl(-midLength/2,0,0)*trotz(180));
Leg(3) = SerialLink(four_Link,'name','leg3','base',transl(-Width/2,-Length/2,0)*trotz(225));
Leg(4) = SerialLink(four_Link,'name','leg4','base',transl(Width/2,Length/2,0)*trotz(45));
Leg(5) = SerialLink(four_Link,'name','leg5','base',transl(midLength/2,0,0));
Leg(6) = SerialLink(four_Link,'name','leg6','base',transl(Width/2,-Length/2,0)*trotz(-45));

view(3);
p_start = [157.5,-60,-140.8];
p_mid = [157.5,0,-90.8];
p_final = [157.5,60,-140.8];
Legth = 60;
step =10;
hold on
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

vector_start_big = p_start - center;                          %由圆心指向起点的向量
vector_start = (p_start - center) ./ norm(p_start - center);  %由圆心指向起点的单位向量
vector_final = (p_final - center) ./ norm(p_start - center);  %由圆心指向终点的单位向量
rotation_axis = cross(vector_start,vector_final);   %旋转轴

theta = acos(dot(vector_start , vector_final));     %弧度制的圆弧角度
theta_per = theta / step;%角度制的每个轨迹点之间的角度

theta_current = 0;   %初始化当前路径点与起始点之间的角度
p_current = 0;       %初始化轨迹点位置
initial_angles = zeros(1,24);
for i=1:6
    initial_angles((i-1)*4+1:i*4) = [0,0,0,0];
end
%绘制初始位置
for i = 1:6
    Leg(i).plot(initial_angles((i-1)*4+1:i*4));
end
%预分配内存
Swing_x = zeros(1,step);
Swing_y = zeros(1,step);
Swing_z = zeros(1,step);

Support_x = zeros(1,step);
Support_y = zeros(1,step);
Support_z = zeros(1,step);
%存放支撑相、摆动相位置数据
for i = 1 : step
    matrix_current = rotation_matrix(rotation_axis,theta_current);  %旋转矩阵的计算
    vector_current = matrix_current * (vector_start_big');         %使向量绕旋转轴旋转
    p_current = center + vector_current';%轨迹点坐标 
    T_current=transl(p_current);                                   %起始点齐次变换矩阵
    Tc(:,:,i) = T_current;                                      %保存轨迹点为齐次变化矩阵
    theta_current = i * theta_per;                                 %角度变化
    temp(:,:,i) = Tc(1:3,4,i);

    Swing_x(1,i) = temp(1,1,i);
    Swing_y(1,i) = temp(2,1,i);
    Swing_z(1,i) = temp(3,1,i);

    Support_x(1,i) = temp(1,1,i);
    Support_y(1,i) = -temp(2,1,i);
    Support_z(1,i) = temp(3,1,1);
end
Sw_via_angle = zeros(10,3);
Sp_via_angle = zeros(10,3);

% 初始化视频文件
VideoFile = 'robot_gait_cycle.mp4';
VideoWriter = VideoWriter(VideoFile);
VideoWriter.FrameRate = 10; % 设置帧率
open(VideoWriter);
%机器人步态周期仿真
while true
for flag = 1:2%设置支撑、摆动占空比为50%
    for step = 1:10
        Swing_Position   = [Swing_x(1,step),Swing_y(1,step),Swing_z(1,step)];
        Support_Position = [Support_x(1,step),Support_y(1,step),Support_z(1,step)];
        base = four_Link.base;
        base = base.T;

        Sw_via_point = base*transl(Swing_Position(1), Swing_Position(2), Swing_Position(3));
        temp_angle = four_Link.ikine(Sw_via_point,'mask',[1,1,1,0,0,0]);
        Sw_via_angle(step,1) = temp_angle(1,1);
        Sw_via_angle(step,2) = temp_angle(1,2);
        Sw_via_angle(step,3) = temp_angle(1,3);
        
        Sp_via_point = base*transl(Support_Position(1), Support_Position(2), Support_Position(3));
        temp_angle = four_Link.ikine(Sp_via_point,'mask',[1,1,1,0,0,0]);
        Sp_via_angle(step,1) = temp_angle(1,1);
        Sp_via_angle(step,2) = temp_angle(1,2);
        Sp_via_angle(step,3) = temp_angle(1,3);


        % via_point = transl(Swing_Position(1), Swing_Position(2), Swing_Position(3));
        angle = zeros(6,4);
        for leg = 1:6
            base_transform = Leg(leg).base;
            base_transform = base_transform.T;
            if flag == 1
               if mod(leg,2) == 1
                  if leg == 1||leg == 3
                     target_position = base_transform * transl(Swing_Position(1), -Swing_Position(2), Swing_Position(3));
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  else 
                     target_position = base_transform * transl(Swing_Position(1), Swing_Position(2), Swing_Position(3));
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  end
               end
               if mod(leg,2) == 0
                  if leg == 2
                     target_position = base_transform * transl(Support_Position(1), -Support_Position(2), Support_Position(3)); 
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  else 
                     target_position = base_transform * transl(Support_Position(1),Support_Position(2), Support_Position(3)); 
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  end
               end
            end
            %后半周期
             if flag == 2
               if mod(leg,2) == 1
                  if leg == 1||leg == 3
                     target_position = base_transform * transl(Support_Position(1), -Support_Position(2), Support_Position(3));
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  else 
                     target_position = base_transform * transl(Support_Position(1), Support_Position(2), Support_Position(3));
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  end
               end
               if mod(leg,2) == 0
                  if leg == 2
                     target_position = base_transform * transl(Swing_Position(1), -Swing_Position(2), Swing_Position(3));
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  else 
                     target_position = base_transform * transl(Swing_Position(1),Swing_Position(2), Swing_Position(3));
                     plot3(target_position(1,4),target_position(2,4),target_position(3,4),".r",'LineWidth',5);
                  end
               end
             end

            q = Leg(leg).ikine(target_position, 'mask', [1, 1, 1, 0, 0, 0]);%逆解出各个腿关节角度
            if ~isempty(q)
                % 更新关节角度
                angle(leg,:) = q;
            end
        end
        %写法一，不经过for循环使仿真看起来更灵活，可以做到腿部同步运动，缺点：代码多，不美观
        Leg(1).plot(angle(1,:));
        Leg(2).plot(angle(2,:));
        Leg(3).plot(angle(3,:));
        Leg(4).plot(angle(4,:));
        Leg(5).plot(angle(5,:));
        Leg(6).plot(angle(6,:));
        frame = getframe(gcf);
        % 将捕获的帧写入视频文件
        writeVideo(VideoWriter, frame);
        pause(0.01); % 控制移动速度
        %写法二，经过for循环仿真看起来机械，不能做到腿部同步运动，缺点：运动卡顿，推荐运用写法一
        % for leg=1:6
        % Leg(leg).plot(angle(leg,:));
        end

        % pause(0.01); % 控制移动速度
    end
close(VideoWriter);
end

% [qTrag1,qdTrag1,qddTrag1] = mstraj(Sw_via_angle,[],[1:2],[],0.01,1)
% end

%%theothers
% T01 = [cos(theta1),-sin(theta1),0,0;
%        sin(theta1),cos(theta1),0 ,0;
%        0          ,     0     ,1 ,0;
%        0          ,     0     ,0 ,1];
% 
% T12 = [cos(theta2)  ,-sin(theta2),0,L1;
%            0        ,      0     ,-1,0;
%        sin(theta2),cos(theta2) ,0,0;
%        0          ,     0     ,0 ,1];
% 
% T23 = [cos(theta3),-sin(theta3),0,L2;
%        sin(theta3),cos(theta3),0 ,0;
%        0          ,     0     ,1 ,0;
%        0          ,     0     ,0 ,1];
% 
% T34 = [1,0,0,L3;
%        0,1,0,0;
%        0,0,1,0;
%        0,0,0,1];
% TT = [nx ox ax px;
%       ny oy ay py;
%       nz oz az pz;
%       0  0   0  1];
% 
% T = T01*T12*T23*T34;  
% simplify(T);                                                           
% 
% hold on
% four_Link = SerialLink([L(1),L(2),L(3),L(4)],'name','SixLegRobtor');
% four_Link.base = transl(0,0,0);%定义机器人基坐标
% % four_Link.tool = transl(L3,0,0);
% % four_Link.plot([0 0 0]);
% %显示机器人模型
% 
% % four_Link.plot([0,0,0,0],'nobase','noshadow');
% %使用display显示出我们建立的这个机械臂模型的信息
% four_Link.display
% %示教模式，驱动机器人运动
% four_Link.teach

%
%验证运动学逆解

% %运动学逆解解析解
% f1 = sqrt(x^2+y^2);
% Lr = sqrt((f1-L1)^2 +z^2);
% alpha_r = atan2d(z,f1-L1);
% %
% %求得该位置下各个关节角度333
% alpha1 = atan2d(y,x);
% alpha_2 = acosd((L2^2-L3^2+Lr^2)/(2*Lr*L2)) - atan2d(z,L1-f1)
% alpha_3 = acosd((Lr^2-L2^2-L3^2)/(2*L2*L3));
% alpha3  = acosd((Lr^2 - L2^2 - L3^2) / (2 * L2 * L3));
% %路径规划（末端位置为关节1旋转pi/12,步长：,运动周期T=6s,摆动相周期T1=T/2,支撑相周期T1=T/2）
% 
% %腿部基座标相对于身体中心点位姿
% Leg1_frame = [cos(3*pi/4), -sin(3*pi/4), 0, -Width/2; sin(3*pi/4), cos(3*pi/4), 0, Length/2; 0, 0, 1, 0; 0, 0, 0, 1]
% Leg2_frame = [cos(pi), -sin(pi), 0, -midLength/2; sin(pi), cos(pi), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]
% Leg3_frame = [cos(5*pi/4), -sin(5*pi/4), 0, -Width/2; sin(5*pi/4), cos(5*pi/4), 0, -Length/2; 0, 0, 1, 0; 0, 0, 0, 1]
% Leg4_frame = [cos(pi/4), -sin(pi/4), 0, Width/2; sin(pi/4), cos(pi/4), 0, Length/2; 0, 0, 1, 0; 0, 0, 0, 1]
% Leg5_frame = [1, 0, 0, midLength/2; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]
% Leg6_frame = [cos(-pi/4), -sin(-pi/4), 0, Width/2; sin(-pi/4), cos(-pi/4), 0, -Length/2; 0, 0, 1, 0; 0, 0, 0, 1]
% %末端执行器相对于身体中心点位姿
% Leg1_T05 = Leg1_frame*T;
% Leg2_T05 = Leg2_frame*T;
% Leg3_T05 = Leg3_frame*T;
% Leg4_T05 = Leg4_frame*T;
% Leg5_T05 = Leg5_frame*T;
% Leg6_T05 = Leg6_frame*T;
% simplify(Leg1_T05)
% simplify(Leg2_T05);
% simplify(Leg3_T05);
% simplify(Leg4_T05);
% simplify(Leg5_T05);
% simplify(Leg6_T05);
% %初始位置相对于身体位姿
% S_position1 = [-Width/2-(L1+L2)*cos(pi/4),Length/2+(L1+L2)*sin(pi/4),-L3,1];
% S_position2 = [-midLength/2-(L1+L2),0,-L3,1]
% S_position3 = [-Width/2-(L1+L2)*cos(pi/4),-(Length/2+(L1+L2)*sin(pi/4)), -L3,1];
% S_position4 = [Width/2+(L1+L2)*cos(pi/4),Length/2+(L1+L2)*sin(pi/4),-L3,1];
% S_position5 = [midLength/2+(L1+L2),0,-L3,1];
% S_position6 = [Width/2+(L1+L2)*cos(pi/4),-(Length/2+(L1+L2)*sin(pi/4)),-L3,1];
% 
% %终点位置相对于身体位姿
% E_position1 = [((-Width)/2+(L1+L2)*cos(pi/3));Length/2+(L1+L2)*sin(pi/3);-L3;1];
% E_position2 = [(-midLength)/2-(L1+L2)*cos(pi/12),(L1+L2)*cos(pi/12),-L3,1];
% E_position3 = [(-Width)/2+(L1+L2)*cos(pi/3),((-Length)/2+(L1+L2)*sin(pi/3)), -L3,1];
% E_position4 = [Width/2+(L1+L2)*cos(pi/3),Length/2+(L1+L2)*sin(pi/3),-L3,1];
% E_position5 = [midLength/2+(L1+L2)*cos(pi/12),(L1+L2)*cos(pi/12),-L3,1];
% E_position6 = [Width/2+(L1+L2)*cos(pi/3),-(Length/2+(L1+L2)*sin(pi/3)), -L3,1];
% 
% 
% %末端位姿相对于腿部基座坐标

%给定关节角度路径

% T2 = four_Link.fkine([-pi/4, 0,0])
% T1 = four_Link.fkine([0, 0, 0])
% T2 = four_Link.fkine([0, pi/3,0])
% T3 = four_Link.fkine([0, pi/8, pi/8])
% T1 = four_Link.fkine([pi/3, 0, 0]);
% T2 = four_Link.fkine([-pi/3, 0,0]);
% T3 = four_Link.fkine([0, 0, 0]);
% T1 = T1.T;
% T2 = T2.T;
% T3 = T3.T;

% T6 = four_Link.fkine([rad1,rad2,rad3]);
% q6 = ikine(four_Link,T6,'mask',[1 1 1 0 0 0]);

% q0 = ikine(four_Link,T1,'mask',[1 1 1 0 0 0]);
% q1 = ikine(four_Link,T2,'mask',[1 1 1 0 0 0]);
% q2 = ikine(four_Link,T3,'mask',[1 1 1 0 0 0]);


% %基于中间点矩阵生成一个多段多轴轨迹，参数是：
% %中间点矩阵，每轴的最大速度向量，每段的运动时间向量，起点各轴坐标，采样时间间隔，加速时间
% via = [q0;q1;q2;q0]; 
% 
% path = mstraj(via,[],[0,0.5,0.35,0.15],via(1,:),0.01,0);
% 
% path_1 = four_Link.fkine(path);
% 
% qcycle = ikine(four_Link,path_1,'mask',[1 1 1 0 0 0]);
% 
% 
% plot3(squeeze(qcycle(:,1,:)),squeeze(qcycle(:,2,:)),squeeze(qcycle(:,3,:)));
% % four_Link.plot(qcycle,'trail','b.','loop','nobase','noshadow');
% figure(2);




