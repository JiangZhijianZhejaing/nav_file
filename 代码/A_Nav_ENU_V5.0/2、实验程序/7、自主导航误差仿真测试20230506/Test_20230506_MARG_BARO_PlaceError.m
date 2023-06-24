%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：验证ESKF气压计约束
%
% Notice：1、姿态误差0.1°
%         2、高度误差0.1m
%         3、空速误差10%
%
% conclusion：
%
% By Taoran Zhao
% 2023/05/04
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、导入数据---------------
clc;                                                        % 清理命令行
clear;                                                      % 清理工作区
addpath(genpath('../../'));                                 % 导入系统文件夹所有文件
CalibParm_Z_No1_20230505;                                     % 加载Z号模块标定参数
EarthGvar;                                                  % 加载全局变量
% ---------------二、标定数据---------------
[Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
    MargCalib('0505', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
Marg_Number = 6000;
% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargENU(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);
Acc_Set = zeros(Marg_Number,3);
for k = 1:Marg_Number
    a = 0.002;
    Acc_Set(k,:) = [0;0;1]' - [a;a;a]' + [2*a;2*a;2*a]'.*rand(1,3);
end
% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
for k = 1:Marg_Number
    a = 0.1;
    Eul_AccMag_Set(k,3) = 0 - a + 2*a*rand();
end
High_ksub1 = 0;
for k = 1:Marg_Number
    High_Alti_Set(k,:) = 0 - 1 + 2*rand();
    V_Alti_Set(k,:) = (High_Alti_Set(k,:) - High_ksub1)/0.01;
    High_ksub1 = High_Alti_Set(k,:);
end

% ---------------五、ESKF初始化---------------
Sample_Interval = 0.01;                                      % Gyro采样间隔
% ----------1、初始avp----------
% -----(1)姿态-----
Eul_Init = [0;0;0];                                          % 初始姿态欧拉角均0
Qua_Init = Eul2Qnb(Eul_Init);                                % 初始姿态四元数 
% -----(2)速度-----
V_Init = [0;50;0];                                           % 初始速度
% -----(3)位置-----
Place_Init = [0; 0; 0];                                      % 初始当地直角坐标系位置
Pos_Init = [31.90209*pi/180; 117.17*pi/180; 0];             % 初始经纬坐标(由GNSS获得)：合肥 [纬度 经度 高度]
% ----------2、提前分配内存----------
Eul_Set = zeros(Marg_Number,3);                 
Qua_Set = zeros(Marg_Number,4);               
V_Set = zeros(Marg_Number,3);
Pos_Set = zeros(Marg_Number,3);
Place_Set = zeros(Marg_Number,3);
% ----------3、kalman初始化----------
Qua_ksub1 = Qua_Init; 
V_ksub1 = V_Init;
Pos_ksub1 = Pos_Init;
Place_ksub1 = Place_Init;   
X_ksub1 = zeros(9,1);                             % 初始状态(姿态误差、速度误差、位置误差、加速度随机飘移 3*4 = 12)             
P_ksub1 = eye(9);
% Q_ksub1 = (1e-9)*eye(12);                        % 状态噪声方差阵
Q_ksub1 = diag([1e-8,1e-8,1e-8,1e-9,1e-9,1e-9,1e-9,1e-9,1e-9]);       
% R_ksub1 = (1e-1)*eye(5);                         % 加速度计量测噪声方差阵
R_ksub1 = diag([1e-5,1e-5,1e-5,1e-1,1e-4]);       
H_k = [1, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 1, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 1, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 1, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0, 0, 1;];
gn = CalcGn(Pos_Init);                           % 解算当地重力加速度
% ---------------三、ESKF--------------
for k = 1:Marg_Number
    % ----------1、姿态更新----------
    W_Deg_ksub1 = Gyro_Set(k,:);
    % 一阶龙格库塔法
    Qua_k_IMU = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qua_ksub1);   % k时刻 未校正的Qua 一阶龙格库塔法
    Eul_k_IMU = Qnb2Eul(Qua_k_IMU);                                      % k时刻 未校正的姿态欧拉角
    % ----------2、速度更新----------
    Cnb_ksub1 = Qnb2Cnb(Qua_k_IMU);                                      % k-1时刻 Cnb
    fb_ksub1 = -gn(3)*Acc_Set(k,:)';                                     % k-1时刻 fb
    fn_ksub1 = Cnb_ksub1*fb_ksub1;                                       % k-1时刻 fn
    Motion_Acceleration_ksub1 = fn_ksub1 + gn;                           % k-1时刻 扣除重力后的运动加速度
    V_k_IMU = V_ksub1 + Motion_Acceleration_ksub1*Sample_Interval;       % k时刻 V
    MMM_Set(k,:) = Motion_Acceleration_ksub1;
    % ----------3、位置更新(Place)----------
    Place_k_IMU = Place_ksub1 + 0.5*(V_k_IMU + V_ksub1)*Sample_Interval; % k时刻 Place
    % ----------4、计算状态转移矩阵k-1时刻PHI----------
    % -----(1)A-----
    Eul_ksub1 = Qnb2Eul(Qua_ksub1);                 % k-1时刻 姿态欧拉角(角度)
    Eul_Rad_ksub1 = Eul_ksub1*pi/180;               % k-1时刻 姿态欧拉角(弧度)
    Pitch = Eul_Rad_ksub1(1);  
    Yaw = Eul_Rad_ksub1(3);
%     Mre = [-cos(Yaw) -sin(Yaw) 0; sin(Yaw)/cos(Pitch) -cos(Yaw)/cos(Pitch) 0; -tan(Pitch)*sin(Yaw) tan(Pitch)*cos(Yaw) -1];    % A = Mre*R
    Mer = [-cos(Yaw) cos(Pitch)*sin(Yaw) 0;-sin(Yaw) -cos(Pitch)*cos(Yaw) 0;0  -sin(Pitch) -1];                                % R = Mer*A
    % -----(2)V-----
    % Mva                         
    Mva = [0, -fn_ksub1(3), fn_ksub1(2); fn_ksub1(3), 0, -fn_ksub1(1); -fn_ksub1(2), fn_ksub1(1), 0]*Mer;
    % -----(3)P-----
    % ----------4、phi_k_ksub1----------
    %                A            V            P            
    phi_k_ksub1 = [  zeros(3),    zeros(3),    zeros(3);     % A
                     Mva,         zeros(3),    zeros(3);     % V
                     zeros(3),    eye(3),      zeros(3);];   % P
    PHI_k_ksub1 = eye(9) + phi_k_ksub1*Sample_Interval;                  % 离散化
    % ----------3、Kalman Filter----------
    % -----(1)状态一步预测-----
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;
    % -----(2)状态一步预测均方误差阵-----
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);                    % 对称化
    % -----(3)自适应估计Z_k-----
    Z_k_est = [Eul_AccMag_Set(k,:)';V_Alti_Set(k);High_Alti_Set(k)];
    Z_k = [Eul_k_IMU;V_k_IMU(3);Place_k_IMU(3)] - Z_k_est;
    if abs(Z_k(3)) > 90
        Z_k(3) = X_k_ksub1(3);
    end
    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1; 
    % -----(4)滤波增益-----
    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_ksub1;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    % -----(5)状态估计-----
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    % -----(6)状态估计均方误差阵-----
    P_k = (eye(9) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    % ----------4、avp修正值----------
    % -----(1)姿态-----
    Eul_k = Eul_k_IMU - X_k(1:3);             % 修正姿态欧拉角
    Eul_Set(k,:) = Eul_k;
    Qua_k = Eul2Qnb(Eul_k);        
    Qua_Set(k,:) = Qua_k';
    % -----(2)速度-----
    V_k(1,:) = V_k_IMU(1);
    V_k(2,:) = V_k_IMU(2);
    V_k(3,:) = V_k_IMU(3) - X_k(6);                 % 修正速度
    V_Set(k,:) = V_k';
    % -----(3)位置-----
    Place_k(1,:) = Place_k_IMU(1);
    Place_k(2,:) = Place_k_IMU(2);
    Place_k(3,:) = Place_k_IMU(3) - X_k(9);        % 修正位置
    Place_Set(k,:) = Place_k';
    Pos_k = Place2Pos(Place_k, Pos_Init);
    Pos_Set(k,:) = Pos_k';               % [纬度 经度 高度]
    % ----------5、状态归0----------
    X_k(1:9) = 0;
    % ----------6、参数迭代----------
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Qua_ksub1 = Qua_k;
    V_ksub1 = V_k;
    Place_ksub1 = Place_k;
end
for k = 1:Marg_Number
    p = Place_Set(k,:);
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
Serror_Set_all(k,:) = sqrt((p1-0)^2 + (p2-50*0.01*k)^2 + (p3-0)^2);
kkk_all(k,:) = Serror_Set_all(k,:)/(50*0.01*k);
end
for k = 1:Marg_Number
    p = Place_Set(k,:);
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
Serror_Set_1(k,:) = sqrt((p1-0)^2 + (p2-50*0.01*k)^2);
kkk_1(k,:) = Serror_Set_1(k,:)/(50*0.01*k);
end
for k = 1:Marg_Number
    p = Place_Set(k,:);
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
Serror_Set_2(k,:) = sqrt((p3-0)^2);
kkk_2(k,:) = Serror_Set_2(k,:)/(50*0.01*k);
end
% figure(1)
% plot((1:Marg_Number)/100,kkk_all(:,1)*100),title('位置相对误差'),xlabel('时间 t/s'),ylabel('相对误差（%行程）');
% set(gca,'FontSize',15); 
figure(2)
plot((1:Marg_Number)/100,kkk_1(:,1)*100),title('水平方向位置相对误差'),xlabel('时间 t/s'),ylabel('相对误差（%行程）');
set(gca,'FontSize',15); ylim([0 10]);
figure(3)
plot((1:Marg_Number)/100,kkk_2(:,1)*100),title('垂直方向位置相对误差'),xlabel('时间 t/s'),ylabel('相对误差（%行程）');
set(gca,'FontSize',15); ylim([0 1]);