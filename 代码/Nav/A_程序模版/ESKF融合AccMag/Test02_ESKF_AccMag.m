%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：运用误差状态Kalman滤波融合动静态姿态角(低成本版)
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入主文件夹所有m文件
gvar;                         % 加载地球参数
CalibParm_No1;                  % 加载1号模块标定参数
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%%
Sample_Interval = 0.01;                                 % Gyro采样间隔
% 初始avp
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % 取前3秒平均为初始姿态欧拉角
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % 初始姿态四元数
Qnb_Set = zeros(4,Marg_Number);

V_ksub1 = [0;0;0];                                      % 初始速度
Place_ksub1 = [0; 0; 0];                                % 初始当地直角坐标系位置
Pos_ksub1 = [31.90209*pi/180;117.17*pi/180;15];         % 初始经纬坐标(由GNSS获得)
gn = CalcGn(Pos_ksub1); 
Eul_Set = zeros(Marg_Number,3);
V_Set = zeros(Marg_Number,3);
Place_Set = zeros(Marg_Number,3);
% kalman初始化
X_ksub1 = zeros(15,1);
P_ksub1 = eye(15);
Q_ksub1 = (1e-9)*eye(15);                     % 状态噪声方差阵
R_ksub1 = (1e-1)*eye(3);                     % 加速度计量测噪声方差阵

H_k = [eye(3),zeros(3,12)];
Beta_ksub1 = 1;
b = 0.95;

for k = 1:Marg_Number
    % 惯导更新算法
    [Qnb_k_0, V_k_0, Place_k_0, fn_ksub1] = SinsUpdate...
        (Qnb_ksub1, V_ksub1, Place_ksub1, gn, Acc_Set, Gyro_Set, Qnb_Set, Sample_Interval, k);

    Eul_k_0 = Qnb2Eul(Qnb_k_0);           % 未修正的姿态欧拉角
    
    Eul_AccMag = Eul_AccMag_Set(k,:)';
    
    [X_k, P_k, R_k, Beta_k] = Eskf(Sample_Interval, fn_ksub1, Qnb_k_0 , X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, b, R_ksub1, Eul_AccMag);
    
    Eul_k = Eul_k_0 - X_k(1:3);             % 修正姿态欧拉角
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    Qnb_Set(:,k) = Qnb_k;

    V_k = V_k_0 - X_k(4:6);                   % 修正速度
    V_Set(k,:) = V_k;

    Place_k = Place_k_0 - X_k(7:9);
    Place_Set(k,:) = Place_k;

    X_k(1:9) = 0;
    
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    
    Qnb_ksub1 = Qnb_k;
    V_ksub1 = V_k;
    Place_ksub1 = Place_k;
    
end
%%
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');
figure(3)
subplot(3,1,1),plot(V_Set(:,1)),title('VE');
subplot(3,1,2),plot(V_Set(:,2)),title('VN');
subplot(3,1,3),plot(V_Set(:,3)),title('VU');
figure(4)
subplot(3,1,1),plot(Place_Set(:,1)),title('L');
subplot(3,1,2),plot(Place_Set(:,2)),title('λ');
subplot(3,1,3),plot(Place_Set(:,3)),title('h');



