%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：基于卡尔曼滤波的姿态更新算法
%
% By Zhijian Jiang
% 2023/04/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ---------------1、标定数据并计算静态姿态角---------------
addpath(genpath('../../'));     % 导入主文件夹所有m文件
clc;                            % 清理命令行
clear;                          % 清理工作区
gvar;                           % 加载地球参数
CalibParm_No1;          %加载惯导更新
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
% -----Eul_AccMag_Set画图-----
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%% ---------------2、ESKF初始化--------------- 
Sample_Interval = 0.01;                                 % Gyro采样间隔
% -----初始avp-----
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % 取前3秒平均为初始姿态欧拉角
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % 初始姿态四元数
V_ksub1 = [0;0;0];                                      % 初始速度
Place_ksub1 = [0; 0; 0];                                % 初始当地直角坐标系位置
% -----计算当地gn值-----
Pos_ksub1 = [31.90209*pi/180;117.17*pi/180;15];         % 初始经纬坐标(由GNSS获得)：合肥
gn = CalcGn(Pos_ksub1);   
Qnb_Set = zeros(4,Marg_Number);                         % 预分配Qnb_Set内存
Eul_Set = zeros(Marg_Number,3);
V_Set = zeros(Marg_Number,3);
Place_Set = zeros(Marg_Number,3);
% -----kalman初始化-----
X_ksub1 = zeros(15,1);
P_ksub1 = eye(15);
Q_ksub1 = (1e-9)*eye(15);                     % 状态噪声方差阵
R_ksub1 = (1e-1)*eye(3);                     % 加速度计量测噪声方差阵
H_k = [eye(3),zeros(3,12)];
Beta_ksub1 = 1;
b = 0.95;





