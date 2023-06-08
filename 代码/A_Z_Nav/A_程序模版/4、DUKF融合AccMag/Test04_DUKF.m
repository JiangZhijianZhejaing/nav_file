%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：DUKF融合Gyro\Acc\Mag
%
% notice：P的初始值不宜过大，容易非正定
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------1、标定数据并计算静态姿态角---------------
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入主文件夹所有m文件
% gvar;                         % 加载地球参数
CalibParm_No1;                  % 加载1号模块标定参数
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);

figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%% ---------------2、DUKF初始化---------------
Sample_Interval = 0.01;                                 % Gyro采样间隔
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % 取前3秒平均为初始姿态欧拉角
X_ksub1 = Eul2Qnb(Eul_AccMag_Init);                     % 初始姿态四元数
Eul_Set = zeros(Marg_Number,3);                         % Eul_Set 提前分配内存空间 
% -----计算当地gn值-----
Pos_ksub1 = [[31.90209; 117.17]*pi/180; 15];            % 初始经纬坐标(由GNSS获得)：合肥
gn = CalcGn(Pos_ksub1);                                 
% -----kalman初始化-----
P_ksub1 = (1e-3)*eye(4);      % 不要设的大于单位矩阵，会失去正定性(2023/04/06)
Q_ksub1 = (1e-5)*eye(4);      % 状态噪声方差阵
R_Acc_ksub1 = (1e-2)*eye(3);  % 加速度计量测噪声方差阵
R_Mag_ksub1 = (1e-2)*eye(3);  % 加速度计量测噪声方差阵
h_Acc_k_Fun = @(q)([2*(q(2)*q(4) - q(1)*q(3)); 2*(q(3)*q(4) + q(1)*q(2)); q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2]);
h_Mag_k_Fun = @(q,mny,mnz)([2*(q(2)*q(3) + q(1)*q(4))*mny + 2*(q(2)*q(4) - q(1)*q(3))*mnz;
                            (q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2)*mny + 2*(q(3)*q(4) + q(1)*q(2))*mnz;
                            2*(q(3)*q(4) - q(1)*q(2))*mny + (q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2)*mnz]);

%% ---------------3、DUKF--------------
for k = 1:Marg_Number
    % -----解算一步状态转移矩阵-----           
    Wxyz_ksub1 = Gyro_Set(k,:)*pi/180;   % 转弧度   
    Wx = Wxyz_ksub1(1);   Wy = Wxyz_ksub1(2);   Wz = Wxyz_ksub1(3);  
    PHI_k_ksub1 = eye(4) + 0.5*Sample_Interval*[0, -Wx, -Wy, -Wz; Wx, 0, Wz, -Wy; Wy, -Wz, 0, Wx; Wz, Wy, -Wx, 0];
    % -----DUKF-----
    [X_k,P_k] = Dukf(PHI_k_ksub1, X_ksub1, P_ksub1, Acc_Set, Mag_Set, Q_ksub1, R_Acc_ksub1, R_Mag_ksub1, k, h_Acc_k_Fun, h_Mag_k_Fun);
    Eul = Qnb2Eul(X_k);
    Eul_Set(k,:) = Eul;
    X_ksub1= X_k;
    P_ksub1 = P_k;
end
%%
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');