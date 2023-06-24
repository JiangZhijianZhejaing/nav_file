%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：运用误差状态Kalman滤波融合Marg姿态角(解耦版)
%
% By Taoran Zhao
% 2023/04/29 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、导入数据---------------
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ---------------一、导入数据---------------
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
% CalibParm_No1;                         % 加载1号模块标定参数
% %% ---------------二、标定数据---------------
% [Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
%     MargCalib('marg1', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% %% ---------------三、转换东北天坐标系---------------
% [Gyro_Set, Acc_Set, Mag_Set] = MargENU(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);    % 加速度、磁场矢量归一化
format long;
Marg_Raw_Set = xlsread('水平', 'sheet1');             % 导入惯性传感器原始数据文件
Marg_Number = size(Marg_Raw_Set,1);                   % MARG采样数
Gyro_Set = Marg_Raw_Set(:,4:6);                       % 陀螺仪原始数据集
Acc_Set = Marg_Raw_Set(:,1:3);                        % 加速度计原始数据集
Mag_Set = Marg_Raw_Set(:,7:9);                        % 磁传感器原始数据集
for k = 1:Marg_Number
    if k >= 6
        Gyro_Set(k,1) = mean(Gyro_Set(k-5:k,1));
        Gyro_Set(k,2) = mean(Gyro_Set(k-5:k,2));
        Gyro_Set(k,3) = mean(Gyro_Set(k-5:k,3));
    end
end

%% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag1(Acc_Set, Mag_Set);
%% ---------------五、ESKF初始化---------------
Sample_Interval = 0.001;                          % Gyro采样间隔
% ----------1、初始avp----------
Eul_Init = mean(Eul_AccMag_Set(1:300,:));        % 初始姿态欧拉角
Qnb_Init = Eul2Qnb(Eul_Init);                    % 初始姿态四元数                  
% ----------2、分配内存----------
Eul_Set = zeros(Marg_Number,3);                  % 提前分配Eul_Set内存
% ----------3、kalman初始化----------
Qnb_ksub1 = Qnb_Init; 
X_ksub1 = zeros(3,1);                            % 初始状态，即姿态误差为0              
P_ksub1 = eye(3);
Q_ksub1 = (1e-150)*eye(3);                         % 状态噪声方差阵
R_ksub1 = (1e-7)*eye(3);                         % 量测噪声方差阵
% Q_ksub1 = (1e-13)*eye(3);                         % 状态噪声方差阵
% R_ksub1 = (1e-7)*eye(3);                         % 量测噪声方差阵
H_k = eye(3);
% ---------------三、ESKF--------------
for k = 1:Marg_Number
    % ----------1、姿态更新----------
    W_Deg_ksub1 = Gyro_Set(k,:);
    Qnb_k_ksub1 = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);        % k时刻 姿态四元数预测 一阶龙格库塔法
    Eul_k_ksub1 = Qnb2Eul(Qnb_k_ksub1);                                         % k时刻 姿态欧拉角预测
    % ----------2、解算状态一步转移矩阵----------
    phi_k_ksub1 = zeros(3) ;                        
    PHI_k_ksub1 = eye(3) + phi_k_ksub1*Sample_Interval;                         % 离散化
    % ----------3、Kalman Filter----------
    % -----(1)状态一步预测-----
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;
    % -----(2)状态一步预测均方误差阵-----
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);    % 对称化
    % -----(3)Z_k-----
    Eul_k_AccMag0 = Eul_AccMag_Set(k,:)';                                       % k时刻 静态姿态欧拉角(倾角量测+磁场矢量->航向角)
    Eul_k_AccMag = Eul_k_AccMag0;
    Eul_k_AccMag(3) = YawMag(Eul_k_ksub1(1), Eul_k_ksub1(2), Mag_Set(k,:));     % 2023/05/19 由倾角预测来解算航向角，这样航向角量测更加精准
    Z_k = Eul_k_ksub1 - Eul_k_AccMag;                                            
%     if abs(Z_k(3)) > 5      % 阈值暂定5，可能偏大 2023/05/26
%         Z_k(3) = X_k_ksub1(3);
%     end 
    % 注：在ESKF中，状态、量测应均为小量精度才高(非线性系统线性化)，故由于万向节死锁、0-360同位(必须去除)、磁干扰所带来的较大Z_k要去除
    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1; 
    % -----(4)滤波增益-----
    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_ksub1;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    % -----(5)状态估计-----
%     K_k = zeros(3,3);
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    % -----(6)状态估计均方误差阵-----
    P_k = (eye(3) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    % ----------4、avp修正值----------
    Eul_k = Eul_k_ksub1 - X_k(1:3);             % 修正姿态欧拉角
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    % ----------5、状态归0----------
    X_k(1:3) = 0;
    % ----------6、参数迭代----------
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Qnb_ksub1 = Qnb_k;
end
% ---------------七、作图---------------
figure(1)
subplot(3,1,1),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('姿态角量测');
figure(2)
subplot(3,1,1),plot((1:Marg_Number)/100,Eul_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/100,Eul_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/100,Eul_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('ESKF融合后的姿态角');



