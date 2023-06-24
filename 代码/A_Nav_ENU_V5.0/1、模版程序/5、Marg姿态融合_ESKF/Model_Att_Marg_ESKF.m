%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：运用误差状态Kalman滤波融合Marg姿态角(解耦版)
%
% By Taoran Zhao
% 2023/04/29 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ###############一、数据导入及参数设定#############################
% ----------1、导入数据----------
File_Name = 'marg1';                   % 读取文件
CalibrationParameter_No1;              % 加载1号模块标定参数
% ----------2、采样率----------
SAMPLE_FREQUENCY = 100;                % 100Hz    
% ----------3、ESKF参数----------
Q_Init = (1e-9)*eye(3);                            % 状态噪声方差阵
R_Init = (1e-6)*eye(3);                          % 量测噪声方差阵
% ----------4、是否作图----------
BOOL_Plot_Eul_AccMag_Set = 1;
BOOL_Plot_Eul_Set = 1;
% ########################################################################
%% ---------------二、标定数据---------------
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.R);
%% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS);      % 加速度、磁归一化
%% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
%% ---------------五、ESKF初始化---------------
Sample_Interval = 1/SAMPLE_FREQUENCY;     % 采样间隔0.01s
% ----------1、初始att----------
Eul_Init = mean(Eul_AccMag_Set(1:300,:)); % 初始姿态欧拉角
Qnb_Init = Eul2Qnb(Eul_Init);             % 初始姿态四元数                  
% ----------2、分配内存----------
Eul_Set = zeros(Marg_Number,3);           % 提前分配Eul_Set内存
% ----------3、kalman初始化----------
Qnb_ksub1 = Qnb_Init; 
X_ksub1 = zeros(3,1);                     % 初始状态，即姿态误差为0              
P_ksub1 = eye(3);
Q_ksub1 = Q_Init;                         % 状态噪声方差阵
R_ksub1 = R_Init;                         % 量测噪声方差阵
H_k = eye(3);
%% ---------------六、ESKF---------------
for k = 1:Marg_Number
    % ----------1、解算姿态角预测值----------
    W_Deg_ksub1 = Gyro_Set(k,:);
    Qnb_k_ksub1 = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);        % k时刻 姿态四元数预测 一阶龙格库塔法
    Eul_k_ksub1 = Qnb2Eul(Qnb_k_ksub1);                                         % k时刻 姿态欧拉角预测
    % ----------2、解算姿态角误差估计值：Kalman Filter----------
    % -----解算状态一步转移矩阵-----
    phi_k_ksub1 = zeros(3) ;                        
    PHI_k_ksub1 = eye(3) + phi_k_ksub1*Sample_Interval;    % 离散化
    % -----(1)状态一步预测：KF式1-----
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;
    % -----(2)状态一步预测均方误差阵：KF式2-----
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);                        % 对称化
    % -----(3)确定Z_k-----
    Eul_k_Acc = Eul_AccMag_Set(k,1:2)';                    % k时刻 倾角量测
    Eul_k_Mag = YawMag(Eul_k_ksub1(1), Eul_k_ksub1(2), Mag_Set(k,:));      % 2023/05/19 由倾角预测来解算航向角，这样航向角量测更加精准
    Eul_k_AccMag = [Eul_k_Acc;Eul_k_Mag];
    Z_k = Eul_k_ksub1 - Eul_k_AccMag;                                            
    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1; 
    % -----(4)滤波增益：KF式3-----
    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_ksub1;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    % -----(5)状态估计：KF式4-----
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    Z_k_Eul_Threshold = 5;
    if abs(Z_k(1)) > Z_k_Eul_Threshold
        X_k(1) = 0;
    end
    if abs(Z_k(2)) > Z_k_Eul_Threshold
        X_k(2) = 0;
    end
    if abs(Z_k(3)) > Z_k_Eul_Threshold
        X_k(3) = 0;
    end
    % 注：在ESKF中，状态、量测应均为小量精度才高(非线性系统线性化)，故由于万向节死锁、(横滚-180&180/航向0&360)同位跳变(必须去除)、磁干扰所带来的较大Z_k要去除
    % -----(6)状态估计均方误差阵：KF式5-----
    P_k = (eye(3) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    % ----------4、att修正值----------
    Eul_k = Eul_k_ksub1 - X_k(1:3);             % 修正姿态欧拉角
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    % ----------5、状态归0----------
    X_k(1:3) = 0;
    % ----------6、变量迭代----------
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Qnb_ksub1 = Qnb_k;
end
% ---------------七、作图---------------
if BOOL_Plot_Eul_AccMag_Set == 1
figure(1)
subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('姿态角量测');
end
if BOOL_Plot_Eul_Set == 1
figure(2)
subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('ESKF融合后的姿态角');
end



