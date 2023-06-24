function Eul_Set = SAESKF_Fun(File_Name, Q_ksub1, R_ksub1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：序贯自适应ESKF
%
% Prototype: Eul_SAESKF_Set = SAESKF_Fun(File_Name, Q_ksub1, R_ksub1)
% Inputs: File_Name - 文件名
%         Q_ksub1 - 状态噪声
%         R_ksub1 - 量测噪声
% Output: Eul_SAESKF_Set - 融合欧拉角集
%
% By Taoran Zhao
% 2023/04/26 2023/05/06 2023/05/19 2023/05/26
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ###############一、数据导入及参数设定#############################
% ----------1、导入数据----------
CalibrationParameter_No1;               % 加载1号模块标定参数
% ----------2、采样率----------
SAMPLE_FREQUENCY = 100;                 % 100Hz 
% ########################################################################
% ---------------二、标定数据---------------
% [Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
%     MargCalibrationNoZero(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.R);
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun_NoZero(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
% 未阈值内归零
% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
% 加速度、磁数据归一化
% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% ---------------五、ESKF初始化---------------
Sample_Interval = 1/SAMPLE_FREQUENCY;           % 采样间隔0.01s
% ----------1、初始att----------
Eul_Init = mean(Eul_AccMag_Set(1:300,:));       % 初始姿态欧拉角，取第50个到第200个欧拉角平均
Qnb_Init = Eul2Qnb(Eul_Init);                   % 初始姿态四元数                  
% ----------2、分配内存----------
Eul_Set = zeros(Marg_Number,3);                 % 提前分配Eul_Set内存
% ----------3、kalman初始化----------
Qnb_ksub1 = Qnb_Init; 
X_ksub1 = zeros(3,1);                           % 初始状态，即姿态误差为0              
P_ksub1 = eye(3);
H_k = eye(3);
Beta_Init = 1;
Beta_ksub1 = Beta_Init;
% ---------------三、ESKF--------------
for k = 1:Marg_Number
   % ----------1、解算姿态角预测值----------
    W_Deg_ksub1 = Gyro_Set(k,:);
    Qnb_k_ksub1 = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);        % k时刻 姿态四元数预测 一阶龙格库塔法
    Eul_k_ksub1 = Qnb2Eul(Qnb_k_ksub1);                                         % k时刻 未校正的姿态欧拉角
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
    [X_k, P_k, Beta_k, R_k] = SAKF(Z_k, X_k_ksub1, R_ksub1, Beta_ksub1, H_k, P_k_ksub1);
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
    % ----------4、att修正值----------
    Eul_k = Eul_k_ksub1 - X_k(1:3);             % 修正姿态欧拉角
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    % ----------5、状态归0----------
    X_k(1:3) = 0;
    % ----------6、变量迭代----------
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    Qnb_ksub1 = Qnb_k;
end






