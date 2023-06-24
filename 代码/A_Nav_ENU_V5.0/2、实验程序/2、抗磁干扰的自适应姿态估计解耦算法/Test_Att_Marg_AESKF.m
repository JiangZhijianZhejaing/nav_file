%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：运用误差状态Kalman滤波融合Marg姿态角(解耦版) + 自适应
%
% notice：位置采用大地坐标系(即，经纬度)
%
% By Taoran Zhao
% 2023/04/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、导入数据---------------
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入系统文件夹所有文件
CalibParm_No1_20230301;         % 加载1号模块标定参数
% ---------------二、标定数据---------------
[Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
    MargCalibNoZero('Copy_of_20230523', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargENU(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);
% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% ---------------五、ESKF初始化---------------
Sample_Interval = 0.01;                          % Gyro采样间隔
% ----------1、初始avp----------
Eul_Init = mean(Eul_AccMag_Set(50:200,:));       % 初始姿态欧拉角，取第50个到第200个欧拉角平均
Qnb_Init = Eul2Qnb(Eul_Init);                    % 初始姿态四元数                  
% ----------2、分配内存----------
Eul_AESKF_Set = zeros(Marg_Number,3);                  % 提前分配Eul_Set内存
Qnb_Set = zeros(Marg_Number,4);                  % 提前分配Qnb_Set内存  
% ----------3、kalman初始化----------
Qnb_ksub1 = Qnb_Init; 
X_ksub1 = zeros(3,1);                            % 初始状态，即姿态误差为0              
P_ksub1 = eye(3);
Q_ksub1 =  diag([1e-4,1e-5,1e-7]);               % 状态噪声方差阵
R_ksub1 = diag([1e-1,1e-1,1e-4]);                % 量测噪声方差阵
H_k = eye(3);
Beta_Init = 1;
Beta_ksub1 = Beta_Init;
% ---------------三、ESKF--------------
for k = 1:Marg_Number
    % ----------1、姿态更新----------
    W_Deg_ksub1 = Gyro_Set(k,:);
    % 一阶龙格库塔法
    Qnb_k_Gyro = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);        % k时刻 未校正的Qua 一阶龙格库塔法
    Eul_k_Gyro = Qnb2Eul(Qnb_k_Gyro);                                          % k时刻 未校正的姿态欧拉角
    % ----------2、解算状态一步转移矩阵----------
    phi_k_ksub1 = zeros(3) ;                        
    PHI_k_ksub1 = eye(3) + phi_k_ksub1*Sample_Interval;                        % 离散化
    % ----------3、Kalman Filter----------
    % -----(1)状态一步预测-----
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;
    % -----(2)状态一步预测均方误差阵-----
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);    % 对称化
    % -----(3)自适应估计Z_k-----
    Eul_k_AccMag0 = Eul_AccMag_Set(k,:)';                                       % k时刻 静态姿态欧拉角
    Eul_k_AccMag = Eul_k_AccMag0;
    Eul_k_AccMag(3) = YawMag(Eul_k_Gyro(1), Eul_k_Gyro(2), Mag_Set(k,:));       % 2023/05/19 由预测俯仰角、横滚角来解算航向角，这样航向量测更加精准
    Z_k = Eul_k_Gyro - Eul_k_AccMag;
    if abs(Z_k(3)) > 5
        Z_k(3) = X_k_ksub1(3);
    end
    [X_k, P_k, Beta_k, R_k] = ASKF(Z_k, X_k_ksub1, R_ksub1, Beta_ksub1, H_k, P_k_ksub1);
    % ----------4、avp修正值----------
    Eul_k = Eul_k_Gyro - X_k(1:3);             % 修正姿态欧拉角
    Eul_AESKF_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    Qnb_Set(k,:) = Qnb_k';
    % ----------5、状态归0----------
    X_k(1:3) = 0;
    % ----------6、参数迭代----------
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    Qnb_ksub1 = Qnb_k;
end
Yaw_MSE0 = 0;
for k = 1:Marg_Number
   Yaw_E =  Eul_AESKF_Set(k,3) - Eul_Init(3);
   Yaw_SE = Yaw_E^2;
   Yaw_MSE0 = Yaw_MSE0 + Yaw_SE;
end
Yaw_MSE = Yaw_MSE0/Marg_Number;
Yaw_RMSE = sqrt(Yaw_MSE);
% ---------------七、作图---------------
figure(1)
subplot(3,1,1),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('姿态角量测');
figure(2)
subplot(3,1,1),plot((1:Marg_Number)/100,Eul_AESKF_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/100,Eul_AESKF_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/100,Eul_AESKF_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('ESKF融合后的姿态角');



