%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：TEKF融合Marg姿态 (二段扩展Kalman滤波)
%
% Notice：见《徐小龙. 基于惯性-磁传感器的机器人位姿捕捉技术研究[D].山东大学,2020.DOI:10.27272/d.cnki.gshdu.2020.004078.》
%
% By Taoran Zhao
% 2023/05/06 2023/05/18
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
% ----------3、TEKF参数----------
Q_Init = (1e-9)*eye(4);                         % 状态噪声方差阵
R_Acc_Init = (1e-5)*eye(3);                     % 加速度计量测噪声方差阵
R_Mag_Init = (1e-6)*eye(3);                     % 磁传感器量测噪声方差阵
% ----------4、是否作图----------
BOOL_Plot_Eul_AccMag_Set = 1;
BOOL_Plot_Eul_Set = 1;
% ########################################################################
%% ---------------二、标定数据---------------
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.R);
%% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS);    % 加速度、磁场矢量归一化
%% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
%% ---------------五、DCF初始化---------------
Sample_Interval = 1/SAMPLE_FREQUENCY;            % 采样间隔0.01s
% ----------1、初始att----------
Eul_Init = mean(Eul_AccMag_Set(1:300,:));        % 初始姿态欧拉角
Qnb_A_Init = Eul2Qnb(Eul_Init);                  % 初始姿态四元数
Qnb_M_Init = [1;0;0;0];                          % 初始航向校正四元数
% ----------2、分配内存----------
Eul_Set = zeros(Marg_Number,3);                  % 提前分配Eul_Set内存 
% ----------3、kalman初始化---------- 
X_ksub1_A = Qnb_A_Init; 
X_ksub1_M = Qnb_M_Init;
P_ksub1_A = eye(4);
P_ksub1_M = eye(4);
Q_ksub1 = Q_Init;                               % 状态噪声方差阵
R_Acc_ksub1 = R_Acc_Init;                       % 加速度计量测噪声方差阵
R_Mag_ksub1 = R_Mag_Init;                       % 磁传感器量测噪声方差阵
h_Acc_k_Fun = @(q)([2*(q(2)*q(4) - q(1)*q(3)); 2*(q(3)*q(4) + q(1)*q(2)); q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2]);
H_Acc_k_Fun = @(q)(2*[-q(3), q(4), -q(1), q(2); q(2), q(1), q(4), q(3); q(1), -q(2), -q(3), q(4)]);    
h_Mag_k_Fun = @(q)([2*q(1)*q(4);q(1)^2 + q(4)^2 ;0]);
H_Mag_k_Fun = @(q)(2*[q(4), 0, 0, q(1);q(1), 0, 0, q(4);0, 0, 0, 0]);
%% ---------------六、TEKF---------------
for k = 1:Marg_Number
    % ----------1、融合加速度矢量和角速度矢量：EKF----------
    W_Deg_ksub1 = Gyro_Set(k,:);
    Wx = W_Deg_ksub1(1)*pi/180; Wy = W_Deg_ksub1(2)*pi/180; Wz = W_Deg_ksub1(3)*pi/180;
    Omega_A = [ 0,  -Wx, -Wy, -Wz;Wx,  0,   Wz, -Wy;Wy, -Wz,  0,   Wx;Wz,  Wy, -Wx,  0  ];                          
    PHI_k_ksub1_A = (eye(4) + 0.5*Sample_Interval*Omega_A);   
    % -----(1)状态一步预测-----
    X_k_ksub1_A = PHI_k_ksub1_A*X_ksub1_A;
    X_k_ksub1_A = NormlzQua(X_k_ksub1_A);
    % -----(2)状态一步预测均方误差阵-----
    P_k_ksub1_A = PHI_k_ksub1_A*P_ksub1_A*PHI_k_ksub1_A' + Q_ksub1;
    P_k_ksub1_A = P2diagP(P_k_ksub1_A);                    % 对称化
    % -----(3)确定Z_k-----
    Z_Acc_k = Acc_Set(k,:)';
    h_Acc_k = h_Acc_k_Fun(X_k_ksub1_A);
    H_Acc_k = H_Acc_k_Fun(X_k_ksub1_A);
    Z_Acc_error_k_ksub1 = Z_Acc_k - h_Acc_k; 
    % -----(4)滤波增益-----
    PXZ_Acc_k_ksub1 = P_k_ksub1_A*H_Acc_k';
    PZZ_Acc_k_ksub1 = H_Acc_k*PXZ_Acc_k_ksub1 + R_Acc_ksub1;
    K_Acc_k = PXZ_Acc_k_ksub1/PZZ_Acc_k_ksub1;
    % -----(5)状态估计-----
    X_k_A0 = X_k_ksub1_A + K_Acc_k*(Z_Acc_error_k_ksub1);
    X_k_A0 = NormlzQua(X_k_A0);
    % -----(6)状态估计均方误差阵-----
    P_k_A = (eye(4) - K_Acc_k*H_Acc_k)*P_k_ksub1_A;
    P_k_A = P2diagP(P_k_A);    
    % ----------2、从单位姿态四元数中剔除航向姿态信息----------
    % -----(1)变化四元数-----
    Qua_A_del = QuaMulQua(X_k_A0,QuaConj(X_k_ksub1_A));  
    Cnb_A_del = Qnb2Cnb(Qua_A_del);
    % -----(2)假设水平单位矢量[1;0;0]-----
    UnitV_b = [1;0;0];
    UnitV_n = Cnb_A_del*UnitV_b;
    UnitV_n(3) = 0; 
    UnitV_n = NormlzV3(UnitV_n);
    q3 = sqrt(0.5*(1 - UnitV_n(1)));
    q3_sign = sign(UnitV_n(2));
    if 0.5*(1 - UnitV_n(1)) <= 0
        Qua_del_Z = [1;0;0;0];
    else
       q3 = q3_sign*q3;
       q0 = UnitV_n(2)/(2*q3);
       Qua_del_Z = [q0;0;0;q3];
       Qua_del_Z = NormlzQua(Qua_del_Z);
    end 
    X_k_A = QuaMulQua(QuaConj(Qua_del_Z),X_k_A0);   % 将Qua_del_Z航向校正四元数剔除
    X_k_A = NormlzQua(X_k_A);
    Eul0 = Qnb2Eul(X_k_A);    
    % ----------3、融合磁场和角速度获得航向姿态：EKF----------
    % -----(1)从磁场矢量中提取与重力场垂直的水平分量-----
    Cnb_M = Qnb2Cnb(X_k_A);
    mb0 = Mag_Set(k,:)';
    m_mid0 = Cnb_M*mb0;         % 将磁场数据由载体坐标系转移到中间坐标系(导航系扣去航向校正变化)
    ml = m_mid0;
    ml(3) = 0;
    ml = NormlzV3(ml);          % 导航系下水平的量测
    % -----(2)从角速度矢量中提取垂直分量-----    
    W_b = [Wx;Wy;Wz];
    W_n0 = Cnb_M*W_b;
    W_n = [0;0;W_n0(3)];
    Wz_n = W_n(3);  % 导航系下的垂直速度
    % -----(3)融合-----
    Omega_M = [ 0,  0, 0, -Wz_n;
          0,  0,   Wz_n, 0;
          0, -Wz_n,  0,   0;
          Wz_n,  0, 0,  0  ];   
    PHI_k_ksub1_M = (eye(4)+0.5*Sample_Interval*Omega_M);   
    % -----(4)状态一步预测-----
    X_k_ksub1_M = PHI_k_ksub1_M*X_ksub1_M;
    X_k_ksub1_M = NormlzQua(X_k_ksub1_M);
    % -----(5)状态一步预测均方误差阵-----
    P_k_ksub1_M = PHI_k_ksub1_M*P_ksub1_M*PHI_k_ksub1_M' + Q_ksub1;
    P_k_ksub1_M = P2diagP(P_k_ksub1_M);                    % 对称化
    % -----(6)Z_k-----
    Z_Mag_k = ml;
    h_Mag_k = h_Mag_k_Fun(X_k_ksub1_M);
    H_Mag_k = H_Mag_k_Fun(X_k_ksub1_M);
    Z_Mag_error_k_ksub1 = Z_Mag_k - h_Mag_k; 
    % -----(7)滤波增益-----
    PXZ_Mag_k_ksub1 = P_k_ksub1_M*H_Mag_k';
    PZZ_Mag_k_ksub1 = H_Mag_k*PXZ_Mag_k_ksub1 + R_Mag_ksub1;
    K_Mag_k = PXZ_Mag_k_ksub1/PZZ_Mag_k_ksub1;
    % -----(8)状态估计-----
    X_k_M = X_k_ksub1_M + K_Mag_k*(Z_Mag_error_k_ksub1);
    X_k_M = NormlzQua(X_k_M);
    % -----(9)状态估计均方误差阵-----
    P_k_M = (eye(4) - K_Mag_k*H_Mag_k)*P_k_ksub1_M;
    P_k_M = P2diagP(P_k_M);
    Qua = QuaMulQua(X_k_M,X_k_A);
    Eul = Qnb2Eul(Qua);
    Eul_Set(k,:) = Eul;
    % 变量迭代
    X_ksub1_A = X_k_A;
    P_ksub1_A = P_k_A;
    X_ksub1_M = X_k_M;
    P_ksub1_M = P_k_M;    
end
%% ---------------七、作图---------------
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
sgtitle('TEKF融合后的姿态角');
end