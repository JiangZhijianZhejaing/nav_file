%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：序贯自适应TEKF融合Marg姿态 (二段扩展Kalman滤波)
%
% Notice：见《徐小龙. 基于惯性-磁传感器的机器人位姿捕捉技术研究[D].山东大学,2020.DOI:10.27272/d.cnki.gshdu.2020.004078.》
%
% By Taoran Zhao
% 2023/05/06 2023/05/18 2023/05/23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                            % 清理命令行
clear;                          % 清理工作区

tic
File = '20230606_static';
P_ksub1_A = eye(4);
P_ksub1_M = eye(4);
Q_ksub1_A = (1e-7)*eye(4);                      % 状态噪声方差阵
Q_ksub1_M = (1e-11)*eye(4);
R_Acc_ksub1 = (1e-1)*eye(3);                     % 加速度计量测噪声方差阵
R_Mag_ksub1 = (1e-1)*eye(3);                     % 磁传感器量测噪声方差阵
Eul_SATEKF_Set = ASTEKF_Fun(File,P_ksub1_A,P_ksub1_M,Q_ksub1_A,Q_ksub1_M,R_Acc_ksub1,R_Mag_ksub1);
toc


% %% ---------------一、导入数据---------------
% clc;                            % 清理命令行
% clear;                          % 清理工作区
% addpath(genpath('../../'));     % 导入系统文件夹所有文件
% CalibParm_No1;         % 加载1号模块标定参数
% % ---------------二、标定数据---------------
% [Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
%     MargCalib_NoZero('Copy_of_20230523', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% % ---------------三、转换东北天坐标系---------------
% [Gyro_Set, Acc_Set, Mag_Set] = MargENU(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);
% % ---------------四、解算静态姿态欧拉角---------------
% Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% %% ---------------五、DCF初始化---------------
% Sample_Interval = 0.01;                          % Gyro采样间隔
% % ----------1、初始avp----------
% Eul_Init = mean(Eul_AccMag_Set(50:250,:));       % 初始姿态欧拉角，取第50个到第250个欧拉角平均
% Qnb_A_Init = Eul2Qnb(Eul_Init);                  % 初始姿态四元数
% Qnb_M_Init = [1;0;0;0];                          % 初始航向校正四元数
% % ----------2、分配内存----------
% Eul_ATEKF_Set = zeros(Marg_Number,3);            % 提前分配Eul_ATEKF_Set内存
% % ----------3、kalman初始化---------- 
% X_ksub1_A = Qnb_A_Init; 
% X_ksub1_M = Qnb_M_Init;
% P_ksub1_A = (1e-5)*eye(4);
% P_ksub1_M = eye(4);
% Q_ksub1_A = (1e-7)*eye(4);                       % 状态噪声方差阵
% Q_ksub1_M = (1e-11)*eye(4);
% R_ksub1_A = (1e-4)*eye(3);                     % 加速度计量测噪声方差阵
% R_ksub1_M = (1e-3)*eye(3);                     % 磁传感器量测噪声方差阵
% h_k_Fun_A = @(q)([2*(q(2)*q(4) - q(1)*q(3)); 2*(q(3)*q(4) + q(1)*q(2)); q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2]);
% H_k_Fun_A = @(q)(2*[-q(3), q(4), -q(1), q(2); q(2), q(1), q(4), q(3); q(1), -q(2), -q(3), q(4)]);    
% h_k_Fun_M = @(q)([2*q(1)*q(4);q(1)^2 + q(4)^2 ;0]);
% H_k_Fun_M = @(q)(2*[q(4), 0, 0, q(1);q(1), 0, 0, q(4);0, 0, 0, 0]);
% Beta_Init = 1;
% Beta_ksub1 = Beta_Init;
% % ---------------六、TEKF---------------
% for k = 1:Marg_Number
%     % ----------1、融合加速度矢量和角速度矢量估计物体姿态信息----------
%     W_Deg_ksub1 = Gyro_Set(k,:);
%     Wx = W_Deg_ksub1(1)*pi/180; Wy = W_Deg_ksub1(2)*pi/180; Wz = W_Deg_ksub1(3)*pi/180;
%     Omega_A = [ 0,  -Wx, -Wy, -Wz;Wx,  0,   Wz, -Wy;Wy, -Wz,  0,   Wx;Wz,  Wy, -Wx,  0  ];                          
%     PHI_k_ksub1_A = (eye(4) + 0.5*Sample_Interval*Omega_A); 
%     Z_k_A = Acc_Set(k,:)';
%     [X_k_A, P_k_A, X_k_ksub1_A, Beta_k, R_k_A] = ASTEKF_EKF...
%         (PHI_k_ksub1_A, X_ksub1_A, P_ksub1_A, Q_ksub1_A, R_ksub1_A, Z_k_A, h_k_Fun_A, H_k_Fun_A, Beta_ksub1);
%     % ----------2、从单位姿态四元数中剔除航向姿态信息----------
%     % -----(1)变化四元数-----
%     Qnb_k_A_del = QuaMulQua(X_k_A, QuaConj(X_k_ksub1_A)); 
%     Cnb_k_A_del = Qnb2Cnb(Qnb_k_A_del);
%     % -----(2)假设X_k_A的载体坐标系下有一水平单位矢量[1;0;0]-----
%     UnitV_b = [1;0;0];
%     UnitV_n = Cnb_k_A_del*UnitV_b;
%     UnitV_n(3) = 0; 
%     UnitV_n = NormlzV3(UnitV_n);
%     q3 = sqrt(0.5*(1 - UnitV_n(1)));
%     q3_sign = sign(UnitV_n(2));
%     if 0.5*(1 - UnitV_n(1)) <= 0
%        Qnb_k_A_del_Z = [1;0;0;0];
%     else
%        q3 = q3_sign*q3;
%        q0 = UnitV_n(2)/(2*q3);
%        Qnb_k_A_del_Z = [q0;0;0;q3];
%        Qnb_k_A_del_Z = NormlzQua(Qnb_k_A_del_Z);
%     end 
%     X_k_A_WithoutYaw = QuaMulQua(QuaConj(Qnb_k_A_del_Z), X_k_A);   % 将Qnb_del_Z航向校正四元数剔除
%     X_k_A_WithoutYaw = NormlzQua(X_k_A_WithoutYaw);
%     X_ksub1_A = X_k_A_WithoutYaw;
%     P_ksub1_A = P_k_A;
%     % ----------3、从磁场矢量中提取与重力场垂直的水平分量----------
%     Cnb_M = Qnb2Cnb(X_k_A_WithoutYaw);
%     m_b = Mag_Set(k,:)';
%     m_mid = Cnb_M*m_b;         % 将磁场数据由载体坐标系转移到中间坐标系(导航系扣去航向校正变化)
%     m_nl = m_mid;
%     m_nl(3) = 0;
%     m_nl = NormlzV3(m_nl);     % 导航系下水平的量测
%     % ----------4、从角速度矢量中提取垂直分量---------- 
%     W_b = W_Deg_ksub1'*pi/180;
%     W_mid = Cnb_M*W_b;         % 中间坐标系的角速度
%     W_n = [0;0;W_mid(3)];      % 导航系下的角速度
%     Wz_n = W_n(3);             % 导航系下的垂直角速度
%     % ----------5、融合磁场和角速度获得航向姿态信息----------
%     Omega_M = [ 0,  0, 0, -Wz_n;0,  0,   Wz_n, 0;0, -Wz_n,  0,   0;Wz_n,  0, 0,  0  ];   
%     PHI_k_ksub1_M = (eye(4) + 0.5*Sample_Interval*Omega_M);  
%     Z_k_M = m_nl;
%     [X_k_M, P_k_M, X_k_ksub1_M, Beta_k, R_k_M] = ASTEKF_EKF...
%         (PHI_k_ksub1_M, X_ksub1_M, P_ksub1_M, Q_ksub1_M, R_ksub1_M, Z_k_M, h_k_Fun_M, H_k_Fun_M, Beta_ksub1);
%     Qnb_k = QuaMulQua(X_k_M, X_k_A_WithoutYaw);
%     Eul_k = Qnb2Eul(Qnb_k);
%     Eul_ATEKF_Set(k,:) = Eul_k;
%     X_ksub1_M = X_k_M;
%     P_ksub1_M = P_k_M;  
%     Beta_ksub1 = Beta_k;
%     R_ksub1_A = R_k_A;
%     R_ksub1_M = R_k_M;
% end
% Yaw_MSE0 = 0;
% for k = 1:Marg_Number
%    Yaw_E =  Eul_ATEKF_Set(k,3) - Eul_Init(3);
%    Yaw_SE = Yaw_E^2;
%    Yaw_MSE0 = Yaw_MSE0 + Yaw_SE;
% end
% Yaw_MSE = Yaw_MSE0/Marg_Number;
% Yaw_RMSE = sqrt(Yaw_MSE);
% % ---------------七、作图---------------
% figure(1)
% subplot(3,1,1),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
% subplot(3,1,2),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
% subplot(3,1,3),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
% sgtitle('姿态角量测');
% figure(2)
% subplot(3,1,1),plot((1:Marg_Number)/100,Eul_ATEKF_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
% subplot(3,1,2),plot((1:Marg_Number)/100,Eul_ATEKF_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
% subplot(3,1,3),plot((1:Marg_Number)/100,Eul_ATEKF_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
% sgtitle('ASTEKF融合后的姿态角');