function [X_k, P_k, R_k, Beta_k] = Eskf(Sample_Interval, fn_ksub1, Qnb_k_0 , X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, b, R_ksub1, Eul_AccMag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：ESKF滤波
%
% Prototype: Qnb_k = Rungekuta(Qnb_ksub1, Sample_Interval, k, Gyro_Set, Qnb_Set)
% Inputs: Sample_Interval - 采样间隔
%         fn_ksub1 - fn
%         Qnb_k_0 - k时刻未修正四元数
%         X_ksub1 - k - 1时刻状态估计
%         P_ksub1 - k - 1时刻误差阵
%         Q_ksub1 - k - 1时刻误差阵
%         H_k - k - 1时刻量测矩阵
%         Beta_ksub1 - k - 1时刻渐消因子
%         b - k - 0.95
%         R_ksub1 - k - 1时刻量测噪声
%         Eul_AccMag - k时刻静态姿态角
% Output: X_k - k时刻状态估计
%         P_k - k时刻误差
%         R_k - k时刻量测噪声
%         Beta_k - k时刻渐消因子
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% 计算状态转移矩阵PHI  
    Eul = Qnb2Eul(Qnb_k_0);
    Eul_Rad = Eul*pi/180; % 弧度
    Pitch = Eul_Rad(1);  
    Yaw = Eul_Rad(3);
    Cnb = Qnb2Cnb(Qnb_k_0);
    % Mr2e 和 Me2r 是用于计算角速度和角加速度的矩阵。Ma1、Mva、Mv2 和 Mpv 是用于计算状态转移矩阵 PHI 的矩阵。
    Mr2e = [-cos(Yaw) -sin(Yaw) 0; sin(Yaw)/cos(Pitch) -cos(Yaw)/cos(Pitch) 0; -tan(Pitch)*sin(Yaw) tan(Pitch)*cos(Yaw) -1];    % A = Mr2e*R
    Me2r = [-cos(Yaw) cos(Pitch)*sin(Yaw) 0;-sin(Yaw) -cos(Pitch)*cos(Yaw) 0;0  -sin(Pitch) -1];                                % R = Me2r*A
    Ma1 = Mr2e*(-Cnb);
    Mva = [0, -fn_ksub1(3), fn_ksub1(2); fn_ksub1(3), 0, -fn_ksub1(1); -fn_ksub1(2), fn_ksub1(1), 0]*Me2r;
    Mv2 = Cnb;
    Mpv = eye(3);

    %                A            v            p            eb           db      角速度误差、速度误差、位置误差、陀螺仪偏差和加速度计偏差。
    phi_k_ksub1 = [  zeros(3),    zeros(3),    zeros(3),    Ma1 ,        zeros(3) ;
                     Mva,         zeros(3),    zeros(3),    zeros(3),    Mv2;
                     zeros(3),    Mpv,         zeros(3),    zeros(3),    zeros(3);
                     zeros(3),    zeros(3),    zeros(3),    zeros(3),    zeros(3);
                     zeros(3),    zeros(3),    zeros(3),    zeros(3),    zeros(3);];
    
    PHI_k_ksub1 = eye(15) + phi_k_ksub1*Sample_Interval;
    %%
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;

    % P阵预测
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);
 
    Z_k = Eul - Eul_AccMag;


    if abs(Z_k(3)) > 90
        Z_k(3) = X_k_ksub1(3);
    end

    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1;   
    Beta_k = Beta_ksub1/(Beta_ksub1 + b);
    R_k = (1 - Beta_k)*R_ksub1 + Beta_k*(Z_error_k_ksub1*Z_error_k_ksub1' - H_k*P_k_ksub1*H_k');

    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_k;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    
    P_k = (eye(15) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    
    