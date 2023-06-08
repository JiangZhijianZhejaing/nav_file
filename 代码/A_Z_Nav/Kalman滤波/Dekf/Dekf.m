function [X_k, P_k, Beta_k, R_Acc_k, R_Mag_k] = Dekf(X_ksub1, Gyro_Set, Acc_Set, Mag_Set, Sample_Interval, P_ksub1, Q_ksub1, R_Acc_ksub1, R_Mag_ksub1, k, b, Beta_ksub1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：四元数更新-四阶龙格库塔法
%
% Prototype: Qnb_k = Rungekuta(Qnb_ksub1, Sample_Interval, k, Gyro_Set, Qnb_Set)
% Inputs: Qnb_ksub1 - k-1时刻姿态四元数
%         Sample_Interval - 采样间隔
%         k - 当前时刻
%         Gyro_Set - 角速度集
%         Qnb_Set - 已有的姿态四元数集
% Output: Qnb_k - k时刻姿态四元数
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    h_Acc_k_Fun = @(q)([2*(q(2)*q(4) - q(1)*q(3)); 2*(q(3)*q(4) + q(1)*q(2)); q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2]);
    H_Acc_k_Fun = @(q)(2*[-q(3), q(4), -q(1), q(2); q(2), q(1), q(4), q(3); q(1), -q(2), -q(3), q(4)]);
    h_Mag_k_Fun = @(q,mny,mnz)([2*(q(2)*q(3) + q(1)*q(4))*mny + 2*(q(2)*q(4) - q(1)*q(3))*mnz;
                                (q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2)*mny + 2*(q(3)*q(4) + q(1)*q(2))*mnz;
                                2*(q(3)*q(4) - q(1)*q(2))*mny + (q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2)*mnz]);
    H_Mag_k_Fun = @(q,mny,mnz)(2*[   q(4)*mny - q(3)*mnz, q(3)*mny + q(4)*mnz, q(2)*mny - q(1)*mnz, q(1)*mny + q(2)*mnz; 
                                q(1)*mny + q(2)*mnz, -q(2)*mny + q(1)*mnz, q(3)*mny + q(4)*mnz, -q(4)*mny + q(3)*mnz;
                                -q(2)*mny + q(1)*mnz, -q(1)*mny - q(2)*mnz, q(4)*mny - q(3)*mnz, q(3)*mny + q(4)*mnz    ]);  
    % 1、状态一步预测
    Wxyz_ksub1 = Gyro_Set(k,:)*pi/180;   % 转弧度   
    Wx = Wxyz_ksub1(1);   Wy = Wxyz_ksub1(2);   Wz = Wxyz_ksub1(3);  
    PHI_k_ksub1 = eye(4) + 0.5*Sample_Interval*[0, -Wx, -Wy, -Wz; Wx, 0, Wz, -Wy; Wy, -Wz, 0, Wx; Wz, Wy, -Wx, 0];
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;  
    X_k_ksub1 = NormlzQnb(X_k_ksub1);
    % 2、状态一步预测均方误差阵
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);
    %% 第一段Acc
    % 3、量测误差计算
    Z_Acc_k = Acc_Set(k,:)';
    h_Acc_k = h_Acc_k_Fun(X_k_ksub1);
    H_Acc_k = H_Acc_k_Fun(X_k_ksub1);
    Z_Acc_k_ksub1_err = Z_Acc_k - h_Acc_k;
    Beta_k = Beta_ksub1/(Beta_ksub1 + b);
    R_Acc_k = (1 - Beta_k)*R_Acc_ksub1 + Beta_k*(Z_Acc_k_ksub1_err*Z_Acc_k_ksub1_err' - H_Acc_k*P_k_ksub1*H_Acc_k');
    PXZ_Acc_k_ksub1 = P_k_ksub1*H_Acc_k';
    PZZ_Acc_k_ksub1 = H_Acc_k*PXZ_Acc_k_ksub1 + R_Acc_k;
    K_Acc_k = PXZ_Acc_k_ksub1/PZZ_Acc_k_ksub1;
    del_X_Acc = K_Acc_k*(Z_Acc_k_ksub1_err);
    X_del_Acc = del_X_Acc.*[1;1;1;0];
    X_Acc_k = X_k_ksub1 + X_del_Acc; 
    P_Acc_k = P_k_ksub1 - K_Acc_k*PZZ_Acc_k_ksub1*K_Acc_k';
    P_Acc_k = P2diagP(P_Acc_k);
    %% 第二段Mag
    Cnb = Qnb2Cnb(X_k_ksub1);
    mn = Cnb*Mag_Set(k,:)';
    mny = mn(2);
    mnz = mn(3);
    h_Mag_k = h_Mag_k_Fun(X_k_ksub1,mny,mnz);
    H_Mag_k = H_Mag_k_Fun(X_k_ksub1,mny,mnz);
    Z_Mag_k = Mag_Set(k,:)';
    Z_Mag_k_ksub1_err = Z_Mag_k - h_Mag_k;
    R_Mag_k = (1 - Beta_k)*R_Mag_ksub1 + Beta_k*(Z_Mag_k_ksub1_err*Z_Mag_k_ksub1_err' - H_Mag_k*P_k_ksub1*H_Mag_k');
    PXZ_Mag_k_ksub1 = P_k_ksub1*H_Mag_k';
    PZZ_Mag_k_ksub1 = H_Mag_k*PXZ_Mag_k_ksub1 + R_Mag_k;
    K_Mag_k = PXZ_Mag_k_ksub1/PZZ_Mag_k_ksub1;
    del_X_Mag = K_Mag_k*(Z_Mag_k_ksub1_err);
    X_del_Mag = del_X_Mag.*[1;0;0;1];
    X_k = X_Acc_k + X_del_Mag; 
    P_k = P_Acc_k - K_Mag_k*PZZ_Mag_k_ksub1*K_Mag_k';
    P_k = P2diagP(P_k);
    
    
    
    
    
    
    
    
    
    
    
    
    