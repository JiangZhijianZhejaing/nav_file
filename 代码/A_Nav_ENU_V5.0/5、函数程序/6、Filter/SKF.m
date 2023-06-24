function [X_k, P_k] = SKF(Z_k, X_k_ksub1, R_ksub1, H_k, P_k_ksub1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：序贯KF估计R_k
%
% Prototype: [X_k, P_k, Beta_k, R_k] = ASKF(Z_k, X_k_ksub1, R_ksub1, Beta_ksub1, H_k, P_k_ksub1)
% Inputs:  Z_k - 量测列向量
%          X_k_ksub1 - k时刻 状态预测
%          R_ksub1 - k-1时刻 量测噪声
%          Beta_ksub1 - k-1时刻 β
%          H_k - 量测矩阵
% Output:  X_k - 状态估计
%          P_k - 协方差估计
%          Beta_k - k时刻 β
%          R_k - k时刻 量测噪声
%
% By Taoran Zhao
% 2023/05/22
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、初始化---------------
n = size(R_ksub1,2);
Xn_k_ksub1 = X_k_ksub1; % 第n次序贯处理的状态预测
Pn_k_ksub1 = P_k_ksub1; % 第n次序贯处理的协方差预测
%% ---------------二、序贯处理---------------
for k = 1:n
    % -----1、自适应估计R_k(k,k)-----    
    Hn_k = H_k(k,:);
    Zn_k = Z_k(k);
    Zn_k_ksub1 = Hn_k*Xn_k_ksub1;
    Zn_error_k_ksub1 = Zn_k - Zn_k_ksub1;                      % 量测误差
    % -----2、解算滤波增益-----
    PXZ = Pn_k_ksub1*Hn_k';
    PZZ = Hn_k*PXZ + R_ksub1(k,k);
    Kn_k = PXZ/PZZ;
    % -----3、状态估计-----
    Xn_k = Xn_k_ksub1 + Kn_k*(Zn_error_k_ksub1);
    % -----4、均方误差阵估计-----
    Pn_k = (eye(3) - Kn_k*Hn_k)*Pn_k_ksub1;
    % -----5、迭代更新-----
    Xn_k_ksub1 = Xn_k;
    Pn_k_ksub1 = Pn_k;
end
X_k = Xn_k;
P_k = Pn_k;
P_k = P2diagP(P_k);


