function [X_k, P_k] = EKF(PHI_k_ksub1,X_ksub1,P_ksub1,Q_ksub1,R_ksub1,Z_k,h_k_Fun,H_k_Fun) 
% -----(1)状态一步预测-----
X_k_ksub1 = PHI_k_ksub1*X_ksub1;
X_k_ksub1 = NormlzQua(X_k_ksub1);
% -----(2)状态一步预测均方误差阵-----
P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
P_k_ksub1 = P2diagP(P_k_ksub1);                    % 对称化
% -----(3)Z_k-----
h_k = h_k_Fun(X_k_ksub1);
H_k = H_k_Fun(X_k_ksub1);
Z_error_k_ksub1 = Z_k - h_k; 
% -----(4)滤波增益-----
PXZ_k_ksub1 = P_k_ksub1*H_k';
PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_ksub1;
K_k = PXZ_k_ksub1/PZZ_k_ksub1;
% -----(5)状态估计-----
X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
X_k = NormlzQua(X_k);
% -----(6)状态估计均方误差阵-----
P_k = (eye(4) - K_k*H_k)*P_k_ksub1;
P_k = P2diagP(P_k);  