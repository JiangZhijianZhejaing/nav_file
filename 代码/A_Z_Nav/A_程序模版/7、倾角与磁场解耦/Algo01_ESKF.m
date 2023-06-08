%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：运用误差状态Kalman滤波融合动静态姿态角(低成本版)
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、标定数据并计算静态姿态角---------------
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入主文件夹所有m文件
CalibParm_No1;                  % 加载1号模块标定参数
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);

% ---------------二、ESKF初始化---------------
Sample_Interval = 0.01;                                 % Gyro采样间隔
% ----------1、初始avp----------
Eul_AccMag_0 = mean(Eul_AccMag_Set(1:300,:));           % 取前3秒均值为初始姿态欧拉角
Qnb_0 = Eul2Qnb(Eul_AccMag_0);                          % 初始姿态四元数
Qnb_ksub1 = Qnb_0;                                      % k-1时刻 Qnb
Qnb_Set = zeros(4,Marg_Number);                         % 预分配内存
Eul_Set = zeros(Marg_Number,3);
% ----------2、kalman初始化----------
X_ksub1 = zeros(6,1);
P_ksub1 = eye(numel(X_ksub1));
Q_ksub1 = (1e-9)*eye(numel(X_ksub1));                   % 状态噪声方差阵
R_ksub1 = (1e-1)*eye(3);                                % 加速度计量测噪声方差阵
H_k = [eye(3),zeros(3,3)];
Beta_ksub1 = 1;
b = 0.95;

%% ---------------三、ESKF--------------
for k = 1:Marg_Number
    % ----------1、姿态更新----------
    if k <= 2
        % 前两次一阶Rungekuta
        Wxyz_ksub1 = Gyro_Set(k,:)*pi/180;% 转弧度
        Wx = Wxyz_ksub1(1); Wy = Wxyz_ksub1(2); Wz = Wxyz_ksub1(3);   
        Qnb_k_0 = (eye(4)+0.5*Sample_Interval*[0,   -Wx,  -Wy,  -Wz; Wx,  0,    Wz,   -Wy; Wy,  -Wz,  0,    Wx; Wz,  Wy,   -Wx,  0 ])*Qnb_ksub1;  
    else
        % 后面四阶Rungekuta
        Qnb_ksub2 = Qnb_Set(:, k - 2);
        Wxyz_ksub1_2_ksub3_Set = Gyro_Set(k - 2 : k, :);
        T = 2*Sample_Interval;
        Wxyz1 = Wxyz_ksub1_2_ksub3_Set(1,:)*pi/180;
        Wxyz2 = Wxyz_ksub1_2_ksub3_Set(2,:)*pi/180;
        Wxyz3 = Wxyz_ksub1_2_ksub3_Set(3,:)*pi/180;
        Wx1 = Wxyz1(1); Wy1 = Wxyz1(2); Wz1 = Wxyz1(3);    % t
        Wx2 = Wxyz2(1); Wy2 = Wxyz2(2); Wz2 = Wxyz2(3);    % t + 1/2T 
        Wx3 = Wxyz3(1); Wy3 = Wxyz3(2); Wz3 = Wxyz3(3);    % t + T
        k1 = 1/2*[0,   -Wx1,  -Wy1,  -Wz1; Wx1,  0,    Wz1,   -Wy1; Wy1,  -Wz1,  0,    Wx1; Wz1,  Wy1,   -Wx1,  0   ]*Qnb_ksub2;
        k2 = 1/2*[0,   -Wx2,  -Wy2,  -Wz2; Wx2,  0,    Wz2,   -Wy2; Wy2,  -Wz2,  0,    Wx2; Wz2,  Wy2,   -Wx2,  0   ]*(Qnb_ksub2 + k1/2);
        k3 = 1/2*[0,   -Wx2,  -Wy2,  -Wz2; Wx2,  0,    Wz2,   -Wy2; Wy2,  -Wz2,  0,    Wx2; Wz2,  Wy2,   -Wx2,  0   ]*(Qnb_ksub2 + k2/2);
        k4 = 1/2*[0,   -Wx3,  -Wy3,  -Wz3; Wx3,  0,    Wz3,   -Wy3; Wy3,  -Wz3,  0,    Wx3; Wz3,  Wy3,   -Wx3,  0   ]*(Qnb_ksub2 + k3);
        Qnb_k_0 = Qnb_ksub2 + T/6*(k1 + 2*k2 + 2*k3 + k4);  % 2时刻前  k-2
    end
    Qnb_k_0 = NormlzQnb(Qnb_k_0);                                          % k时刻 未校正的Qnb
    Eul_k_0 = Qnb2Eul(Qnb_k_0);                                            % k时刻 未校正的姿态欧拉角
    % ----------2、eskf：得到修正值----------
    Eul_AccMag = Eul_AccMag_Set(k,:)';              % k时刻 静态姿态欧拉角

    Eul_ksub1 = Qnb2Eul(Qnb_ksub1);                 % k-1时刻 姿态欧拉角(角度)
    Eul_Rad_ksub1 = Eul_ksub1*pi/180;               % k-1时刻 姿态欧拉角(弧度)
    Pitch = Eul_Rad_ksub1(1);  
    Yaw = Eul_Rad_ksub1(3);
    Cnb_ksub1 = Qnb2Cnb(Qnb_ksub1);                 % k-1时刻 Cnb
    Mre = [-cos(Yaw) -sin(Yaw) 0; sin(Yaw)/cos(Pitch) -cos(Yaw)/cos(Pitch) 0; -tan(Pitch)*sin(Yaw) tan(Pitch)*cos(Yaw) -1];    % A = Mr2e*R
    % -----(1)PHI_k_ksub1-----
    Ma1 = Mre*(-Cnb_ksub1);
    %                A            eb(or1)            
    phi_k_ksub1 = [  zeros(3),    Ma1;           % A
                     zeros(3),    zeros(3)];     % eb(or1)                
    PHI_k_ksub1 = eye(6) + phi_k_ksub1*Sample_Interval;                    % 离散化
    % ----------3、Kalman Filter----------
    % -----(1)状态一步预测-----
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;
    % -----(2)状态一步预测均方误差阵-----
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);
    % -----(3)自适应估计R_k-----
    Z_k = Eul_k_0 - Eul_AccMag;
    if abs(Z_k(3)) > 90
        Z_k(3) = X_k_ksub1(3);
    end
    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1; 
    Beta_k = Beta_ksub1/(Beta_ksub1 + b);
    R_k = (1 - Beta_k)*R_ksub1 + Beta_k*(Z_error_k_ksub1*Z_error_k_ksub1' - H_k*P_k_ksub1*H_k');
    % -----(4)滤波增益-----
    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_k;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    % -----(5)状态估计-----
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    % -----(6)状态估计均方误差阵-----
    P_k = (eye(6) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    % ----------3、avp修正值----------
    Eul_k = Eul_k_0 - X_k(1:3);             % 修正姿态欧拉角
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    Qnb_Set(:,k) = Qnb_k;
    % ----------4、状态归0----------
    X_k(1:3) = 0;
    % ----------5、参数迭代----------
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    Qnb_ksub1 = Qnb_k;
end
%% ----------三、画图----------
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('俯仰角 Pitch'),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\theta');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('横滚角 Roll '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\gamma');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\psi');
sgtitle('姿态角量测');
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('俯仰角 Pitch'),xlabel('时间 t/s'),ylabel('A/°'),legend('\theta');
subplot(3,1,2),plot(Eul_Set(:,2)),title('横滚角 Roll '),xlabel('时间 t/s'),ylabel('A/°'),legend('\gamma');
subplot(3,1,3),plot(Eul_Set(:,3)),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('A/°'),legend('\psi');
sgtitle('姿态角');



