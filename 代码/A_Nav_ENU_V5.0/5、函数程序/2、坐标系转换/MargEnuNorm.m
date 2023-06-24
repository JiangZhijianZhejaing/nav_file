function [Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：Marg数据转换到东北天坐标系，归一
%
% Prototype: [Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS)
% Inputs: Gyro_Set_InitCS - 原坐标系角速度集
%         Acc_Set_InitCS - 原坐标系加速度集（归一）
%         Mag_Set_InitCS - 原坐标系磁矢量集（归一）
% Output: Gyro_Set - ENU角速度集
%         Acc_Set - ENU加速度集
%         Mag_Set - ENU磁矢量集
%
% By Taoran Zhao
% 2023/04/26 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、坐标系转换---------------
% ----------1、角速度----------
Gyro_Set(:,1) = -Gyro_Set_InitCS(:,2);       % Wx     X与Y互换
Gyro_Set(:,2) = -Gyro_Set_InitCS(:,1);       % Wy
Gyro_Set(:,3) = Gyro_Set_InitCS(:,3);        % Wz
% ----------2、加速度----------
Acc_Set(:,1) = -Acc_Set_InitCS(:,2);         % fbx    X与Y互换
Acc_Set(:,2) = Acc_Set_InitCS(:,1);          % fby
Acc_Set(:,3) = Acc_Set_InitCS(:,3);          % fbz
% ----------3、磁矢量----------
Mag_Set(:,1) = Mag_Set_InitCS(:,2);           % mbx    X与Y互换
Mag_Set(:,2) = Mag_Set_InitCS(:,1);           % mby
Mag_Set(:,3) = -Mag_Set_InitCS(:,3);          % mbz
%% ---------------二、加速度、磁矢量归一化---------------
% 2023/05/19
Marg_Number = size(Acc_Set,1);
for k = 1: Marg_Number
    Acc = Acc_Set(k,:)'; 
    Acc_Set(k,:) = NormlzV3(Acc)';
end
for k = 1: Marg_Number
    Mag = Mag_Set(k,:)'; 
    Mag_Set(k,:) = NormlzV3(Mag)';
end
    