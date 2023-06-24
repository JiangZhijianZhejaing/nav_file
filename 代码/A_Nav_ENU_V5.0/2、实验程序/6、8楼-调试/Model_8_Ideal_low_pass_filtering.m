%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：FFT频谱分析+低通滤波
%
% By Taoran Zhao
% 2023/05/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、导入数据---------------
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
format long;
Marg_Raw_Set = xlsread('往复', 'sheet1');             % 导入惯性传感器原始数据文件
Marg_Number = size(Marg_Raw_Set,1);                        % MARG采样数
Gyro_Set = Marg_Raw_Set(:,4:6);                        % 陀螺仪原始数据集
Gyro_Set(:,2) = -Gyro_Set(:,2);
Acc_Set = Marg_Raw_Set(:,1:3);                         % 加速度计原始数据集
Acc_Set(:,2) = -Acc_Set(:,2); 
Mag_Set = Marg_Raw_Set(:,7:9);                         % 磁传感器原始数据集
Acc_Set(:,2) = -Acc_Set(:,2);


% % CalibParm_No1;                         % 加载1号模块标定参数
% %% ---------------二、标定数据---------------
% [Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
%     MargCalib_NoZero('MARG1', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% %% ---------------三、转换东北天坐标系---------------
% [Gyro_Set, Acc_Set, Mag_Set] = MargENU_UnNorm(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);
Data = Gyro_Set(:,2)';
Data_new = Fft_IdealLowPassFilter1(Data, 25);    % 截止频率设5Hz