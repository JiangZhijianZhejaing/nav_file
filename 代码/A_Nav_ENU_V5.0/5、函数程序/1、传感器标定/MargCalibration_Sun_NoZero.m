function [Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = MargCalibration_Sun_NoZero(File_Name, Sheet_Name, Wp, P, Mb, B)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：MARG传感器标定，输出实验室模块坐标系下的Marg数据
%
% Prototype: [Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = MargCalibration_Sun_NoZero(File_Name, Sheet_Name, Wp, P, Mb, B)
% Inputs: File_Name - 文件名
%         Sheet_Name - 表单名
%         Wp - 陀螺仪标定矩阵(零偏取零)
%         P - 加速度计标定矩阵
%         Mb - 磁传感器标定矩阵
%         B - 零偏
% Output: Gyro_Set_InitCS - 原坐标系角速度集
%         Acc_Set_InitCS - 原坐标系加速度集
%         Mag_Set_InitCS - 原坐标系磁场矢量集
%         Marg_Number - 采样数
%
% Notice：详见《储志伟.基于MARG传感器的微型航姿系统[D].中国科学技术大学,2018.》第四章
%
% By Taoran Zhao
% 2023/04/26 2023/05/06 2023/05/19 2023/05/26
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、导入原始数据---------------
format long;
Marg_Raw_Set = xlsread(File_Name, Sheet_Name);    % 导入惯性传感器原始数据文件
Marg_Number = size(Marg_Raw_Set,1);               % MARG采样数
Gyro_Raw_Set = Marg_Raw_Set(:,1:3);               % 陀螺仪原始数据集
Acc_Raw_Set = Marg_Raw_Set(:,4:6);                % 加速度计原始数据集
Mag_Raw_Set = Marg_Raw_Set(:,7:9);                % 磁传感器原始数据集
%% ---------------二、传感器标定---------------
% ----------1、陀螺仪标定----------
% 标定后，前3秒静置，扣除随机误差(每次启动后不变，但每次启动不是固定值)
Gyro_offset = mean(Gyro_Raw_Set(1:300,:));                 % 零偏，取300个平均
Gyro_Raw_Set_0 = Gyro_Raw_Set - Gyro_offset;               % 扣除每次开机的零偏
Gyro_Raw_Set_add1 = horzcat(Gyro_Raw_Set_0, ones(Marg_Number, 1));
Gyro_Set_InitCS = (Gyro_Raw_Set_add1*Wp);                  % 陀螺仪实际输出值（deg/s），依次是绕x轴角速率、绕y轴角速率、绕z轴角速率
% ----------2、加速度计标定----------
Acc_Raw_set_add1 = horzcat(Acc_Raw_Set, ones(Marg_Number, 1));                 % 加一列1
Acc_Set_InitCS = Acc_Raw_set_add1*P;                       % 三轴加速度（单位化）
% ----------3、磁传感器标定----------
% -----(1)Mb R-----
% headingData = (Mb*Mag_Raw_Set')';
% Mag_Set_InitCS = [headingData(:,1) - R(1), headingData(:,2) - R(2), headingData(:,3) - R(3)];                % 三轴磁场矢量
% -----(1)Mb B-----
Mag_Raw_Set_0 = [Mag_Raw_Set(:,1) - B(1), Mag_Raw_Set(:,2) - B(2), Mag_Raw_Set(:,3) - B(3)];
Mag_Set_InitCS = Mag_Raw_Set_0*Mb;










