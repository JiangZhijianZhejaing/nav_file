%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：FFT频谱分析+低通滤波
%
% By Taoran Zhao
% 2023/05/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ###############一、数据导入及参数设定#############################
% ----------1、导入数据----------
File_Name = 'marg1';                   % 读取文件
CalibrationParameter_No1;              % 加载1号模块标定参数
% ########################################################################
%% ---------------二、标定数据---------------
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.R);
%% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS);    % 加速度、磁场矢量归一化
Data = Gyro_Set(:,1)';
Data_new = Fft_IdealLowPassFilter(Data, 3);    % 截止频率设3Hz