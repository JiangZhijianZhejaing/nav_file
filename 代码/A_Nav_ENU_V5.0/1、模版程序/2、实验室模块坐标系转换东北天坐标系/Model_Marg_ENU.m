%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：实验室坐标系转东北天坐标系
%
% By Taoran Zhao
% 2023/04/02 2023/04/27 2023/06/08
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ###############一、数据导入及参数设定#############################
% ----------1、导入数据----------
File_Name = 'marg1';                   % 读取文件
CalibrationParameter_No1;              % 加载1号模块标定参数
% ----------2、采样率----------
SAMPLE_FREQUENCY = 100;                            % 100Hz    
% ########################################################################
%% ---------------二、标定数据---------------
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.R);
%% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS);      % 加速度、磁归一化
%% ---------------四、作图---------------
figure(1)
subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Gyro_Set(:,1)),title('Wx'),xlabel('时间t/(s)'),ylabel('角度每秒/(°/s ) '),legend('Wx');   
subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Gyro_Set(:,2)),title('Wy'),xlabel('时间t/(s)'),ylabel('角度每秒/(°/s ) '),legend('Wy');
subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Gyro_Set(:,3)),title('Wz'),xlabel('时间t/(s)'),ylabel('角度每秒/(°/s ) '),legend('Wz');
sgtitle('角速度数据');
figure(2)
subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Acc_Set(:,1)),title('Fbx'),xlabel('时间t/(s)'),ylabel('g'),legend('Fbx');
subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Acc_Set(:,2)),title('Fby'),xlabel('时间t/(s)'),ylabel('g'),legend('Fby');
subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Acc_Set(:,3)),title('Fbz'),xlabel('时间t/(s)'),ylabel('g'),legend('Fbz');
sgtitle('加速度数据');
figure(3)
subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Mag_Set(:,1)),title('Mbx'),xlabel('时间t/(s)'),ylabel('Mbx '),legend('Mbx');
subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Mag_Set(:,2)),title('Mby'),xlabel('时间t/(s)'),ylabel('Mby '),legend('Mby');
subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Mag_Set(:,3)),title('Mbz'),xlabel('时间t/(s)'),ylabel('Mbz '),legend('Mbz');
sgtitle('磁场矢量数据');