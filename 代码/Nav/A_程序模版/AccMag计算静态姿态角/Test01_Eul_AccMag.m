%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：标定传感器及计算静态姿态角
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(genpath('../../'));     % 导入主文件夹所有m文件
clc;                            % 清理命令行
clear;                          % 清理工作区
gvar;                           % 加载地球参数
CalibParm_No1;                  % 加载1号模块标定参数
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
%%
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');