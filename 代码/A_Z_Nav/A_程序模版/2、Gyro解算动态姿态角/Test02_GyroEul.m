%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：纯陀螺仪解算姿态
%
% By Taoran Zhao
% 2023/04/03
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入主文件夹所有m文件
CalibParm_No1;                  % 加载1号模块标定参数
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
% -----画图-----
% figure(1)
% subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
% subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
% subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%%
Sample_Interval = 0.01;                                 % Gyro采样间隔
% 初始avp
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % 取前3秒平均为初始姿态欧拉角
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % 初始姿态四元数
Qnb_Set = zeros(4,Marg_Number);
Eul_Set = zeros(Marg_Number,3);
for k = 1:Marg_Number
    Qnb_k = Rungekuta(Qnb_ksub1, Sample_Interval, k, Gyro_Set, Qnb_Set);   % 四阶龙格库塔
    Eul = Qnb2Eul(Qnb_k);
    Qnb_Set(:,k) = Qnb_k;
    Eul_Set(k,:) = Eul;
    Qnb_ksub1 = Qnb_k;
end
% -----画图-----
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');
