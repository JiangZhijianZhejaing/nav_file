%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：陀螺仪解算姿态欧拉角
%
% By Taoran Zhao
% 2023/04/27 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------一、导入数据---------------
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入系统文件夹所有文件
CalibParm_No1;         % 加载1号模块标定参数
% ---------------二、标定数据---------------
[Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
    MargCalib_NoZero('0531_Roll', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargENU_UnNorm(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);
% 未归一化
% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% ---------------五、解算动态姿态欧拉角---------------
Sample_Interval = 0.01;                          % Gyro采样间隔
Eul_Init = mean(Eul_AccMag_Set(50:200,:));       % 取第50个到第200个欧拉角平均为初始姿态欧拉角
Qnb_ksub1 = Eul2Qnb(Eul_Init);                   % 初始姿态四元数
Eul_Gyro_Set = zeros(Marg_Number,3);             % 提前分配Eul_Gyro_Set内存
for k = 1:Marg_Number
    W_Deg_ksub1 = Gyro_Set(k,:);
    Qnb_k = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);      % 一阶龙格库塔法(内涵归一化)
    Eul_Gyro = Qnb2Eul(Qnb_k);
    Eul_Gyro_Set(k,:) = Eul_Gyro;
    Qnb_ksub1 = Qnb_k;
end
% ---------------六、作图---------------
% animation3D_FLyModel(Eul_Gyro_Set, 1);
figure(2)
subplot(3,1,1),plot((1:Marg_Number)/100,Eul_Gyro_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/100,Eul_Gyro_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/100,Eul_Gyro_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('纯陀螺仪解算姿态角');
