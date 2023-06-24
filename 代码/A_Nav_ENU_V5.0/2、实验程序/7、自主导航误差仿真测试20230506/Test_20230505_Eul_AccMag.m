%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：加速度计、磁传感器解算姿态欧拉角
%
% By Taoran Zhao
% 2023/04/27 2023/04/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------一、导入数据---------------
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入系统文件夹所有文件
CalibParm_Z_No1_20230505;                    
% ---------------二、标定数据---------------
[Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
    MargCalib('0505', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargENU(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);
% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% ---------------五、作图---------------
% animation3D_FLyModel(Eul_AccMag_Set, 1);
k = Marg_Number;
figure(2)
subplot(3,1,1),plot((1:k)/100,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch'),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\theta');
subplot(3,1,2),plot((1:k)/100,Eul_AccMag_Set(:,2)),title('横滚角 Roll '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\gamma');
subplot(3,1,3),plot((1:k)/100,Eul_AccMag_Set(:,3)),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\psi');
sgtitle('姿态角量测');