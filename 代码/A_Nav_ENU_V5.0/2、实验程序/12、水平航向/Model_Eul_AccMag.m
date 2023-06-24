%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：加速度计、磁传感器解算姿态欧拉角
%
% By Taoran Zhao
% 2023/04/27 2023/04/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                                   % 清理命令行
clear;                                 % 清理工作区
%% ###############零、参数设定#######################################
File_Name1 = '0_00';
File_Name2 = '45_00';
File_Name3 = '90_00';
File_Name4 = '135_00';
File_Name5 = '180_00';
File_Name6 = '225_00';
File_Name7 = '270_00';
File_Name8 = '315_00';
% ----------1、采样率----------
SAMPLE_FREQUENCY = 100;                            % 100Hz          
% ----------2、是否作图----------
BOOL_Plot_Animation3d = false;
BOOL_Plot_Eul_AccMag_Set = false;
% ########################################################################
%% ---------------一、导入数据---------------
addpath(genpath('../../'));            % 导入系统文件夹所有文件
CalibrationParameter_No1;              % 加载1号模块标定参数
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name1, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
%% ---------------四、解算静态姿态欧拉角---------------
% Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
for k = 1:Marg_Number
    Yaw_Set_1(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_1 = mean(Yaw_Set_1);
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name2, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
for k = 1:Marg_Number
    Yaw_Set_2(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_2 = mean(Yaw_Set_2);
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name3, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
for k = 1:Marg_Number
    Yaw_Set_3(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_3 = mean(Yaw_Set_3);
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name4, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
for k = 1:Marg_Number
    Yaw_Set_4(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_4 = mean(Yaw_Set_4);
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name5, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
for k = 1:Marg_Number
    Yaw_Set_5(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_5 = mean(Yaw_Set_5);
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name6, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
for k = 1:Marg_Number
    Yaw_Set_6(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_6 = mean(Yaw_Set_6);
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name7, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
for k = 1:Marg_Number
    Yaw_Set_7(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_7 = mean(Yaw_Set_7);
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun(File_Name8, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
for k = 1:Marg_Number
    Yaw_Set_8(k,:) = YawMag(0, 0, Mag_Set(k,:));
end
Yaw_8 = mean(Yaw_Set_8);


bianma = [0;45;90;135;180;225;270;315;] - 277.45*ones(1,8)';
Yaw_Set = [Yaw_1;Yaw_2;Yaw_3;Yaw_4;Yaw_5;Yaw_6;Yaw_7;Yaw_8];









% % Yaw_Set = Eul_AccMag_Set(:,3);
% %% ---------------五、作图---------------
% if BOOL_Plot_Animation3d == 1
% Animation3dFLyModel(Eul_AccMag_Set, 1);
% end
% if BOOL_Plot_Eul_AccMag_Set == 1
% figure(2)
% subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
% subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
% subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
% sgtitle('姿态角量测');
% end