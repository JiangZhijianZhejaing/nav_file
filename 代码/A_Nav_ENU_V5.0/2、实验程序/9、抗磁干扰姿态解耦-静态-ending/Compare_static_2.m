%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：静态对比
%
% By Taoran Zhao
% 2023/05/06 2023/05/18 2023/05/23
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ###############一、数据导入及参数设定#############################
% ----------1、导入数据----------
File_Name = '20230606_static';         % 读取文件
CalibrationParameter_No1;              % 加载1号模块标定参数
% ----------2、采样率----------
SAMPLE_FREQUENCY = 100;                % 100Hz   
% ----------3、SAESKF参数----------
Q_ksub1 =  (1e-7)*eye(3);              % 状态噪声方差阵
R_ksub1 = diag([1e-1,1e-1,1e-1]);      % 量测噪声方差阵
% ----------4、TSAEKF参数----------
Q_ksub1_A = (1e-7)*eye(4);                      % 状态噪声方差阵
Q_ksub1_M = (1e-11)*eye(4);
R_Acc_ksub1 = (1e-1)*eye(3);                     % 加速度计量测噪声方差阵
R_Mag_ksub1 = (1e-1)*eye(3);                     % 磁传感器量测噪声方差阵
% ----------5、TACF参数----------
xa = 1e-2;
xm = 1e-5;
% ########################################################################
% ---------------二、标定数据---------------
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun_NoZero(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
% 角速度阈值内未归零
% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuUnnorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); % 加速度、磁数据取消归一化   
% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% ---------------五、三种姿态解耦估计算法---------------
Eul_SAESKF_Set = SAESKF_Fun(File_Name, Q_ksub1, R_ksub1);
Eul_TSAEKF_Set = TSAEKF_Fun(File_Name, Q_ksub1_A, Q_ksub1_M, R_Acc_ksub1, R_Mag_ksub1);
Eul_TACF_Set = TACF_Fun(File_Name,xa,xm);
% ---------------六、对比分析---------------
Eul_Init = mean(Eul_AccMag_Set(1:300,:));       % 初始姿态欧拉角，取第50个到第200个欧拉角平均
Eul_Reference_Set = zeros(Marg_Number,3);
for i = 1:Marg_Number
    Eul_Reference_Set(i,:) = Eul_Init;
end
Pitch1_MSE0 = 0;
for k = 1:Marg_Number
   Pitch1_E =  Eul_SAESKF_Set(k,1) - Eul_Init(1);
   Pitch1_SE = Pitch1_E^2;
   Pitch1_MSE0 = Pitch1_MSE0 + Pitch1_SE;
end
Pitch1_MSE = Pitch1_MSE0/Marg_Number;
Pitch1_RMSE = sqrt(Pitch1_MSE);
Pitch2_MSE0 = 0;
for k = 1:Marg_Number
   Pitch2_E =  Eul_TSAEKF_Set(k,1) - Eul_Init(1);
   Pitch2_SE = Pitch2_E^2;
   Pitch2_MSE0 = Pitch2_MSE0 + Pitch2_SE;
end
Pitch2_MSE = Pitch2_MSE0/Marg_Number;
Pitch2_RMSE = sqrt(Pitch2_MSE);
Pitch3_MSE0 = 0;
for k = 1:Marg_Number
   Pitch3_E =  Eul_TACF_Set(k,1) - Eul_Init(1);
   Pitch3_SE = Pitch3_E^2;
   Pitch3_MSE0 = Pitch3_MSE0 + Pitch3_SE;
end
Pitch3_MSE = Pitch3_MSE0/Marg_Number;
Pitch3_RMSE = sqrt(Pitch3_MSE);
Pitch_RMSE = [Pitch1_RMSE;Pitch2_RMSE;Pitch3_RMSE];

Roll1_MSE0 = 0;
for k = 1:Marg_Number
   Roll1_E =  Eul_SAESKF_Set(k,2) - Eul_Init(2);
   Roll1_SE = Roll1_E^2;
   Roll1_MSE0 = Roll1_MSE0 + Roll1_SE;
end
Roll1_MSE = Roll1_MSE0/Marg_Number;
Roll1_RMSE = sqrt(Roll1_MSE);
Roll2_MSE0 = 0;
for k = 1:Marg_Number
   Roll2_E =  Eul_TSAEKF_Set(k,2) - Eul_Init(2);
   Roll2_SE = Roll2_E^2;
   Roll2_MSE0 = Roll2_MSE0 + Roll2_SE;
end
Roll2_MSE = Roll2_MSE0/Marg_Number;
Roll2_RMSE = sqrt(Roll2_MSE);
Roll3_MSE0 = 0;
for k = 1:Marg_Number
   Roll3_E =  Eul_TACF_Set(k,2) - Eul_Init(2);
   Roll3_SE = Roll3_E^2;
   Roll3_MSE0 = Roll3_MSE0 + Roll3_SE;
end
Roll3_MSE = Roll3_MSE0/Marg_Number;
Roll3_RMSE = sqrt(Roll3_MSE);
Roll_RMSE = [Roll1_RMSE;Roll2_RMSE;Roll3_RMSE];
Yaw1_MSE0 = 0;
for k = 1:Marg_Number
   Yaw1_E =  Eul_SAESKF_Set(k,3) - Eul_Init(3);
   Yaw1_SE = Yaw1_E^2;
   Yaw1_MSE0 = Yaw1_MSE0 + Yaw1_SE;
end
Yaw1_MSE = Yaw1_MSE0/Marg_Number;
Yaw1_RMSE = sqrt(Yaw1_MSE);
Yaw2_MSE0 = 0;
for k = 1:Marg_Number
   Yaw2_E =  Eul_TSAEKF_Set(k,3) - Eul_Init(3);
   Yaw2_SE = Yaw2_E^2;
   Yaw2_MSE0 = Yaw2_MSE0 + Yaw2_SE;
end
Yaw2_MSE = Yaw2_MSE0/Marg_Number;
Yaw2_RMSE = sqrt(Yaw2_MSE);
Yaw3_MSE0 = 0;
for k = 1:Marg_Number
   Yaw3_E =  Eul_TACF_Set(k,3) - Eul_Init(3);
   Yaw3_SE = Yaw3_E^2;
   Yaw3_MSE0 = Yaw3_MSE0 + Yaw3_SE;
end
Yaw3_MSE = Yaw3_MSE0/Marg_Number;
Yaw3_RMSE = sqrt(Yaw3_MSE);
Yaw4_MSE0 = 0;
for k = 1:Marg_Number
   Yaw4_E =  Eul_AccMag_Set(k,3) - Eul_Init(3);
   Yaw4_SE = Yaw4_E^2;
   Yaw4_MSE0 = Yaw4_MSE0 + Yaw4_SE;
end
Yaw4_MSE = Yaw4_MSE0/Marg_Number;
Yaw4_RMSE = sqrt(Yaw4_MSE);
Percentage_SAESKF = 1-Yaw1_RMSE/Yaw4_RMSE;
Percentage_TSAEKF = 1-Yaw2_RMSE/Yaw4_RMSE;
Percentage_ATCF = 1-Yaw3_RMSE/Yaw4_RMSE;
Percentage = [Percentage_SAESKF,Percentage_TSAEKF,Percentage_ATCF];
Yaw_RMSE = [Yaw1_RMSE;Yaw2_RMSE;Yaw3_RMSE;];
RMSE = [Pitch_RMSE,Roll_RMSE,Yaw_RMSE];
% ---------------七、对比分析---------------
figure(1)
plot((1:Marg_Number)/100,Eul_TACF_Set(:,3),'g',(1:Marg_Number)/100,Eul_TSAEKF_Set(:,3),'b',(1:Marg_Number)/100,Eul_SAESKF_Set(:,3),'y',(1:Marg_Number)/100,Eul_Reference_Set(:,3),'r','LineWidth',2),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('ψ/°'),legend('TACF','TSAEKF','SAESKF','Reference');
set (gca,'FontSize',13);
figure(2)
plot((1:Marg_Number)/100,Eul_AccMag_Set(:,3),'m','LineWidth',2),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('ψ/°'),legend('Measured');
% ylim([-65,-44]);
set (gca,'FontSize',13);
figure(3)
plot((1:Marg_Number)/100,Eul_AccMag_Set(:,3),'m',(1:Marg_Number)/100,Eul_TACF_Set(:,3),'g','LineWidth',2),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('ψ/°'),legend('Measured');

