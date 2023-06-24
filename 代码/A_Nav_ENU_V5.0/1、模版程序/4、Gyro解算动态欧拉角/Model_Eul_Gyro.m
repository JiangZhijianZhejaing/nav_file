%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：陀螺仪解算姿态欧拉角
%
% By Taoran Zhao
% 2023/04/27 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ###############一、数据导入及参数设定#############################
% ----------1、导入数据----------
File_Name = 'marg1';                   % 读取文件
CalibrationParameter_No1;              % 加载1号模块标定参数
% ----------2、采样率----------
SAMPLE_FREQUENCY = 100;                % 100Hz        
% ----------3、是否作图----------
BOOL_Plot_Animation3d = 0;
BOOL_Plot_Eul_AccMag_Set = 1;
BOOL_Plot_Eul_Gyro_Set = 1;
% ########################################################################
%% ---------------二、标定数据---------------
% 角速度阈值内归零处理
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.R);
%% ---------------三、转换东北天坐标系---------------
% 加速度、磁归一化处理
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuNorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); 
%% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
%% ---------------五、四元数更新初始化---------------
Sample_Interval = 1/SAMPLE_FREQUENCY;            % 采样间隔
Eul_Init = mean(Eul_AccMag_Set(1:300,:));        % 取第1到第300欧拉角均值为初始姿态欧拉角
Qnb_Init = Eul2Qnb(Eul_Init);                    % 初始姿态四元数
Qnb_ksub1 = Qnb_Init;                            % 初始化四元数更新
Eul_Gyro_Set = zeros(Marg_Number,3);             % 提前分配Eul_Gyro_Set内存
%% ---------------六、四元数更新解算动态姿态欧拉角---------------
for k = 1:Marg_Number
    W_Deg_ksub1 = Gyro_Set(k,:);
    Qnb_k = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);      % 一阶龙格库塔法(内涵归一化)
    Eul_Gyro = Qnb2Eul(Qnb_k);
    Eul_Gyro_Set(k,:) = Eul_Gyro;
    Qnb_ksub1 = Qnb_k;
end
% ---------------六、作图---------------
if BOOL_Plot_Animation3d == 1
Animation3dFLyModel(Eul_Gyro_Set, 1);
end
if BOOL_Plot_Eul_AccMag_Set == 1
figure(2)
subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_AccMag_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('姿态角量测');
end
if BOOL_Plot_Eul_Gyro_Set == 1
figure(3)
subplot(3,1,1),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_Gyro_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_Gyro_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number)/SAMPLE_FREQUENCY,Eul_Gyro_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('纯陀螺仪解算姿态角');
end
