%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：陀螺仪解算姿态欧拉角
%
% By Taoran Zhao
% 2023/04/27 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、导入数据---------------
clc;                                   % 清理命令行
clear;                                 % 清理工作区
addpath(genpath('../../'));            % 导入系统文件夹所有文件

format long;
Marg_Raw_Set = xlsread('水平', 'sheet1');             % 导入惯性传感器原始数据文件
Marg_Number = size(Marg_Raw_Set,1);                   % MARG采样数
for k = 1:Marg_Number
    a = mod(k,1);
    if a == 0
        Marg_Raw_Set_new(k/1,:) = Marg_Raw_Set(k,:);
    end
end


Marg_Number = size(Marg_Raw_Set_new,1);
Gyro_Set = Marg_Raw_Set_new(:,4:6);                       % 陀螺仪原始数据集
for k = 1:Marg_Number
    if k >= 6
        Gyro_Set(k,1) = mean(Gyro_Set(k-5:k,1));
        Gyro_Set(k,2) = mean(Gyro_Set(k-5:k,2));
        Gyro_Set(k,3) = mean(Gyro_Set(k-5:k,3));
    end
end




Acc_Set = Marg_Raw_Set_new(:,1:3);                         % 加速度计原始数据集 
for k = 1:Marg_Number
    if k >= 6
        Acc_Set(k,1) = mean(Acc_Set(k-5:k,1));
        Acc_Set(k,2) = mean(Acc_Set(k-5:k,2));
        Acc_Set(k,3) = mean(Acc_Set(k-5:k,3));
    end
end




Mag_Set = Marg_Raw_Set(:,7:9);                         % 磁传感器原始数据集


% Data1 = Gyro_Set(:,1)';
% Data2 = Gyro_Set(:,2)';
% Data3 = Gyro_Set(:,3)';
% Gyro_Set(:,1) = Fft_IdealLowPassFilter1(Data1, 8);   
% Gyro_Set(:,2) = Fft_IdealLowPassFilter1(Data2, 8);   
% Gyro_Set(:,3) = Fft_IdealLowPassFilter1(Data3, 8);  

% Acc_Set(:,2) = -Acc_Set(:,2);

%% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
%% ---------------五、四元数更新初始化---------------
Sample_Interval = 0.01;                          % Gyro采样间隔
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
% animation3D_FLyModel(Eul_Gyro_Set, 1);
figure(1)
subplot(3,1,1),plot((1:Marg_Number),Eul_AccMag_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number),Eul_AccMag_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number),Eul_AccMag_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('姿态角量测');
figure(2)
subplot(3,1,1),plot((1:Marg_Number),Eul_Gyro_Set(:,1)),title('俯仰角 Pitch \theta'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\theta');
subplot(3,1,2),plot((1:Marg_Number),Eul_Gyro_Set(:,2)),title('横滚角 Roll \gamma'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\gamma');
subplot(3,1,3),plot((1:Marg_Number),Eul_Gyro_Set(:,3)),title('航向角 Yaw \psi'),xlabel('时间t/(s)'),ylabel('角度A/(°)'),legend('\psi');
sgtitle('纯陀螺仪解算姿态角');
