function [Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul(File_Name, Sheet_Name, Wp, p, mb, R)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：1、MARG传感器标定 
%           2、计算静态欧拉角(角度)
%
% Prototype: [Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Deg_Set] = IDAndCamEul(File_Name, Sheet_Name, Wp, p, mb, R)
% Inputs: File_Name - 文件名
%         Sheet_Name - 表单名
%         Wp - 陀螺仪标定矩阵
%         p - 加速度计标定矩阵
%         mb - 磁传感器标定矩阵
%         R - 椭球中心
% Output: Gyro_Set - 角速度集
%         Acc_Set - 加速度集
%         Mag_Set - 磁矢量集
%         Marg_Number - 采样数
%         Eul_AccMag_Deg_Set - 静态姿态角集
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    format long;
    Marg_Raw_Set = xlsread(File_Name, Sheet_Name);          % 导入惯性传感器原始数据
    Marg_Number = size(Marg_Raw_Set,1);                     % MARG采样数
    Gyro_Raw_Set = Marg_Raw_Set(:,1:3);                     % 陀螺仪原始数据集
    Acc_Raw_Set = Marg_Raw_Set(:,4:6);                      % 加速度计原始数据集（X轴加速度原始数据的正负性与YZ相反，后期处理需加负号）
    Mag_Raw_Set = Marg_Raw_Set(:,7:9);                      % 磁传感器原始数据集

    %% 陀螺仪标定
    Gyro_offset = mean(Gyro_Raw_Set(1:300,:));              % 随机误差，每次启动后不变，但每次启动不是固定值
    Gyro_Raw_Set = Gyro_Raw_Set - Gyro_offset;              % 扣除每次开机的随机误差
    GyroMax = max(Gyro_Raw_Set(1:300,:));
    GyroMin = min(Gyro_Raw_Set(1:300,:));
    for i = 1 : Marg_Number
        if Gyro_Raw_Set(i,1) <= GyroMax(1) && Gyro_Raw_Set(i,1) >= GyroMin(1)
           Gyro_Raw_Set(i,1) = 0; 
        end
        if Gyro_Raw_Set(i,2) <= GyroMax(2) && Gyro_Raw_Set(i,2) >= GyroMin(2)
           Gyro_Raw_Set(i,2) = 0; 
        end
        if Gyro_Raw_Set(i,3) <= GyroMax(3) && Gyro_Raw_Set(i,3) >= GyroMin(3)
           Gyro_Raw_Set(i,3) = 0; 
        end      
    end
    Gyro_Set_0 = (Gyro_Raw_Set*Wp);         % 陀螺仪实际输出值（deg/s），依次是绕x轴角速率、绕y轴角速率、绕z轴角速率
    Gyro_Set(:,1) = -Gyro_Set_0(:,2);       % Wx    X与Y互换
    Gyro_Set(:,2) = -Gyro_Set_0(:,1);       % Wy
    Gyro_Set(:,3) = Gyro_Set_0(:,3);        % Wz    Z*(-1)
    
    %% 加速度计标定
    Acc_Raw_set_add1 = horzcat(Acc_Raw_Set, ones(Marg_Number, 1));  % 加一列1
    % p =[   0.002903387627154  -0.000060755386809   0.000000894230601			
    %   -0.000031834924067   0.002946774674143   0.000004641374621			
    %    0.000003733332459  -0.000009064807629   0.002855073329104			
    %   -0.001121527428827  -0.004231396444889  -0.081810863260599];             % 加速度计标定参数
    Acc_Set_0 = Acc_Raw_set_add1*p;  % 三轴加速度（归一化）
    Acc_Set(:,1) = -Acc_Set_0(:,2);  % fbx    X与Y互换 
    Acc_Set(:,2) = Acc_Set_0(:,1);  % fby    
    Acc_Set(:,3) = Acc_Set_0(:,3);  % fbz    Z*(-1)
    
    %% 磁传感器标定
    headingData = (mb*Mag_Raw_Set')';
    Mag_Set_0 = [headingData(:,1) - R(1), headingData(:,2) - R(2), headingData(:,3) - R(3)]; % 三轴实际磁通量
    Mag_Set(:,1) = Mag_Set_0(:,2);   % mbx    X与Y互换
    Mag_Set(:,2) = Mag_Set_0(:,1);   % mby 
    Mag_Set(:,3) = -Mag_Set_0(:,3);  % mbz    Z*(-1)

    %% 计算静态姿态角
    Pitch(1:Marg_Number) = 0;
    Roll(1:Marg_Number) = 0;
    for i = 1 : Marg_Number
        fbx = Acc_Set(i, 1);     
        fby = Acc_Set(i, 2);
        fbz = Acc_Set(i, 3);
        % 俯仰角 
        Pitch(i) = atan(fby/sqrt((fbx)^2+(fbz)^2));     % y的输出值加- 
        % 横滚角 
        Roll_main = atan(-fbx/fbz);   
        if  fbz >= 0 
           Roll(i) = Roll_main;       % -90°～+90°
        elseif  (Roll_main <= 0) && (fbz < 0)
           Roll(i) = pi + Roll_main;  % 90~180度
        else
           Roll(i) = -pi + Roll_main; % -180~-90度 (Roll_main > 0) && (fz > 0)
        end  
    end  
    Pitch_Acc_Set = Pitch*180/pi; 
    Roll_Acc_Set = Roll*180/pi;       % 弧度->角度
    % 航向角 
    for i = 1:Marg_Number
        % 将标定后的磁传感器投影到水平坐标系
        mbx = Mag_Set(i,1);
        mby = Mag_Set(i,2);
        mbz = Mag_Set(i,3);
        % 转水平坐标系CSCY:l
        mlx = mbx*cos(Roll(i)) + mbz*sin(Roll(i));
        mly = mbx*sin(Pitch(i))*sin(Roll(i)) + mby*cos(Pitch(i)) - mbz*sin(Pitch(i))*cos(Roll(i));
        Yaw_main = -atan(mlx/mly);
        if (mlx <= 0) && (mly >=0)        % 0°到90°
            Yaw(i) = Yaw_main;
        elseif (mlx <= 0) && (mly <0)     % 90°到180°
            Yaw(i) = pi + Yaw_main;
        elseif (mlx > 0) && (mly < 0)     % 180°到270°
            Yaw(i) = pi + Yaw_main; 
        elseif (mlx > 0) && (mly >= 0)     % 270°到360°
            Yaw(i) = 2*pi + Yaw_main;
        end
    end
    Yaw_Mag_Set = Yaw * 180/pi;                % 北偏东为正，弧度->角度
    % #########################################################################
    % 3、静态姿态角(deg)
    Eul_AccMag_Set(:,1) = Pitch_Acc_Set;   % 输出欧拉角(单位：deg）
    Eul_AccMag_Set(:,2) = Roll_Acc_Set;
    Eul_AccMag_Set(:,3) = Yaw_Mag_Set;      

