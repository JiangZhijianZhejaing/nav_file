function Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：计算量测欧拉角(角度)
%
% Prototype: Eul_AccMag_Set = EulAccMag(Gyro_Set, Acc_Set, Mag_Set)
% Inputs: Gyro_Set - 角速度集
%         Acc_Set - 加速度集
%         Mag_Set - 磁矢量集
% Output: Eul_AccMag_Deg_Set - 静态姿态角集
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、计算俯仰角和横滚角---------------
Marg_Number = size(Acc_Set);                    % 采样数
Pitch(1:Marg_Number) = 0;                          % 提前分配内存
Roll(1:Marg_Number) = 0;
for i = 1 : Marg_Number
    fb_x = Acc_Set(i, 1);     
    fb_y = Acc_Set(i, 2);
    fb_z = Acc_Set(i, 3);
    if fb_z == 0
        fb_z = 0.000001;
    end
    % 俯仰角 
    Pitch(i) = atan(fb_y/sqrt((fb_x)^2+(fb_z)^2));      
    % 横滚角                   
    Roll_main = atan(-fb_x/fb_z);   % atan取值±90°，需要真值判断 
    % Roll真值判断
    if  fb_z >= 0 
       Roll(i) = Roll_main;         % -90°～+90°
    elseif  (Roll_main <= 0) && (fb_z < 0)
       Roll(i) = pi + Roll_main;    % 90°～180°
    else
       Roll(i) = -pi + Roll_main;   % -180°～-90°
    end  
end  
Pitch_Acc_Set = Pitch*180/pi; 
Roll_Acc_Set = Roll*180/pi;         % 弧度->角度
%% ---------------二、计算航向角---------------
Yaw(1:Marg_Number) = 0;
for i = 1:Marg_Number
    % 将标定后的磁传感器投影到水平坐标系
    mbx = Mag_Set(i,1);
    mby = Mag_Set(i,2);
    mbz = Mag_Set(i,3);
    % 转水平坐标系CSCY:l
    mlx = mbx*cos(Roll(i)) + mbz*sin(Roll(i));
    mly = mbx*sin(Pitch(i))*sin(Roll(i)) + mby*cos(Pitch(i)) - mbz*sin(Pitch(i))*cos(Roll(i));
    Yaw_main = -atan(mlx/mly);
    if (mlx <= 0) && (mly >=0)          % 0°～90°
        Yaw(i) = Yaw_main;
    elseif (mlx <= 0) && (mly <0)       % 90°～180°
        Yaw(i) = pi + Yaw_main;
    elseif (mlx > 0) && (mly < 0)       % -180°～-90°
        Yaw(i) = Yaw_main - pi; 
    elseif (mlx > 0) && (mly >= 0)      % -90°～0°
        Yaw(i) = Yaw_main;
    end
end
Yaw_Mag_Set = Yaw * 180/pi;                % 北偏东为正，弧度->角度
% 3、静态姿态角(deg)
Eul_AccMag_Set(:,1) = Pitch_Acc_Set;       % 输出欧拉角(单位：deg）
Eul_AccMag_Set(:,2) = Roll_Acc_Set;
Eul_AccMag_Set(:,3) = Yaw_Mag_Set;   


