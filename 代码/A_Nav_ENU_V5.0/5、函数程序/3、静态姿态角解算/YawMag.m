function yaw = YawMag(pitch, roll, Mag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：计算静态航向角(角度)
%
% Prototype: yaw = YawMag(pitch,roll,Mag)
% Inputs: pitch - 俯仰角
%         roll - 横滚角
%         Mag - 磁矢量
% Output: yaw - 航向角
%
% By Taoran Zhao
% 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% ---------------一、将标定后的磁传感器投影到水平坐标系---------------
% Pitch = pitch*pi/180;
% Roll = roll*pi/180;
% mbx = Mag(1);
% mby = Mag(2);
% mbz = Mag(3);
% % 转水平坐标系CSCY:l
% mlx = mbx*cos(Roll) + mbz*sin(Roll);
% mly = mbx*sin(Pitch)*sin(Roll) + mby*cos(Pitch) - mbz*sin(Pitch)*cos(Roll);
% %% ---------------二、解算航向角---------------
% Yaw_main = -atan(mlx/mly);
% if (mlx <= 0) && (mly >=0)        % 0°到90°
%     yaw = Yaw_main;
% elseif (mlx <= 0) && (mly <0)     % 90°到180°
%     yaw = pi + Yaw_main;
% elseif (mlx > 0) && (mly < 0)     % -180°到-90°
%     yaw = Yaw_main - pi; 
% elseif (mlx > 0) && (mly >= 0)     % -90°到0°
%     yaw = Yaw_main;
% end
% Yaw = yaw * 180/pi;                % 北偏东为正，弧度->角度

%% ---------------一、将标定后的磁传感器投影到水平坐标系---------------
Pitch = pitch*pi/180;
Roll = roll*pi/180;
mbx = Mag(1);
mby = Mag(2);
mbz = Mag(3);
% 转水平坐标系CSCY:l
mlx = mbx*cos(Roll) + mbz*sin(Roll);
mly = mbx*sin(Pitch)*sin(Roll) + mby*cos(Pitch) - mbz*sin(Pitch)*cos(Roll);
Yaw_main = -atan(mlx/mly);
if (mlx <= 0) && (mly >=0)        % 0°到90°
    Yaw = Yaw_main;
elseif (mlx <= 0) && (mly <0)     % 90°到180°
    Yaw = pi + Yaw_main;
elseif (mlx > 0) && (mly < 0)     % -180°到-90°
    Yaw = Yaw_main - pi; 
elseif (mlx > 0) && (mly >= 0)     % -90°到0°
    Yaw = Yaw_main;
end
yaw = Yaw * 180/pi;                % 北偏东为正，弧度->角度