function Eul = Cnb2Eul(Cnb)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：姿态矩阵转欧拉角(角度)
%
% Prototype: Eul = Cnb2Eul(Cnb)
% Inputs: Cnb - 姿态矩阵
% Output: Eul - 欧拉角(角度)
%
% By Taoran Zhao
% 2023/05/18 2023/05/19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、姿态矩阵转欧拉角---------------
% 见《惯性导航 (第三版) (秦永元 编著)》P260
% ----------1、计算T矩阵----------
% 完整T矩阵即Cnb
T12 = Cnb(1,2);
T22 = Cnb(2,2);
T31 = Cnb(3,1);
T32 = Cnb(3,2);                  
T33 = Cnb(3,3);
% ----------2、计算临时欧拉角----------
Pitch = asin(T32);
Roll_Main = atan(-T31/T33);
Yaw_Main = atan(T12/T22);
%% ---------------二、欧拉角真值判断---------------
% ----------1、航向角真值判断----------
if abs(T22) < 0.000001
    if T12 > 0
        Yaw = 90*pi/180;
    else
        Yaw = -90*pi/180;
    end
elseif T22 > 0
    Yaw = Yaw_Main;
else
    if T12 > 0
        Yaw = Yaw_Main + pi;
    else
        Yaw = Yaw_Main - pi;
    end
end
% ----------2、横滚角真值判断----------
if T33 > 0
    Roll = Roll_Main;
elseif T33 < 0
    if Roll_Main > 0
        Roll = Roll_Main - pi;
    else
        Roll = Roll_Main + pi;
    end
end
Eul_Rad = [Pitch; Roll; Yaw];
Eul = Eul_Rad*180/pi;