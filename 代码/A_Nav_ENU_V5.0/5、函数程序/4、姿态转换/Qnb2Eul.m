function Eul = Qnb2Eul(Qnb)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：姿态四元数转欧拉角(角度)
%
% Prototype: Eul = Qnb2Eul(Qnb)
% Inputs: Qnb - 姿态四元数
% Output: Eul - 欧拉角(角度)
%
% By Taoran Zhao
% 2023/04/29 2023/05/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、统一四元数旋转方向---------------
% 规定q0为正
if Qnb(1) < 0
    Qnb = -Qnb;
end
%% ---------------二、四元数转欧拉角---------------
q0 = Qnb(1);
q1 = Qnb(2);
q2 = Qnb(3);
q3 = Qnb(4);
% ----------1、计算T矩阵----------
% 完整T矩阵即Cnb
T12 = 2*(q1*q2 - q0*q3);
T22 = q0^2 - q1^2 + q2^2 - q3^2;
T31 = 2*(q1*q3 - q0*q2);
T32 = 2*(q2*q3 + q0*q1);                  
T33 = q0^2 - q1^2 - q2^2 + q3^2;
% ----------2、计算临时欧拉角----------
Pitch = asin(T32);
Roll_Main = atan(-T31/T33);
Yaw_Main = atan(T12/T22);
%% ---------------三、欧拉角真值判断---------------
% 见《惯性导航 (第三版) (秦永元 编著)》P260
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