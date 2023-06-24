function Cnb = Eul2Cnb(Eul)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：欧拉角(角度)转姿态矩阵
%
% Prototype: Cnb = Eul2Cnb(Eul)
% Inputs: Eul - 欧拉角(角度)
% Output: Cnb - 姿态矩阵 rn = Cnb*rb
%
% Notice：姿态角转换为姿态矩阵，注意方位角北偏东为正
%
% By Taoran Zhao
% 2023/04/29 2023/05/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 见《惯性导航 (第三版) (秦永元 编著)》P260
%% ---------------一、转弧度---------------
Eul_Rad = Eul*pi/180;       
s = sin(Eul_Rad); c = cos(Eul_Rad);
sinPITCH = s(1); sinROLL = s(2); sinYAW = s(3); 
cosPITCH = c(1); cosROLL = c(2); cosYAW = c(3);
%% ---------------二、计算T矩阵---------------
T11 = cosROLL*cosYAW + sinROLL*sinPITCH*sinYAW;
T12 = cosPITCH*sinYAW;
T13 = sinROLL*cosYAW - cosROLL*sinPITCH*sinYAW;
T21 = -cosROLL*sinYAW + sinROLL*sinPITCH*cosYAW;
T22 = cosPITCH*cosYAW;
T23 = -sinROLL*sinYAW - cosROLL*sinPITCH*cosYAW;
T31 = -sinROLL*cosPITCH;
T32 = sinPITCH;
T33 = cosROLL*cosPITCH;
%% ---------------三、计算Cnb---------------
Cnb = [ T11,T12,T13;
        T21,T22,T23;
        T31,T32,T33 ];


