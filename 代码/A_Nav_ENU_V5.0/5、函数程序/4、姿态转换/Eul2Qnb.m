function Qnb = Eul2Qnb(Eul)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：欧拉角(角度)转姿态四元数
%
% Prototype: Qnb = Eul2Qbn(Eul)
% Inputs: Eul - 欧拉角(角度)
% Output: Qnb - 姿态四元数 rn = Qnb⊗rb⊗QuaConj(Qnb)  这里的Qnb为参考严恭敏命名，等同于秦永元版的Qbn          
%
% Notice：姿态角转换为四元数，注意方位角北偏东为正  
%
% By Taoran Zhao
% 2023/04/29 2023/05/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、转弧度---------------
Eul_Rad = Eul*pi/180;       
s = sin(Eul_Rad); c = cos(Eul_Rad);
sinPITCH = s(1); sinROLL = s(2); sinYAW = s(3); 
cosPITCH = c(1); cosROLL = c(2); cosYAW = c(3); 
%% ---------------二、解算四元数---------------
% 见《惯性导航 (第三版) (秦永元 编著)》P260
% 解算T矩阵，即Cnb
T11 = cosROLL*cosYAW + sinROLL*sinPITCH*sinYAW;
T22 = cosPITCH*cosYAW;
T33 = cosROLL*cosPITCH;
T12 = cosPITCH*sinYAW;
T13 = sinROLL*cosYAW - cosROLL*sinPITCH*sinYAW;
T21 = -cosROLL*sinYAW + sinROLL*sinPITCH*cosYAW;
T23 = -sinROLL*sinYAW - cosROLL*sinPITCH*cosYAW;
T31 = -sinROLL*cosPITCH;
T32 = sinPITCH;
T32_T23 = T32 - T23;
T13_T31 = T13 - T31;
T21_T12 = T21 - T12;
% 见《惯性导航 (第三版) (秦永元 编著)》P266
q0_abs = 0.5*sqrt(1+T11+T22+T33);   % q0可正可负，这里统一规定为正
q1_abs = 0.5*sqrt(1+T11-T22-T33);
q2_abs = 0.5*sqrt(1-T11+T22-T33);
q3_abs = 0.5*sqrt(1-T11-T22+T33);
q00 = abs(q0_abs);
q10 = sign(T32_T23)*abs(q1_abs);
q20 = sign(T13_T31)*abs(q2_abs);
q30 = sign(T21_T12)*abs(q3_abs); 
%% ---------------三、归一化---------------
q0 = q00/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
q1 = q10/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
q2 = q20/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
q3 = q30/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
Qnb = [q0;q1;q2;q3];
    