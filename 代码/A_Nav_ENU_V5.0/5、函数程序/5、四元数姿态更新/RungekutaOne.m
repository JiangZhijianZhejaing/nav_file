function Qnb_k = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：一阶龙格库塔法(四元数归一化)
%
% Prototype: Qnb_k = Rungekuta_1(Wxyz_ksub1, Sample_Interval, Qnb_ksub1)
% Inputs: W_ksub1 - k-1时刻角速度(角度)
%         Sample_Interval - 采样间隔
%         Qnb_ksub1 - k-1时刻姿态四元数 严恭敏版命名(秦永元版为Qbn_ksub1)
% Output: Qnb_k - k时刻姿态四元数
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、转弧度---------------
Wx = W_Deg_ksub1(1)*pi/180;   
Wy = W_Deg_ksub1(2)*pi/180;
Wz = W_Deg_ksub1(3)*pi/180;
Omega = [ 0,  -Wx, -Wy, -Wz;
          Wx,  0,   Wz, -Wy;
          Wy, -Wz,  0,   Wx;
          Wz,  Wy, -Wx,  0  ];                          % 见《惯性导航 (第三版) (秦永元 编著)》P262
Qnb_k = (eye(4)+0.5*Sample_Interval*Omega)*Qnb_ksub1;
Qnb_k = NormlzQua(Qnb_k); % 四元数归一化
%% ---------------二、统一四元数旋转方向---------------
% 规定q0为正
if Qnb_k(1) < 0
    Qnb_k = -Qnb_k;
end