function Qnb_k = Rungekuta_4(Qnb_ksub2, Wxyz_ksub1_2_ksub3_Set, Sample_Interval)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：四阶龙格库塔法(废)
%
% Prototype: Qnb_k = Rungekuta_4(Qnb_ksub2, Wxyz_ksub1_2_ksub3_Set, Sample_Interval)
% Inputs: Qnb_ksub2 - k-2时刻姿态四元数
%         Wxyz_ksub1_2_ksub3_Set - k-2到k-3时刻角速度(角度)
%         Sample_Interval - 采样间隔
% Output: Qnb_k - k时刻姿态四元数
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T = 2*Sample_Interval;
    Wxyz1 = Wxyz_ksub1_2_ksub3_Set(1,:);
    Wxyz2 = Wxyz_ksub1_2_ksub3_Set(2,:);
    Wxyz3 = Wxyz_ksub1_2_ksub3_Set(3,:);
    Wx1 = Wxyz1(1)*pi/180;   % t
    Wy1 = Wxyz1(2)*pi/180;
    Wz1 = Wxyz1(3)*pi/180;  
    Wx2 = Wxyz2(1)*pi/180;   % t + 1/2T
    Wy2 = Wxyz2(2)*pi/180;
    Wz2 = Wxyz2(3)*pi/180;    
    Wx3 = Wxyz3(1)*pi/180;   % t + T
    Wy3 = Wxyz3(2)*pi/180;   
    Wz3 = Wxyz3(3)*pi/180;   
    k1 = 1/2*[0,   -Wx1,  -Wy1,  -Wz1; Wx1,  0,    Wz1,   -Wy1; Wy1,  -Wz1,  0,    Wx1; Wz1,  Wy1,   -Wx1,  0   ]*Qnb_ksub2;
    k2 = 1/2*[0,   -Wx2,  -Wy2,  -Wz2; Wx2,  0,    Wz2,   -Wy2; Wy2,  -Wz2,  0,    Wx2; Wz2,  Wy2,   -Wx2,  0   ]*(Qnb_ksub2 + k1/2);
    k3 = 1/2*[0,   -Wx2,  -Wy2,  -Wz2; Wx2,  0,    Wz2,   -Wy2; Wy2,  -Wz2,  0,    Wx2; Wz2,  Wy2,   -Wx2,  0   ]*(Qnb_ksub2 + k2/2);
    k4 = 1/2*[0,   -Wx3,  -Wy3,  -Wz3; Wx3,  0,    Wz3,   -Wy3; Wy3,  -Wz3,  0,    Wx3; Wz3,  Wy3,   -Wx3,  0   ]*(Qnb_ksub2 + k3);
    Qnb_k = Qnb_ksub2 + T/6*(k1 + 2*k2 + 2*k3 + k4);  % 2时刻前  k-2
    Qnb_k = NormlzQnb(Qnb_k);