function Qnb = Eul2Qnb(Eul_Deg) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��ŷ����(�Ƕ�)ת��̬��Ԫ��
%
% Prototype: Qnb = Eul2Qnb(Eul_Rad)
% Inputs: Eul_Rad - ŷ����(�Ƕ�)
% Output: Qnb - ��̬��Ԫ��
%
% Notice����̬��ת��Ϊ��Ԫ����ע�ⷽλ�Ǳ�ƫ��Ϊ��
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     ��232
%     Eul_Rad = Eul_Deg*pi/360;
%     s = sin(Eul_Rad);   c = cos(Eul_Rad);
%     sinPITCH = s(1);    sinROLL = s(2);     sinYAW = s(3); 
%     cosPITCH = c(1);    cosROLL = c(2);     cosYAW = c(3); 
%     Qnb = [cosPITCH*cosROLL*cosYAW  -   sinPITCH*sinROLL*sinYAW;
%                 sinPITCH*cosROLL*cosYAW   -   cosPITCH*sinROLL*sinYAW;
%                 cosPITCH*sinROLL*cosYAW  +   sinPITCH*cosROLL*sinYAW;
%                 cosPITCH*cosROLL*sinYAW  +   sinPITCH*sinROLL*cosYAW];

    Eul_Rad = Eul_Deg*pi/180;
    s = sin(Eul_Rad);   c = cos(Eul_Rad);
    sinPITCH = s(1);    sinROLL = s(2);     sinYAW = s(3); 
    cosPITCH = c(1);    cosROLL = c(2);     cosYAW = c(3); 
    
    T11 = cosROLL*cosYAW + sinROLL*sinPITCH*sinYAW;
    T22 = cosPITCH*cosYAW;
    T33 = cosROLL*cosPITCH;

    T32_T23 = sinPITCH - (-sinROLL*sinYAW-cosROLL*sinPITCH*cosYAW);
    T13_T31 = sinROLL*cosYAW - cosROLL*sinPITCH*sinYAW - (-sinROLL*cosPITCH);
    T21_T12 = -cosROLL*sinYAW + sinROLL*sinPITCH*cosYAW - cosPITCH*sinYAW;
    
    q0_abs = 0.5*sqrt(1 + T11 + T22 + T33);   % q0�����ɸ������⣬������Ϊ��
    q1_abs = 0.5*sqrt(1 + T11 - T22 - T33);
    q2_abs = 0.5*sqrt(1 - T11 + T22 - T33);
    q3_abs = 0.5*sqrt(1 - T11 - T22 + T33);
    
    q00 = abs(q0_abs);
    q10 = sign(T32_T23)*abs(q1_abs);
    q20 = sign(T13_T31)*abs(q2_abs);
    q30 = sign(T21_T12)*abs(q3_abs); 
    
    % ��һ��
    q0 = q00/sqrt(q00*q00+q10*q10+q20*q20+q30*q30); 
    q1 = q10/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
    q2 = q20/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
    q3 = q30/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
    
    Qnb = [q0;q1;q2;q3];