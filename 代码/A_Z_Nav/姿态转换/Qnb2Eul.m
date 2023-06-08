function Eul_Deg = Qnb2Eul(Qnb)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function����̬��Ԫ��תŷ����(�Ƕ�)
%
% Prototype: Eul_Deg = Qnb2Eul(Qnb)
% Inputs: Qnb - ��̬��Ԫ��
% Output: Eul_Deg - ŷ����(�Ƕ�)
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q0 = Qnb(1);
    q1 = Qnb(2);
    q2 = Qnb(3);
    q3 = Qnb(4);
    T32 = 2*(q2*q3 + q0*q1);
    T31 = 2*(q1*q3 - q0*q2);
    T33 = 1 - 2*(q1*q1 + q2*q2);
    T12 = 2*(q1*q2 - q0*q3);
    T22 = 1-2*(q1*q1 + q3*q3);
    Eul_Rad = [ asin(T32);
            atan2(-T31,T33); 
            atan2(T12,T22) ];
%     Eul_Rad = AttYesOrNo(Eul_Rad);
    Eul_Deg = Eul_Rad*180/pi;

    if  0 <= Eul_Deg(3) && Eul_Deg(3)<=180
        Eul_Deg(3) =  Eul_Deg(3);
    else
        Eul_Deg(3) =  Eul_Deg(3)+360;   % -180��0
    end 
 
    
    
    
    
    