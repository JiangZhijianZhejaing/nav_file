function Normlzed_Qnb = NormlzQnb(Unnormlzed_Qnb)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function����Ԫ����һ��
%
% Prototype: Normed_Qnb = NormQnb(Unnormlzed_Qnb)
% Inputs: Unnormlzed_Qnb - δ��һ������̬��Ԫ��
% Output: Normlzed_Qnb - ��һ�������Ԫ��
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Qnb2 = Unnormlzed_Qnb'*Unnormlzed_Qnb;
    q0 = Unnormlzed_Qnb(1)/sqrt(Qnb2);
    q1 = Unnormlzed_Qnb(2)/sqrt(Qnb2);
    q2 = Unnormlzed_Qnb(3)/sqrt(Qnb2);
    q3 = Unnormlzed_Qnb(4)/sqrt(Qnb2);
    Normlzed_Qnb = [q0; q1; q2; q3];      % ��һ��    
    
%     q00 = Unnormlzed_Qnb(1);
%     q10 = Unnormlzed_Qnb(2);
%     q20 = Unnormlzed_Qnb(3);
%     q30 = Unnormlzed_Qnb(4);
%     q0 = q00/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
%     q1 = q10/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
%     q2 = q20/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
%     q3 = q30/sqrt(q00*q00+q10*q10+q20*q20+q30*q30);
%     Normlzed_Qnb = [q0; q1; q2; q3];      % ��һ��