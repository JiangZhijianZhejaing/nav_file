function Place = Pos2Place(Pos, Pos_Init)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function���������ϵλ��ת����ֱ������ϵλ��
%
% Prototype: Place = Pos2Place(Pos, Pos_Init)
% Inputs: Pos - �������ϵλ��
%         Pos_Init - �������ϵ��ʼλ��
%
% Output: Place - ����ֱ������ϵλ��
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gvar;
    RMh = (Re*(1-e2))/(1-e2*sin(Pos(1))*sin(Pos(1)))^(3/2) + Pos(3);       % ����Ȧ
    RNh = Re/(1-e2*sin(Pos(1))*sin(Pos(1)))^(1/2) + Pos(3);                % î��Ȧ
    Place(1) = (Pos(2) - Pos_Init(2))*RMh;  % x��
    Place(2) = (Pos(1) - Pos_Init(1))*RNh;  % y��
    Place(3) = Pos(3) - Pos_Init(3);        % z��