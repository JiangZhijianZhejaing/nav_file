function Place = Pos2Place(Pos, Pos_Init)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：大地坐标系位置转当地直角坐标系位置
%
% Prototype: Place = Pos2Place(Pos, Pos_Init)
% Inputs: Pos - 大地坐标系位置
%         Pos_Init - 大地坐标系初始位置
%
% Output: Place - 当地直角坐标系位置
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gvar;
    RMh = (Re*(1-e2))/(1-e2*sin(Pos(1))*sin(Pos(1)))^(3/2) + Pos(3);       % 子午圈
    RNh = Re/(1-e2*sin(Pos(1))*sin(Pos(1)))^(1/2) + Pos(3);                % 卯酉圈
    Place(1) = (Pos(2) - Pos_Init(2))*RMh;  % x东
    Place(2) = (Pos(1) - Pos_Init(1))*RNh;  % y北
    Place(3) = Pos(3) - Pos_Init(3);        % z天