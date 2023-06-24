function Pos = Place2Pos(Place, Pos_Init)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：当地直角坐标系位置转大地坐标系位置UTM->GC
%
% Prototype: Pos = Place2Pos(Place, Pos_Init)
% Inputs: Place - 当地直角坐标系位置
%         Pos_Init - 大地坐标系初始位置
%
% Output: Pos - 大地坐标系位置
%
% Notice：逆运算方程解不出来，做了简化
%
% By Taoran Zhao
% 2023/05/06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
EarthGvar;
% 计算子午圈半径 RMh 和卯酉圈半径 RNh
RMh = (Re*(1-e2))/(1-e2*sin(Pos_Init(1))*sin(Pos_Init(1)))^(3/2) + Pos_Init(3);       % 子午圈
RNh = Re/(1-e2*sin(Pos_Init(1))*sin(Pos_Init(1)))^(1/2) + Pos_Init(3);                % 卯酉圈
% 计算大地坐标系位置
Pos(1) = Place(2)/RNh + Pos_Init(1);
Pos(2) = Place(1)/RMh + Pos_Init(2);
Pos(3) = Place(3) + Pos_Init(3);
Pos = [Pos(1);Pos(2);Pos(3)];




