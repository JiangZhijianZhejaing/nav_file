function diagP = P2diagP(P)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：P阵对称化
%
% Prototype: diagP = P2diagP(P)
% Inputs: P - P阵
% Output: diagP - 对称化后的P阵
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    diagP = (P + P')/2;
