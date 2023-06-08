function V3 = NormlzV3(Unnormlzed_V3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：3维向量归一化
%
% Prototype: V3 = NormlzV3(Unnormlzed_V3)
% Inputs: Unnormlzed_V3 - 未归一化的3维向量
%
% Output: V3 - 归一化后的3维向量
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    nm = Unnormlzed_V3'*Unnormlzed_V3;
    V3 = Unnormlzed_V3/sqrt(nm);
   
