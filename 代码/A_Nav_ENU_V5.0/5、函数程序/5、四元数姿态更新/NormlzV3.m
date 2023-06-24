function V3_Norm = NormlzV3(V3_UnNorm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：3维向量归一化
%
% Prototype: V3 = NormlzV3(V30)
% Inputs: V3_UnNorm - 未归一化的3维向量
% Output: V3_Norm - 归一化后的3维向量
%
% By Taoran Zhao
% 2023/04/13 2023/05/18
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nm = V3_UnNorm'*V3_UnNorm;
V3_Norm = V3_UnNorm/sqrt(nm);
   
