function V3 = NormlzV3(Unnormlzed_V3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��3ά������һ��
%
% Prototype: V3 = NormlzV3(Unnormlzed_V3)
% Inputs: Unnormlzed_V3 - δ��һ����3ά����
%
% Output: V3 - ��һ�����3ά����
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    nm = Unnormlzed_V3'*Unnormlzed_V3;
    V3 = Unnormlzed_V3/sqrt(nm);
   
