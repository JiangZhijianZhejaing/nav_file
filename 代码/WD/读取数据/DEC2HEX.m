%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function����������txt�ļ�תΪ16����
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ��ȡ�ı��ļ�
% ���ļ�����ȡ����������
clc;                        
clear;
decFileName = input("�������ļ���������test.txt��", 's');
fid = fopen(decFileName, 'rb');
data = fread(fid);

% ������������ת��Ϊ16�����ַ���
hex_string_array = dec2hex(data);
hexString = "";
for k = 1 : size(hex_string_array,1)
    A = hex_string_array(k,:);
    hexString = hexString + A;
end
old = "FAFAFE";
new = newline + old;
newStr = strrep(hexString, old, new );

% ��16��������д���ļ�
hex_filename = strrep(decFileName,'txt','csv');
fileID = fopen(hex_filename,'w');
fprintf(fileID,'%s',newStr);

fclose(fileID);


