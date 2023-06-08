%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：将二进制txt文件转为16进制
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 读取文本文件
% 打开文件并读取二进制数据
clc;                        
clear;
decFileName = input("请输入文件名：比如test.txt：", 's');
fid = fopen(decFileName, 'rb');
data = fread(fid);

% 将二进制数据转换为16进制字符串
hex_string_array = dec2hex(data);
hexString = "";
for k = 1 : size(hex_string_array,1)
    A = hex_string_array(k,:);
    hexString = hexString + A;
end
old = "FAFAFE";
new = newline + old;
newStr = strrep(hexString, old, new );

% 将16进制数据写入文件
hex_filename = strrep(decFileName,'txt','csv');
fileID = fopen(hex_filename,'w');
fprintf(fileID,'%s',newStr);

fclose(fileID);


