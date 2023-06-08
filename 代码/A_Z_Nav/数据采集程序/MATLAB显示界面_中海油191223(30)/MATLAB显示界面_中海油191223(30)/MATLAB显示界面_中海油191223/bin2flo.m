%%% 二进制转为float型
function res = bin2flo(dbin)

if size(dbin,2)~=32
    error('Input fault ! 不足32位 !');
elseif (size(strfind(dbin,'0'),2)+size(strfind(dbin,'1'),2))~=32
    error('Input fault ! 包含非0和1的元素 !');
end

idx = bin2dec(dbin(2:9));
ls = idx-127;    % 移位位数
if ls>0
    flo_int = bin2dec(['1' dbin(10:9+ls)]);    % 整数部分
    mdx = strfind(dbin(10+ls:end),'1');
    flo_dec = sum(2.^(-mdx));     % 小数部分
else
    flo_int = 0;
    mdx = strfind(['1' dbin(10:end)],'1');
    flo_dec = sum(2.^(ls+1-mdx));     % 小数部分
end
res = flo_int + flo_dec;
if dbin(1)=='1'
    res = -res;
end
