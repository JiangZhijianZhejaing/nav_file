%%% float数据转换成int16型数据,小端模式
function res = flo2int(dflo)
num = size(dflo,2);
res = zeros(1,2*num);
for i=1:num
    dbin = flo2bin(dflo(i));
    res(i*2) = bin2dec(dbin(1:16));
    res(i*2-1) = bin2dec(dbin(17:end));
end
res(res>32768) = res(res>32768)-65536;
