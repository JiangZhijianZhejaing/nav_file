%%% float型转为二进制
function res = flo2bin(dflo)
if dflo == 0
    res = 0;
else
    res = int2str(zeros(1,32));
    res(res==' ') = [];
    if dflo<0    % 符号位
        res(1) = '1';
    end
    flo_int = fix(abs(dflo));    % 整数部分
    flo_dec = abs(dflo) - flo_int;    % 小数部分
    bin_flo_int = dec2bin(flo_int);    % 整数部分转换为二进制
    fir1 = strfind(bin_flo_int,'1');    % 查找整数位的1，
    if isempty(fir1)
        num_bfi = 0;                        %若找不到，则整数位位数为0；
    else
        num_bfi = size(bin_flo_int,2);      % 否则，最高位必为1，则整数位位数即为转换后的位数
        if num_bfi>1
            res(10:num_bfi+8) = bin_flo_int(2:end);
        end
    end    
    %%% 计算尾数部分
    bfd=flo_dec;
    i=-1;
    num_bfd=0;
    rs=0;
    while (num_bfi+num_bfd)<24
        a1=fix(bfd/(2^i));  % 第i位值 
        if ~(num_bfi||num_bfd||a1)
            rs = rs+1;    % 记录 右移位数-1
        else
            num_bfd = num_bfd +1;    % 小数部分二进制位数
            res(8+num_bfd+num_bfi) = int2str(a1);
        end
        
        bfd=bfd-a1*(2^i);    % 用于计算下一位
        if bfd==0
            break;
        end
        i=i-1;
    end
    %%% 按四舍五入计算最后是否加1
    if fix(bfd/(2^i))
        finb = dec2bin(bin2dec(res(end-3:end))+1);
        res(end-size(finb,2)+1:end) = finb;
    end
    %%% 计算指数部分
    if num_bfi>0
        if num_bfi==1
            res(3:9) = dec2bin(127);
        else
            res(2:9) = dec2bin(126+num_bfi);
        end
    else
        idx = size(dec2bin(126-rs),2);
        res(10-idx:9) = dec2bin(126-rs);
    end
end
% res_d = bin2dec(res);
% res_x = dec2hex(res_d)