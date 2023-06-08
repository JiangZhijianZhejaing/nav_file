%%% float��תΪ������
function res = flo2bin(dflo)
if dflo == 0
    res = 0;
else
    res = int2str(zeros(1,32));
    res(res==' ') = [];
    if dflo<0    % ����λ
        res(1) = '1';
    end
    flo_int = fix(abs(dflo));    % ��������
    flo_dec = abs(dflo) - flo_int;    % С������
    bin_flo_int = dec2bin(flo_int);    % ��������ת��Ϊ������
    fir1 = strfind(bin_flo_int,'1');    % ��������λ��1��
    if isempty(fir1)
        num_bfi = 0;                        %���Ҳ�����������λλ��Ϊ0��
    else
        num_bfi = size(bin_flo_int,2);      % �������λ��Ϊ1��������λλ����Ϊת�����λ��
        if num_bfi>1
            res(10:num_bfi+8) = bin_flo_int(2:end);
        end
    end    
    %%% ����β������
    bfd=flo_dec;
    i=-1;
    num_bfd=0;
    rs=0;
    while (num_bfi+num_bfd)<24
        a1=fix(bfd/(2^i));  % ��iλֵ 
        if ~(num_bfi||num_bfd||a1)
            rs = rs+1;    % ��¼ ����λ��-1
        else
            num_bfd = num_bfd +1;    % С�����ֶ�����λ��
            res(8+num_bfd+num_bfi) = int2str(a1);
        end
        
        bfd=bfd-a1*(2^i);    % ���ڼ�����һλ
        if bfd==0
            break;
        end
        i=i-1;
    end
    %%% �����������������Ƿ��1
    if fix(bfd/(2^i))
        finb = dec2bin(bin2dec(res(end-3:end))+1);
        res(end-size(finb,2)+1:end) = finb;
    end
    %%% ����ָ������
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