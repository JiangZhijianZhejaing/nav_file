%%% ������תΪfloat��
function res = bin2flo(dbin)

if size(dbin,2)~=32
    error('Input fault ! ����32λ !');
elseif (size(strfind(dbin,'0'),2)+size(strfind(dbin,'1'),2))~=32
    error('Input fault ! ������0��1��Ԫ�� !');
end

idx = bin2dec(dbin(2:9));
ls = idx-127;    % ��λλ��
if ls>0
    flo_int = bin2dec(['1' dbin(10:9+ls)]);    % ��������
    mdx = strfind(dbin(10+ls:end),'1');
    flo_dec = sum(2.^(-mdx));     % С������
else
    flo_int = 0;
    mdx = strfind(['1' dbin(10:end)],'1');
    flo_dec = sum(2.^(ls+1-mdx));     % С������
end
res = flo_int + flo_dec;
if dbin(1)=='1'
    res = -res;
end
