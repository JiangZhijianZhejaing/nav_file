
%% 快速傅里叶变换
% X,Y分别是输出的点值表示法中的点与值
% arry是输入的多项式系数向量，请提前准备好系数向量,长度务必是2的n次幂
addpath(genpath('../../'));     % 导入主文件夹所有m文件
clc;                            % 清理命令行
clear;                          % 清理工作区
gvar;                           % 加载地球参数
CalibParm_No1;                  % 加载1号模块标定参数
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
V = Acc_Set(:,1);



fs=100;
N=numel(V);
n=0:N-1;
t=n/fs;
y=0.5*sin(2*pi*20*t)+2*sin(2*pi*40*t);
x=fft(y,N);
m=abs(x);
f=n*fs/N;
subplot(2,2,1),plot(f,m);
xlabel('频率/Hz');
ylabel('振幅');title('N=128');
grid on;
subplot(2,2,2),plot(f(1:N/2),m(1:N/2));
grid on;


 fs=100;
 N=numel(V);
 n=0:N-1;
 t=n/fs;
 y=0.5*sin(2*pi*20*t)+2*sin(2*pi*40*t);
 x=fft(y,N);
 m=abs(x);
 f=n*fs/N;
 subplot(2,2,3),plot(f,m);
 xlabel('频率/Hz');
ylabel('振幅');title('N=2048');
 grid on;
 subplot(2,2,4),plot(f(1:N/2),m(1:N/2));
grid on;







if mod(numel(V)/2,2) == 0
    V = Acc_Set(:,1);
else
    V = Acc_Set(1:end-1,1);
end

N = numel(V);
k = zeros(1,N/2);
k = [0:N/2-1];
N = 256;
% 计算旋转因子
WKN = zeros(N/2,1);
for k = 0:N/2-1
    T = N*cos(2*pi*k/N);
    T = round(T);
    F = 256*sin(2*pi*k/N);
    F = round(F);
    WKN(k+1,:) = T - F*i;
end

  

fs=100;% HZ
Ts=1/fs;%采样时间间隔
N=128;
n=0:N-1;
t=n*Ts;    %x不是直接关于n的函数，因为是固定的采样时间点
x=0.5*sin(2*pi*15*t)+2*sin(2*pi*40*t);

y=fft(x,N);
f=(0:N-1)'*fs/N;
stem(f,abs(y));










% %% 预计算
% arry=arry(end:-1:1);  %将输入的多项式系数翻转
% % 因为本程序中使用的多项式序列是a0,a1,a2,...,an，按照书面用语写的
% % 在MATLAB中（别的平台，C++可能也是，不太清楚），多项式的序列是an,a(n-1),a(n-2),...,a1,a0
% % 例如你在命令窗口中输入[1 2 3],它代表的多项式是x^2+2*x+3.
% % 而在书面用语中一般是f(x)=1+2*x+3*x^2(a0+a1*x+a2*x^2)
% n=length(arry);   %求系数向量的长度
% num1=0:n-1;       %创建原始多项式系数序列，a0,a1,a2,...an,num1=[0 1 2 ,...,n]
% num2=num1;        %创建一个备用向量，用于存储翻转后的二进制多项式系数
% B=dec2bin(num1);  %将多项式转换为二进制
% %% 将多项式转化为二进制后，将其翻转，并存储
% for L=1:n
%     B(L,:)=reverse(B(L,:));
%     num2(L)=bin2dec(B(L,:));
% end
% X=zeros(1,n);  %为X预分配空间
% %% 预计算复数点X
% for k=1:n
%     X(k)=cos(((k-1)/n)*2*pi)+1i*sin(((k-1)/n)*2*pi);
% end
% count=0;
% N=n;
% %% 计算需要将整个多项式系数分几次，才会分到最后单个系数
% while(1)
%     N=N/2;
%     count=count+1;
%     if N==1
%         break;
%     end
% end
% Y=zeros(1,n); %为多项式值预分配空间
% a1=2^(count-1); %计算每次需要迭代中X需要乘方的次数
%                 %每次需要迭代中X需要乘方的次数构成一个等比数列，a1是第一项
% q=1/2;          %等比数列的q
% %% 对多项式进行迭代计算
% for L=1:n/2     %只需计算n/2次即可，后n/2次可根据前面的A1(X),A2(X)得到
%     num3=num2;
%     for LL=1:count
%         len=length(num3);
%         nt=0;
%         an=a1*q^(LL-1);
%         for LLL=1:2:len
%             nt=nt+1;
%             if LL==1
%                 y1(nt)=arry(num3(LLL)+1)+(X(L)^an)*arry(num3(LLL+1)+1);
%             elseif LL==count
%                 y1(nt)=num3(LLL)+(X(L)^an)*num3(LLL+1);
%                 y2=num3(LLL)-(X(L)^an)*num3(LLL+1);
%             else
%                 y1(nt)=num3(LLL)+(X(L)^an)*num3(LLL+1);
%             end
%         end
%         num3=y1(1:nt);
%     end
%     Y(L)=num3;    %存储前半部分结果值
%     Y(L+n/2)=y2;  %存储后半部分结果值
% end
% end