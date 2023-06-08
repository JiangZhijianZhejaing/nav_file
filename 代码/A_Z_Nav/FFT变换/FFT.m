
%% ���ٸ���Ҷ�任
% X,Y�ֱ�������ĵ�ֵ��ʾ���еĵ���ֵ
% arry������Ķ���ʽϵ������������ǰ׼����ϵ������,���������2��n����
addpath(genpath('../../'));     % �������ļ�������m�ļ�
clc;                            % ����������
clear;                          % ��������
gvar;                           % ���ص������
CalibParm_No1;                  % ����1��ģ��궨����
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
xlabel('Ƶ��/Hz');
ylabel('���');title('N=128');
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
 xlabel('Ƶ��/Hz');
ylabel('���');title('N=2048');
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
% ������ת����
WKN = zeros(N/2,1);
for k = 0:N/2-1
    T = N*cos(2*pi*k/N);
    T = round(T);
    F = 256*sin(2*pi*k/N);
    F = round(F);
    WKN(k+1,:) = T - F*i;
end

  

fs=100;% HZ
Ts=1/fs;%����ʱ����
N=128;
n=0:N-1;
t=n*Ts;    %x����ֱ�ӹ���n�ĺ�������Ϊ�ǹ̶��Ĳ���ʱ���
x=0.5*sin(2*pi*15*t)+2*sin(2*pi*40*t);

y=fft(x,N);
f=(0:N-1)'*fs/N;
stem(f,abs(y));










% %% Ԥ����
% arry=arry(end:-1:1);  %������Ķ���ʽϵ����ת
% % ��Ϊ��������ʹ�õĶ���ʽ������a0,a1,a2,...,an��������������д��
% % ��MATLAB�У����ƽ̨��C++����Ҳ�ǣ���̫�����������ʽ��������an,a(n-1),a(n-2),...,a1,a0
% % �������������������[1 2 3],������Ķ���ʽ��x^2+2*x+3.
% % ��������������һ����f(x)=1+2*x+3*x^2(a0+a1*x+a2*x^2)
% n=length(arry);   %��ϵ�������ĳ���
% num1=0:n-1;       %����ԭʼ����ʽϵ�����У�a0,a1,a2,...an,num1=[0 1 2 ,...,n]
% num2=num1;        %����һ���������������ڴ洢��ת��Ķ����ƶ���ʽϵ��
% B=dec2bin(num1);  %������ʽת��Ϊ������
% %% ������ʽת��Ϊ�����ƺ󣬽��䷭ת�����洢
% for L=1:n
%     B(L,:)=reverse(B(L,:));
%     num2(L)=bin2dec(B(L,:));
% end
% X=zeros(1,n);  %ΪXԤ����ռ�
% %% Ԥ���㸴����X
% for k=1:n
%     X(k)=cos(((k-1)/n)*2*pi)+1i*sin(((k-1)/n)*2*pi);
% end
% count=0;
% N=n;
% %% ������Ҫ����������ʽϵ���ּ��Σ��Ż�ֵ���󵥸�ϵ��
% while(1)
%     N=N/2;
%     count=count+1;
%     if N==1
%         break;
%     end
% end
% Y=zeros(1,n); %Ϊ����ʽֵԤ����ռ�
% a1=2^(count-1); %����ÿ����Ҫ������X��Ҫ�˷��Ĵ���
%                 %ÿ����Ҫ������X��Ҫ�˷��Ĵ�������һ���ȱ����У�a1�ǵ�һ��
% q=1/2;          %�ȱ����е�q
% %% �Զ���ʽ���е�������
% for L=1:n/2     %ֻ�����n/2�μ��ɣ���n/2�οɸ���ǰ���A1(X),A2(X)�õ�
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
%     Y(L)=num3;    %�洢ǰ�벿�ֽ��ֵ
%     Y(L+n/2)=y2;  %�洢��벿�ֽ��ֵ
% end
% end