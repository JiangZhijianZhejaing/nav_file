%导入数据
% datax=xlsread('2022081604.xls','G2:G3231');
% datay=xlsread('2022081604.xls','H2:H3231');
% dataz=xlsread('2022081604.xls','I2:I3231');
datax=xlsread('0321.xls','G2:G2848')';
datay=xlsread('0321.xls','H2:H2848')';
dataz=xlsread('0321.xls','I2:I2848')';
%拟合曲面
H=zeros(length(datax),6);%数据矩阵
for i=1:length(datax)
    H(i,1)=datax(i)*datax(i);
    H(i,2)=datay(i)*datay(i);
    H(i,3)=dataz(i)*dataz(i);
    H(i,4)=2*datax(i);
    H(i,5)=2*datay(i);
    H(i,6)=2*dataz(i);
    %球型拟合，无旋转项，采集的数据应尽量吻合
end
F=ones(length(datax),1);%右侧的1向量
%HA=1，A中是椭球的系数
E_para=((inv(H'*H))*H')*F;
a1=E_para(1);
a2=E_para(2);
a3=E_para(3);
a4=E_para(4);
a5=E_para(5);
a6=E_para(6);
%参数计算
o1=a4/a1;
o2=a5/a2;
o3=a6/a3;
G=1+a4*a4/a1+a5*a5/a2+a6*a6/a3;
g1=sqrt(a1/G);
g2=sqrt(a2/G);
g3=sqrt(a3/G);
%校正
rawdata_x=[4488,3493,869,-2588,-3969,-3227,-398,2594];
rawdata_y=[840,-2469,-4563,-3713,-592,3006,4780,3966];
rawdata_z=[6043,6036,6017,6038,6052,6059,6052,6043];
truedata2=[(rawdata_x(1,:)+o1)/g1; (rawdata_y(1,:)+o2)/g2; (rawdata_z(1,:)+o3)/g3];
truedata2_x=truedata2(1,:);
truedata2_y=truedata2(2,:);
truedata2_z=truedata2(3,:);
%测试
raw_fai=zeros(1,length(rawdata_x));
for i=1:length(rawdata_x);
    if((rawdata_x(i)==0)&&(rawdata_y(i)<0))
        raw_fai(i)=90;
    elseif((rawdata_x(i)==0)&&(rawdata_y(i)>0))
        raw_fai(i)=270;
    elseif((rawdata_x(i)<0))
        raw_fai(i)=(-atan(rawdata_y(i)/rawdata_x(i))*180/pi)+180;
    elseif((rawdata_x(i)>0)&&(rawdata_y(i)<0))
        raw_fai(i)=(-atan(rawdata_y(i)/rawdata_x(i))*180/pi);
    elseif((rawdata_x(i)>0)&&(rawdata_y(i)>0))
        raw_fai(i)=(-atan(rawdata_y(i)/rawdata_x(i))*180/pi)+360;
    end
end
true_fai2=zeros(1,length(rawdata_x));
for i=1:length(rawdata_x)
    if((truedata2_x(i)==0)&&(truedata2_y(i)<0))
        true_fai2(i)=90;
    elseif((truedata2_x(i)==0)&&(truedata2_y(i)>0))
        true_fai2(i)=270;
    elseif((truedata2_x(i)<0))
        true_fai2(i)=(-atan(truedata2_y(i)/truedata2_x(i))*180/pi)+180;
    elseif((truedata2_x(i)>0)&&(truedata2_y(i)<0))
        true_fai2(i)=(-atan(truedata2_y(i)/truedata2_x(i))*180/pi);
    elseif((truedata2_x(i)>0)&&(truedata2_y(i)>0))
        true_fai2(i)=(-atan(truedata2_y(i)/truedata2_x(i))*180/pi)+360;
    end
end 
grid on;
hold on;
plot3(truedata2_x,truedata2_y,truedata2_z,'ro','MarkerSize',2);
axis equal;