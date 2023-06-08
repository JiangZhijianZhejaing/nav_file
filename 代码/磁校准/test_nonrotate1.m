%��������
% datax=xlsread('2022081604.xls','G2:G3231');
% datay=xlsread('2022081604.xls','H2:H3231');
% dataz=xlsread('2022081604.xls','I2:I3231');
datax=xlsread('328.csv','H2:H3300')';
datay=xlsread('328.csv','I2:I3300')';
dataz=xlsread('328.csv','J2:J3300')';



%�������
H=zeros(length(datax),6);%���ݾ���
for i=1:length(datax)
    H(i,1)=datax(i)*datax(i);
    H(i,2)=datay(i)*datay(i);
    H(i,3)=dataz(i)*dataz(i);
    H(i,4)=2*datax(i);
    H(i,5)=2*datay(i);
    H(i,6)=2*dataz(i);
    %������ϣ�����ת��ɼ�������Ӧ�����Ǻ�
end
F=ones(length(datax),1);%�Ҳ��1����
%HA=1��A���������ϵ��
E_para=((inv(H'*H))*H')*F;
a1=E_para(1);
a2=E_para(2);
a3=E_para(3);
a4=E_para(4);
a5=E_para(5);
a6=E_para(6);
%��������
o1=a4/a1;
o2=a5/a2;
o3=a6/a3;
G=1+a4*a4/a1+a5*a5/a2+a6*a6/a3;
g1=sqrt(a1/G);
g2=sqrt(a2/G);
g3=sqrt(a3/G);
%У��
rawdata_x=[-30.9839	 -51.2038   -66.4154  -67.835625  -52.437125  -32.4415    -16.8935  -16.07125 ];
% rawdata_x = rawdata_x(1,:)+45.5393;
rawdata_y=[57.48275 58.1555  42.3825  21.45325 6.839625 6.465875 20.85525   43.5045 ];
% rawdata_y= rawdata_y(1,:)-31.474736184744213;
rawdata_z=[-12.1095 -12.4085 -12.2216 -12.7075 -11.847875 -11.8105 -12.03475 -12.670125];
truedata2=[(rawdata_x(1,:)+o1)*g1; (rawdata_y(1,:)+o2)*g2; (rawdata_z(1,:)+o3)*g3];
rawdata_x = truedata2(1,:);
rawdata_y = truedata2(2,:);
rawdata_z = truedata2(3,:);
truedata2T = truedata2'*truedata2;
truedata2_x=truedata2(1,:);
truedata2_y=truedata2(2,:);
truedata2_z=truedata2(3,:);
%����
raw_fai=zeros(1,length(rawdata_x));
for i=1:length(rawdata_x);
%     if((rawdata_x(i)==0)&&(rawdata_y(i)<0))
%         raw_fai(i)=90;
%     elseif((rawdata_x(i)==0)&&(rawdata_y(i)>0))
%         raw_fai(i)=270;
%     elseif((rawdata_x(i)<0))
%         raw_fai(i)=(-atan(rawdata_y(i)/rawdata_x(i))*180/pi)+180;
%     elseif((rawdata_x(i)>0)&&(rawdata_y(i)<0))
%         raw_fai(i)=(-atan(rawdata_y(i)/rawdata_x(i))*180/pi);
%     elseif((rawdata_x(i)>0)&&(rawdata_y(i)>0))
%         raw_fai(i)=(-atan(rawdata_y(i)/rawdata_x(i))*180/pi)+360;
%     end
    if((rawdata_y(i)==0)&&(rawdata_x(i)<0))
        raw_fai(i)=90;
    elseif((rawdata_y(i)==0)&&(rawdata_x(i)>0))
        raw_fai(i)=270;
    elseif((rawdata_y(i)<0))
        raw_fai(i)=(-atan(rawdata_x(i)/rawdata_y(i))*180/pi)+180;
    elseif((rawdata_y(i)>0)&&(rawdata_x(i)<0))
        raw_fai(i)=(-atan(rawdata_x(i)/rawdata_y(i))*180/pi);
    elseif((rawdata_y(i)>0)&&(rawdata_x(i)>0))
        raw_fai(i)=(-atan(rawdata_x(i)/rawdata_y(i))*180/pi)+360;
    end
end
for i = 1:7
    sub(i) = raw_fai(i+1) - raw_fai(i);
end

true_fai2=zeros(1,length(rawdata_x));
for i=1:length(rawdata_x);
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
M1=[(datax(1,:)-o1)/g1; (datay(1,:)-o2)/g2; (dataz(1,:)-o3)/g3];
datax_a=M1(1,:);
datay_a=M1(2,:);
dataz_a=M1(3,:);
plot3(datax_a,datay_a,dataz_a,'bo','MarkerSize',1);
grid on;
hold on;
%plot3(datax,datay,dataz,'ro','MarkerSize',1);
axis equal;