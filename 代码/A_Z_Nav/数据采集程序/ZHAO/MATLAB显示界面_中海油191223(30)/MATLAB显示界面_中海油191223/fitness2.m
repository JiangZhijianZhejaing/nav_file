function y=fitness2(x)
% datax=xlsread('2022081604.xls','G1000:G1200');
% datay=xlsread('2022081604.xls','H1000:H1200');
% dataz=xlsread('2022081604.xls','I1000:I1200');
datax=xlsread('03241.xls','G2:G3570');
datay=xlsread('03241.xls','H2:H3570');
dataz=xlsread('03241.xls','I2:I3570');
a1 = x(1);
a2 = x(2);
a3 = x(3);
a4 = x(4);
a5 = x(5);
a6 = x(6);
a7 = x(7);
a8 = x(8);
a9 = x(9);
% fitting function
yf=a1.*datax.*datax+a2.*datay.*datay+a3.*dataz.*dataz+a4.*datax.*datay+a5.*datax.*dataz+a6.*datay.*dataz+a7.*datax+a8.*datay+a9.*dataz;
y=sum(abs(yf-1))/length(datax);
end