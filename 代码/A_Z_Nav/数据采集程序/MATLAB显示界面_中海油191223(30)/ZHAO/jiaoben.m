    format long 
data = xlsread('Z+','sheet1');
m=ones(3,3);
m(1,1)=data(1,7);
m(1,2)=data(1,8);
m(1,3)=data(1,9);
m(2,1)=data(1,4);
m(2,2)=data(1,5);
m(2,3)=data(1,6);
m(3,1)=data(1,1);
m(3,2)=data(1,2);
m(3,3)=data(1,3);

row = size(data,1);
HX=data(:,7);
HY=data(:,8);
HZ=data(:,9);
T=data(:,10);
m1=m/30000
 
 
 ellip_data = [HX,HY,HZ];
 data1 = ellip_data*m1;
 [cc, r] = sphereFit(data1);
%   delete('Z+.xls');
%  cc=k*t+cc1;


