% ˝æ›’π æ
% datax=xlsread('2022081604.xls','G2:G3231');
% datay=xlsread('2022081604.xls','H2:H3231');
% dataz=xlsread('2022081604.xls','I2:I3231');
% datax=xlsread('021301.xls','G6005:G6604');
% datay=xlsread('021301.xls','H6005:H6604');
% dataz=xlsread('021301.xls','I6005:I6604');
datax=xlsread('328.csv','H2:H3300')';
datay=xlsread('328.csv','I2:I3300')';
dataz=xlsread('328.csv','J2:J3300')';
% datax=xlsread('021301.xls','G2:G6001');
% datay=xlsread('021301.xls','H2:H6001');
% dataz=xlsread('021301.xls','I2:I6001');
datax=datax';
datay=datay';
dataz=dataz';
subplot(2,2,4)

plot(datax,datay,'ro','MarkerSize',1);
xlabel('x÷·');
ylabel('y÷·');
grid on;
axis equal;
box off
subplot(2,2,2)
plot(datay,dataz,'ro','MarkerSize',1);
xlabel('y÷·');
ylabel('z÷·');
grid on;
axis equal;
box off
subplot(2,2,3)
plot(datax,dataz,'ro','MarkerSize',1);
xlabel('x÷·');
ylabel('z÷·');
grid on;
axis equal;
box off
subplot(2,2,1)
plot3(datax,datay,dataz,'bo','MarkerSize',1);
xlabel('x÷·');
ylabel('y÷·');
zlabel('z÷·');
grid on;
axis equal;
box off
