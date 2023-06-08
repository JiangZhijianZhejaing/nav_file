% ˝æ›…Ÿ¡ø ±
datax=xlsread('03242.xls','G2:G4400');
datay=xlsread('03242.xls','H2:H4400');
dataz=xlsread('03242.xls','I2:I4400');
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
