% ˝æ›…Ÿ¡ø ±
datax=xlsread('0301.xls','sheet1','G2:G801');
datay=xlsread('0301.xls','sheet1','H2:H801');
dataz=xlsread('0301.xls','sheet1','I2:I801');
datax=datax';
datay=datay';
dataz=dataz';
% subplot(2,2,4)
% plot(datax,datay,'ro','MarkerSize',1);
% xlabel('x÷·');
% ylabel('y÷·');
% grid on;
% axis equal;
% box off
% subplot(2,2,2)
% plot(datay,dataz,'ro','MarkerSize',1);
% xlabel('y÷·');
% ylabel('z÷·');
% grid on;
% axis equal;
% box off
% subplot(2,2,3)
% plot(datax,dataz,'ro','MarkerSize',1);
% xlabel('x÷·');
% ylabel('z÷·');
% grid on;
% axis equal;
% box off
% subplot(2,2,1)
plot3(datax,datay,dataz,'bo','MarkerSize',1);
% xlabel('x÷·');
% ylabel('y÷·');
% zlabel('z÷·');
% grid on;
% axis equal;
% box off
