%%% 绘制椭球面
mag_org = xlsread('spfit2.xls','G:I');

m = [1.000807643033786  -0.001833977741707  -0.007328404663079
  -0.038547210996318   1.000014396322138   0.015905096095473
   0.001080677594225  -0.004306339151093   1.000073476039956];
k = [2195.3073,2312.1973,2415.764]; %20191208(海油1cz）   
b = [187.8362,142.5941,-51.4256]; %20191208(海油1cz）
kInv = inv(diag(k));
mb = kInv * m; % m'*kInv;
cc = b*mb;

bx=(mag_org(:,1)-b(1))/k(1);
by=(mag_org(:,2)-b(2))/k(2);
bz=(mag_org(:,3)-b(3))/k(3);
mag_norm = [bx by bz];   %归一化值
mag_cbd = mag_norm*m;   %归一化后进行非正交修正
data = mag_org*mb;
% data(:,1)=data(:,1) - cc(1); 
% data(:,2)=data(:,2) - cc(2);
% data(:,3)=data(:,3) - cc(3);
[cc1, r] = sphereFit(data)

figure(1);plot3(mag_org(:,1),mag_org(:,2),mag_org(:,3));grid on;
xlabel('X-axis'),ylabel('Y-axis'),zlabel('Z-axis');axis equal;  % square;
figure(2);plot3(data(:,1),data(:,2),data(:,3));grid on;
xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');axis equal;
figure(3);plot3(mag_cbd(:,1),mag_cbd(:,2),mag_cbd(:,3));grid on;
xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');axis equal;
