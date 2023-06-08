%求校正系数及矩阵：乔里斯基分解
%A=inv(k)'*inv(k)
%由公式直接得A,b
A1=[a1 (a4/2) (a5/2); 
  (a4/2) a2 (a6/2); 
  (a5/2) (a6/2) a3];
%A_inv=inv(A);
b1=-inv(A1)*[a7/2;
            a8/2;
            a9/2];
%分解A得inv(k)，为下三角阵
k_chol1=chol(A1);
k_chol2=k_chol1';
M1=k_chol1*[datax(1,:)-b1(1); datay(1,:)-b1(2); dataz(1,:)-b1(3)];
datax_a=M1(1,:);
datay_a=M1(2,:);
dataz_a=M1(3,:);
plot3(datax_a,datay_a,dataz_a,'bo','MarkerSize',1);
grid on;
hold on;
%plot3(datax,datay,dataz,'ro','MarkerSize',1);
axis equal;