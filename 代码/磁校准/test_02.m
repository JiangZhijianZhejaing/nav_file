%拟合曲面
H=zeros(length(datax),9);%数据矩阵，x2，y2，z2，xy，xz...
for i=1:length(datax)
    H(i,1)=datax(i)*datax(i);
    H(i,2)=datay(i)*datay(i);
    H(i,3)=dataz(i)*dataz(i);
    H(i,4)=datax(i)*datay(i);
    H(i,5)=datax(i)*dataz(i);
    H(i,6)=datay(i)*dataz(i);
    H(i,7)=datax(i);
    H(i,8)=datay(i);
    H(i,9)=dataz(i);
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
a7=E_para(7);
a8=E_para(8);
a9=E_para(9);

%绘制曲面
% xlimit=[-3000,3000];
% ylimit=[-3000,3000];
% zlimit=[-3000,3000];
xlimit=[-300,300];
ylimit=[-300,300];
zlimit=[-300,300];
% xlimit=[-9000,9000];
% ylimit=[-9000,9000];
% zlimit=[-9000,9000];
gd=[50,50,50];
f=@(x,y,z)a1*x.^2+a2*y.^2+a3*z.^2+a4*x.*y+a5*x.*z+a6*y.*z+a7*x+a8*y+a9*z-1;
x=linspace(xlimit(1),xlimit(2),gd(1));
y=linspace(ylimit(1),ylimit(2),gd(2));
z=linspace(zlimit(1),zlimit(2),gd(3));
[x,y,z]=meshgrid(x,y,z);
val=f(x,y,z);
[f,v]=isosurface(x,y,z,val,0);
p=patch('Faces',f,'Vertices',v,'CData',v(:,3),'facecolor','none','EdgeColor','flat');
view(3);
grid on;
hold on;
plot3(datax,datay,dataz,'ro','MarkerSize',2);
axis equal;