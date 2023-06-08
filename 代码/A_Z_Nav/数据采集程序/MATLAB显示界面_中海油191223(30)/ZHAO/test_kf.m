%Õ÷«Ú∑Ω≥Ã
theta=[0,0,0,0,0,0,0,0,0]';
fai=[0,0,0,0,0,0,0,0,0];
P=10401*eye(9);
change=zeros(1,length(datax));
for i=1:length(datax)
    fai(1)=datax(i)*datax(i);
    fai(2)=datay(i)*datay(i);
    fai(3)=dataz(i)*dataz(i);
    fai(4)=datax(i)*datay(i);
    fai(5)=datax(i)*dataz(i);
    fai(6)=datay(i)*dataz(i);
    fai(7)=datax(i);
    fai(8)=datay(i);
    fai(9)=dataz(i);
    K=(fai*P/(1+fai*P*fai'))';
    theta=theta+K*(1-fai*theta);
    P=(eye(9)-K*fai)*P;
    change(i)=P(1);
    end
theta;
xlimit=[-3000,3000];
ylimit=[-3000,3000];
zlimit=[-3000,3000];
gd=[30,30,30];
f=@(x,y,z)theta(1)*x.^2+theta(2)*y.^2+theta(3)*z.^2+theta(4)*x.*y+theta(5)*x.*z+theta(6)*y.*z+theta(7)*x+theta(8)*y+theta(9)*z-1;
x=linspace(xlimit(1),xlimit(2),gd(1));
y=linspace(ylimit(1),ylimit(2),gd(2));
z=linspace(zlimit(1),zlimit(2),gd(3));
[x,y,z]=meshgrid(x,y,z);
axis equal;
val=f(x,y,z);
[f,v]=isosurface(x,y,z,val,0);
p=patch('Faces',f,'Vertices',v,'CData',v(:,3),'facecolor','none','EdgeColor','flat');
view(3);
grid on;
hold on;
plot3(datax,datay,dataz,'ro','MarkerSize',2);
axis equal;
hold on;
plot(P);
    