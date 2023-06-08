clc;
close all;
Pop = 400;%遗传算法的种群数量
tol = 1e-6;%允许误差
p1 = 0.5;%变异率
p2 = 0.5;%交叉率
MAX = 100;%最大种群数量
%设置ga工具箱参数
options = optimoptions('ga','ConstraintTolerance',tol,'PlotFcn', @gaplotbestf,'MigrationFraction',p1,'CrossoverFraction', p2,'PopulationSize', Pop, 'Generations', MAX,'Display','iter');
nvars = 9;%变量个数
A = [];%线性不等式约束系数矩阵
b = [];%线性不等式约束增广矩阵
Aeq = [];%线性等式约束系数矩阵
beq = [];%线性等式约束增广矩阵
lb = [1e-07 1e-07 1e-08 1e-08 1e-09 -1e-07 1e-05 -1e-04 1e-05]';%变量下界
ub = [1e-06 1e-06 1e-07 1e-07 1e-08 -1e-08 1e-04 -1e-05 1e-04]';%变量上界
nonlcon = [];%非线性约束条件
[x,fval] = ga(@fitness2,nvars,A,b,Aeq,beq,lb,ub,nonlcon,options)
%a1,a2,a3,a4,a5,a6,a7,a8,a9,a1eq,a2eq,a3eq,a4eq,a5eq,a6eq,a7eq,a8eq,a9eq,
% A = x(1);
% b = x(2);
% xx =0:0.1:10;
% yy = fun1(xx,A,b);
% x_0 = 0:1:10;
% y_0 = [1,2.68294196961579,2.81859485365136,1.28224001611973,-0.513604990615856,-0.917848549326277,0.441169003602148,2.31397319743758,2.97871649324676,1.82423697048351,-0.0880422217787396];
% figure;
% plot(x_0,y_0,'c-o');
% hold on;
% plot(xx,yy,'b-');
% xlabel('x');
% legend('原始数据','拟合数据');
% grid on;