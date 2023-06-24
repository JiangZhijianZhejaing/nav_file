%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：运用误差状态Kalman滤波融合Marg姿态角(解耦版)
%
% notice：位置采用大地坐标系(即，经纬度)
%
% By Taoran Zhao
% 2023/04/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------一、导入数据---------------
clc;                            % 清理命令行
clear;                          % 清理工作区
addpath(genpath('../../'));     % 导入系统文件夹所有文件
CalibParm_No1_20230301;                    % 加载1号模块标定参数
% ---------------二、标定数据---------------
[Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
    MargCalib('marg1', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargENU(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);
% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% ---------------五、作图---------------
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('俯仰角 Pitch'),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\theta');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('横滚角 Roll '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\gamma');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\psi');
sgtitle('姿态角量测');
%% ---------------六、ESKF初始化---------------
Sample_Interval = 0.01;                          % Gyro采样间隔
% ----------1、初始avp----------
% -----（1）att-----
Eul_Init = mean(Eul_AccMag_Set(50:200,:));       % 初始姿态欧拉角，取第50个到第200个欧拉角平均
Qua_Init = Eul2Qua(Eul_Init);                    % 初始姿态四元数
Qua_ksub1 = Qua_Init;                   
% -----（2）vel-----
V_Init = [0;0;0];                                % 初始速度
V_ksub1 = V_Init;                               
% -----（3）pos-----
Place_Init = [0; 0; 0];                          % 初始当地直角坐标系位置
Place_ksub1 = Place_Init; 
Pos_Init = [31.90209*pi/180; 117.17*pi/180; 15]; % 初始经纬坐标(由GNSS获得) 合肥 [纬度 经度 高度]
Pos_ksub1 = Pos_Init;
% ----------2、分配内存----------
Eul_Set = zeros(Marg_Number,3);                  % 提前分配Eul_Gyro_Set内存
Qua_Set = zeros(Marg_Number,4);                  % 提前分配Qua_Set内存
V_Set = zeros(Marg_Number,3);
Pos_Set = zeros(Marg_Number,3);
Place_Set = zeros(Marg_Number,3);
% gn = CalcGn(Pos_Init);   
% ----------3、kalman初始化----------
X_ksub1 = zeros(15,1);
P_ksub1 = eye(15);
Q_ksub1 = (1e-9)*eye(15);                     % 状态噪声方差阵
R_ksub1 = (1e-1)*eye(3);                     % 加速度计量测噪声方差阵
H_k = [eye(3),zeros(3,12)];
Beta_ksub1 = 1;
% b = 0.95;

%% ---------------三、ESKF--------------
for k = 1:Marg_Number
    % ----------1、惯导更新----------
    [Qnb_k_0, V_k_0, Pos_k_0, fn_ksub1] = SinsUpdate...
        (Qnb_ksub1, V_ksub1, Pos_ksub1, Acc_Set, Gyro_Set, Qnb_Set, k);    % 得到未修正的姿态、位置、速度更新
    Eul_k_0 = Qnb2Eul(Qnb_k_0);           % 未修正的姿态欧拉角
    % ----------2、eskf：得到修正值----------
    Eul_AccMag = Eul_AccMag_Set(k,:)';    % k时刻 静态姿态欧拉角
    [X_k, P_k, R_k, Beta_k] = Eskf...
        (fn_ksub1, Qnb_ksub1, V_ksub1, Pos_ksub1, X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, R_ksub1, Eul_AccMag, Qnb_k_0);
    % ----------3、avp修正值----------
    Eul_k = Eul_k_0 - X_k(1:3);             % 修正姿态欧拉角
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    Qnb_Set(:,k) = Qnb_k;
    V_k = V_k_0 - X_k(4:6);                 % 修正速度
    V_Set(k,:) = V_k;
    Pos_k = Pos_k_0 - X_k(7:9);         % 修正位置
    Pos_Set(k,:) = Pos_k;               % [纬度 经度 高度]
    Place_k = Pos2Place(Pos_k, Pos_Init);
    Place_Set(k,:) = Place_k;
    % ----------4、状态归0----------
    X_k(1:9) = 0;
    
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    Qnb_ksub1 = Qnb_k;
    V_ksub1 = V_k;
    Pos_ksub1 = Pos_k;
end
% ----------画图----------
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('俯仰角 Pitch'),xlabel('时间 t/s'),ylabel('A/°'),legend('\theta');
subplot(3,1,2),plot(Eul_Set(:,2)),title('横滚角 Roll '),xlabel('时间 t/s'),ylabel('A/°'),legend('\gamma');
subplot(3,1,3),plot(Eul_Set(:,3)),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('A/°'),legend('\psi');
sgtitle('姿态角');
figure(3)
subplot(3,1,1),plot(V_Set(:,1)),title('东向速度'),xlabel('时间 t/s'),ylabel(' V/(m/s )'),legend('V\it_E');
subplot(3,1,2),plot(V_Set(:,2)),title('北向速度'),xlabel('时间 t/s'),ylabel(' V/(m/s )'),legend('V\it_N');
subplot(3,1,3),plot(V_Set(:,3)),title('天向速度'),xlabel('时间 t/s'),ylabel(' V/(m/s )'),legend('V\it_U');
sgtitle('速度');
figure(4)
subplot(3,1,1),plot(Pos_Set(:,1)),title('纬度'),xlabel('时间 t/s'),ylabel('Pos/m'),legend('L');
subplot(3,1,2),plot(Pos_Set(:,2)),title('经度'),xlabel('时间 t/s'),ylabel('Pos/m'),legend('\lambda');
subplot(3,1,3),plot(Pos_Set(:,3)),title('高度'),xlabel('时间 t/s'),ylabel('Pos/m'),legend('h');
sgtitle('位置(大地坐标系)');
figure(5)
subplot(3,1,1),plot(Place_Set(:,1)),title('东向'),xlabel('时间 t/s'),ylabel(' Place/m'),legend('X');
subplot(3,1,2),plot(Place_Set(:,2)),title('北向'),xlabel('时间 t/s'),ylabel(' Place/m'),legend('Y');
subplot(3,1,3),plot(Place_Set(:,3)),title('天向'),xlabel('时间 t/s'),ylabel(' Place/m'),legend('Z');
sgtitle('位置(当地直角坐标系)');


