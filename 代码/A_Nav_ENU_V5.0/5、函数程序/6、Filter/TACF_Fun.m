function Eul_Set = TACF_Fun(File_Name, xa, xm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：TACF
%
% Prototype: Eul_Set = TACF_Fun(File_Name, xa, xm)
% Inputs: File_Name - 文件名
%         xa - 加速度误差阈值
%         xm - 磁误差阈值
% Output: Eul_Set - 融合欧拉角集
%
% By Taoran Zhao
% 2023/04/26 2023/05/06 2023/05/19 2023/05/26
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath(genpath('../../'));            % 导入系统文件夹所有文件
%% ###############一、数据导入及参数设定#############################
% ----------1、导入数据----------
CalibrationParameter_No1;              % 加载1号模块标定参数
% ----------2、采样率----------
SAMPLE_FREQUENCY = 100;                % 100Hz    

% ########################################################################
%% ---------------二、标定数据---------------
% [Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
%     MargCalibrationNoZero(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.R);
[Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS, Marg_Number] = ...
    MargCalibration_Sun_NoZero(File_Name, 'Sheet1', CalibrationParameter.Wp, CalibrationParameter.P, CalibrationParameter.Mb, CalibrationParameter.B);
% 角速度阈值内未归零
%% ---------------三、转换东北天坐标系---------------
[Gyro_Set, Acc_Set, Mag_Set] = MargEnuUnnorm(Gyro_Set_InitCS, Acc_Set_InitCS, Mag_Set_InitCS); % 加速度、磁数据取消归一化   
%% ---------------四、解算静态姿态欧拉角---------------
Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
%% ---------------五、TCF初始化---------------
Sample_Interval = 1/SAMPLE_FREQUENCY;            % 采样间隔0.01s
% ----------1、初始att----------
Eul_Init = mean(Eul_AccMag_Set(1:300,:));           % 初始姿态欧拉角
Qnb_Init = Eul2Qnb(Eul_Init);                       % 初始姿态四元数Qnb                  
% ----------2、分配内存----------
Eul_Set = zeros(Marg_Number,3);                  % 提前分配Eul_Gyro_Set内存
% ----------3、DCF初始化----------
Qnb_ksub1 = Qnb_Init; 
mb0 = mean(Mag_Set(1:300,:))';
%% ---------------六、TCF---------------
for k = 1:Marg_Number
    % ----------1、滤波增益系数u_a----------
    gb = Acc_Set(k,:)';
    gb_mod = sqrt(gb'*gb);
    ea = abs(gb_mod - 1)/1;
    if ea > xa
        ea = xa;
    end
    u_a = 1-(1/xa)*ea;
    % ----------2、陀螺仪估计四元数Qua_k_Gyro----------
    W_Deg_ksub1 = Gyro_Set(k,:);
    % 一阶龙格库塔法
    Qnb_k_Gyro = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);    % k时刻 未校正的Qnb 陀螺仪估计四元数
    % ----------3、一级互补滤波---------- 
    % -----(1)载体加速度估计-----
    Cbn = Qnb2Cnb(Qnb_k_Gyro)';
    ab_hat = NormlzV3(Cbn*[0;0;1]);          % 归一化
    % -----(2)载体加速度量测-----
    ab = NormlzV3(Acc_Set(k,:)');
    % -----(3)确定旋转轴-----
    n_a0 = cross(ab, ab_hat); 
    n_a = NormlzV3(n_a0);                    % 绕n_a旋转
    % -----(4)解算Q_ae-----
    dot_a = dot(ab_hat, ab);                                                 
    if abs(dot_a - 1) < 1e-15
        dot_a = 1;
    end % MATLAB的计算精度问题可能会造成 dot_a > 1 
    Del_a = acos(dot_a);                     % 旋转角度
    Q_ae0 = n_a*sin(u_a*Del_a/2);
    Q_ae = [cos(u_a*Del_a/2); Q_ae0];        % 误差四元数
    % -----(5)解算Q_a-----
    Q_a = QuaMulQua(Qnb_k_Gyro,Q_ae);    
    % ----------2、二级互补滤波----------
    % -----(1)导航磁矢量估计-----
    mb = NormlzV3(Mag_Set(k,:)');
    mn_hat0 = Qnb2Cnb(Q_a)*mb;
    mn_hat0(3) = 0;
    mn_hat = NormlzV3(mn_hat0);
    % ----------(2)滤波增益系数u_m----------
    mb_k = Mag_Set(k,:)';
    mb_k_mod = sqrt(mb_k(1)^2 + mb_k(2)^2 + mb_k(3)^2);
    mb0_mod = sqrt(mb0(1)^2 + mb0(2)^2 + mb0(3)^2);
    em = abs(mb_k_mod - mb0_mod)/mb0_mod;
    if em > xm
        em = xm;
    end
    u_m = 1-(1/xm)*em;
    % -----(3)导航磁矢量量测-----
    mn = [0; 1; 0];
    % -----(4)确定旋转轴-----
    n_m0 = cross(mn_hat,mn); 
    n_m = NormlzV3(n_m0);
    % -----(5)解算Q_me-----
    dot_m = dot(mn, mn_hat);
    Del_m = acos(dot_m);                     % 旋转角度                                                
    Q_me0 = n_m*sin(u_m*Del_m/2);
    Q_me = [cos(u_m*Del_m/2);  Q_me0];
    % -----(6)解算Qnb_k-----
    Qnb_k = QuaMulQua(Q_me,Q_a);
    Qnb_k = NormlzQua(Qnb_k); 
    Eul = Qnb2Eul(Qnb_k);
    Eul_Set(k,:) = Eul;
    Qnb_ksub1 = Qnb_k;
end