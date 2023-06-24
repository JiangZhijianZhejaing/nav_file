%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：DCF融合Marg姿态 (二级互补滤波) + 自适应
%
% Notice：见《高宁,李杰,冯凯强,许廷金,高诗尧,李炳臻.基于双级互补滤波的姿态测量算法设计[J].传感技术学报,2019,32(12):1824-1829.》
%
% By Taoran Zhao
% 2023/04/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------一、导入数据---------------
clc;                            % 清理命令行
clear;                          % 清理工作区

tic
File = '0531_Roll';
xm = 1e-4;xa = 1e-3;
Eul_ADCF_Set = ADCF_Fun(File,xa,xm);
toc




% addpath(genpath('../../'));     % 导入系统文件夹所有文件
% CalibParm_No1;                  % 加载1号模块标定参数
% % ---------------二、标定数据---------------
% [Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs, Marg_Number] = ...
%     MargCalib_NoZero('0531_Roll', 'Sheet1', CalibParm.Wp, CalibParm.p, CalibParm.mb, CalibParm.R);
% % ---------------三、转换东北天坐标系---------------
% [Gyro_Set, Acc_Set, Mag_Set] = MargENU_UnNorm(Gyro_Set_InitCs, Acc_Set_InitCs, Mag_Set_InitCs);% 未归一化
% % ---------------四、解算静态姿态欧拉角---------------
% Eul_AccMag_Set = EulAccMag(Acc_Set, Mag_Set);
% %% ---------------五、DCF初始化---------------
% Sample_Interval = 0.01;                             % Gyro采样间隔
% % ----------1、初始avp----------
% Eul_Init = mean(Eul_AccMag_Set(1:300,:));           % 初始姿态欧拉角
% mb0 = mean(Mag_Set(1:100,:))';
% Qnb_Init = Eul2Qnb(Eul_Init);                       % 初始姿态四元数Qnb                  
% % ----------2、分配内存----------
% Eul_ADCF_Set = zeros(Marg_Number,3);                % 提前分配Eul_Gyro_Set内存 
% % ----------3、DCF初始化----------
% Qnb_ksub1 = Qnb_Init; 
% % ---------------六、DCF---------------
% for k = 1:Marg_Number
%     gb = Acc_Set(k,:)';
%     gb_mod = sqrt(gb'*gb);
%     ea = abs(gb_mod - 1)/1;
%     xa = 1e-3;
%     if ea > xa
%         ea = xa;
%     end
%     u_a = 1-(1/xa)*ea;
%     % ----------1、陀螺仪估计四元数Qua_k_Gyro----------
%     W_Deg_ksub1 = Gyro_Set(k,:);
%     % 一阶龙格库塔法
%     Qnb_k_Gyro = RungekutaOne(W_Deg_ksub1, Sample_Interval, Qnb_ksub1);    % k时刻 未校正的Qnb 陀螺仪估计四元数
%     % ----------2、一级互补滤波---------- 
%     % -----(1)载体加速度估计-----
%     Cbn = Qnb2Cnb(Qnb_k_Gyro)';
%     ab_hat = NormlzV3(Cbn*[0;0;1]);          % 归一化
%     % -----(2)载体加速度量测-----
%     ab = NormlzV3(Acc_Set(k,:)');
%     % -----(3)确定旋转轴-----
%     n_a0 = cross(ab, ab_hat); 
%     n_a = NormlzV3(n_a0);                    % 绕n_a旋转
%     % -----(4)解算Q_ae-----
%     dot_a = dot(ab_hat, ab);                                                 
%     if abs(dot_a - 1) < 1e-15
%         dot_a = 1;
%     end % MATLAB的计算精度问题可能会造成 dot_a > 1 
%     Del_a = acos(dot_a);                     % 旋转角度
%     Q_ae0 = n_a*sin(u_a*Del_a/2);
%     Q_ae = [cos(u_a*Del_a/2); Q_ae0];        % 误差四元数
%     % -----(5)解算Q_a-----
%     Q_a = QuaMulQua(Qnb_k_Gyro,Q_ae);    
%     % ----------2、二级互补滤波----------
%     % -----(1)导航磁矢量估计-----
%     mb = NormlzV3(Mag_Set(k,:)');
%     mn_hat0 = Qnb2Cnb(Q_a)*mb;
%     mn_hat0(3) = 0;
%     mn_hat = NormlzV3(mn_hat0);
%     % -----(2)导航磁矢量量测-----
%     mb_k = Mag_Set(k,:)';
%     mb_k_mod = sqrt(mb_k(1)^2 + mb_k(2)^2 + mb_k(3)^2);
%     mb0_mod = sqrt(mb0(1)^2 + mb0(2)^2 + mb0(3)^2);
%     em = abs(mb_k_mod - mb0_mod)/mb0_mod;
%     xm = 1e-4;
%     if em > xm
%         em = xm;
%     end
%     u_m = 1-(1/xm)*em;
% %     ccc(k) = u_m;
%     mn = [0; 1; 0];
%     % -----(3)确定旋转轴-----
%     n_m0 = cross(mn_hat,mn); 
%     n_m = NormlzV3(n_m0);
%     % -----(4)解算Q_me-----
%     dot_m = dot(mn, mn_hat);
%     Del_m = acos(dot_m);                     % 旋转角度                                                
%     Q_me0 = n_m*sin(u_m*Del_m/2);
%     Q_me = [cos(u_m*Del_m/2);  Q_me0];
%     % -----(5)解算Qnb_k-----
%     Qnb_k = QuaMulQua(Q_me,Q_a);
%     Qnb_k = NormlzQua(Qnb_k); 
%     Eul = Qnb2Eul(Qnb_k);
%     Eul_ADCF_Set(k,:) = Eul;
%     Qnb_ksub1 = Qnb_k;
% end
% Yaw_MSE0 = 0;
% for k = 1:Marg_Number
%    Yaw_E =  Eul_ADCF_Set(k,3) - Eul_Init(3);
%    Yaw_SE = Yaw_E^2;
%    Yaw_MSE0 = Yaw_MSE0 + Yaw_SE;
% end
% Yaw_MSE = Yaw_MSE0/Marg_Number;
% Yaw_RMSE = sqrt(Yaw_MSE);
% 
% Pitch_MSE0 = 0;
% for k = 1:Marg_Number
%    Pitch_E =  Eul_ADCF_Set(k,1) - Eul_Init(1);
%    Pitch_SE = Pitch_E^2;
%    Pitch_MSE0 = Pitch_MSE0 + Pitch_SE;
% end
% Pitch_MSE = Pitch_MSE0/Marg_Number;
% Pitch_RMSE = sqrt(Pitch_MSE);
% 
% 
% 
% % ---------------七、作图---------------
% % figure(1)
% % subplot(3,1,1),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,1)),title('俯仰角 Pitch'),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\theta');
% % subplot(3,1,2),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,2)),title('横滚角 Roll '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\gamma');
% % subplot(3,1,3),plot((1:Marg_Number)/100,Eul_AccMag_Set(:,3)),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('角度 A/°'),legend('\psi');
% % sgtitle('姿态角量测');
% figure(2)
% subplot(3,1,1),plot((1:Marg_Number)/100,Eul_ADCF_Set(:,1)),title('俯仰角 Pitch'),xlabel('时间 t/s'),ylabel('A/°'),legend('\theta');
% subplot(3,1,2),plot((1:Marg_Number)/100,Eul_ADCF_Set(:,2)),title('横滚角 Roll '),xlabel('时间 t/s'),ylabel('A/°'),legend('\gamma');
% subplot(3,1,3),plot((1:Marg_Number)/100,Eul_ADCF_Set(:,3)),title('航向角 Yaw '),xlabel('时间 t/s'),ylabel('A/°'),legend('\psi');
% sgtitle('DCF融合后的姿态角');