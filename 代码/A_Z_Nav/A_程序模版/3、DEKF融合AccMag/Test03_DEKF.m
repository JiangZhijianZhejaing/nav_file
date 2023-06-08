%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��DEKF�ں�Gyro\Acc\Mag(��)
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                            % ����������
clear;                          % ��������
addpath(genpath('../../'));     % �������ļ�������m�ļ�
% gvar;                         % ���ص������
CalibParm_No1;                  % ����1��ģ��궨����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
%%
% figure(1)
% subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
% subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
% subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%%
Sample_Interval = 0.01;                                 % Gyro�������
% ��ʼavp
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % ȡǰ3��ƽ��Ϊ��ʼ��̬ŷ����
X_ksub1 = Eul2Qnb(Eul_AccMag_Init);                     % ��ʼ��̬��Ԫ��

Eul_Set = zeros(Marg_Number,3);
% kalman��ʼ��
P_ksub1 = (1e-1)*eye(4);
Q_ksub1 = (1e-9)*eye(4);      % ״̬����������
R_Acc_ksub1 = (1e-1)*eye(3);  % ���ٶȼ���������������
R_Mag_ksub1 = (1e-1)*eye(3);  % ���ٶȼ���������������

Beta_ksub1 = 1;
b = 0.95;

for k = 1:Marg_Number
    
    [X_k, P_k, Beta_k, R_Acc_k, R_Mag_k] = Dekf...
        (X_ksub1, Gyro_Set, Acc_Set, Mag_Set, Sample_Interval, P_ksub1, Q_ksub1, R_Acc_ksub1, R_Mag_ksub1, k, b, Beta_ksub1);
    
    Eul = Qnb2Eul(X_k);
    Eul_Set(k,:) = Eul;
    
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_Acc_ksub1 = R_Acc_k;
    R_Mag_ksub1 = R_Mag_k;
end
%%
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');




