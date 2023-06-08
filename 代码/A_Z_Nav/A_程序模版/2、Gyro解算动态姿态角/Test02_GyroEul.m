%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function���������ǽ�����̬
%
% By Taoran Zhao
% 2023/04/03
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;                            % ����������
clear;                          % ��������
addpath(genpath('../../'));     % �������ļ�������m�ļ�
CalibParm_No1;                  % ����1��ģ��궨����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
% -----��ͼ-----
% figure(1)
% subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
% subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
% subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%%
Sample_Interval = 0.01;                                 % Gyro�������
% ��ʼavp
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % ȡǰ3��ƽ��Ϊ��ʼ��̬ŷ����
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % ��ʼ��̬��Ԫ��
Qnb_Set = zeros(4,Marg_Number);
Eul_Set = zeros(Marg_Number,3);
for k = 1:Marg_Number
    Qnb_k = Rungekuta(Qnb_ksub1, Sample_Interval, k, Gyro_Set, Qnb_Set);   % �Ľ��������
    Eul = Qnb2Eul(Qnb_k);
    Qnb_Set(:,k) = Qnb_k;
    Eul_Set(k,:) = Eul;
    Qnb_ksub1 = Qnb_k;
end
% -----��ͼ-----
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');
