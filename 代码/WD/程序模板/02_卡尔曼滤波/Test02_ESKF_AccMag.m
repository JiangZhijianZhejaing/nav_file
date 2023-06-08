%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function�����ڿ������˲�����̬�����㷨
%
% By Zhijian Jiang
% 2023/04/24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ---------------1���궨���ݲ����㾲̬��̬��---------------
addpath(genpath('../../'));     % �������ļ�������m�ļ�
clc;                            % ����������
clear;                          % ��������
gvar;                           % ���ص������
CalibParm_No1;          %���عߵ�����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
% -----Eul_AccMag_Set��ͼ-----
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%% ---------------2��ESKF��ʼ��--------------- 
Sample_Interval = 0.01;                                 % Gyro�������
% -----��ʼavp-----
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % ȡǰ3��ƽ��Ϊ��ʼ��̬ŷ����
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % ��ʼ��̬��Ԫ��
V_ksub1 = [0;0;0];                                      % ��ʼ�ٶ�
Place_ksub1 = [0; 0; 0];                                % ��ʼ����ֱ������ϵλ��
% -----���㵱��gnֵ-----
Pos_ksub1 = [31.90209*pi/180;117.17*pi/180;15];         % ��ʼ��γ����(��GNSS���)���Ϸ�
gn = CalcGn(Pos_ksub1);   
Qnb_Set = zeros(4,Marg_Number);                         % Ԥ����Qnb_Set�ڴ�
Eul_Set = zeros(Marg_Number,3);
V_Set = zeros(Marg_Number,3);
Place_Set = zeros(Marg_Number,3);
% -----kalman��ʼ��-----
X_ksub1 = zeros(15,1);
P_ksub1 = eye(15);
Q_ksub1 = (1e-9)*eye(15);                     % ״̬����������
R_ksub1 = (1e-1)*eye(3);                     % ���ٶȼ���������������
H_k = [eye(3),zeros(3,12)];
Beta_ksub1 = 1;
b = 0.95;





