%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��DUKF�ں�Gyro\Acc\Mag
%
% notice��P�ĳ�ʼֵ���˹������׷�����
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------1���궨���ݲ����㾲̬��̬��---------------
clc;                            % ����������
clear;                          % ��������
addpath(genpath('../../'));     % �������ļ�������m�ļ�
% gvar;                         % ���ص������
CalibParm_No1;                  % ����1��ģ��궨����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);

figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%% ---------------2��DUKF��ʼ��---------------
Sample_Interval = 0.01;                                 % Gyro�������
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % ȡǰ3��ƽ��Ϊ��ʼ��̬ŷ����
X_ksub1 = Eul2Qnb(Eul_AccMag_Init);                     % ��ʼ��̬��Ԫ��
Eul_Set = zeros(Marg_Number,3);                         % Eul_Set ��ǰ�����ڴ�ռ� 
% -----���㵱��gnֵ-----
Pos_ksub1 = [[31.90209; 117.17]*pi/180; 15];            % ��ʼ��γ����(��GNSS���)���Ϸ�
gn = CalcGn(Pos_ksub1);                                 
% -----kalman��ʼ��-----
P_ksub1 = (1e-3)*eye(4);      % ��Ҫ��Ĵ��ڵ�λ���󣬻�ʧȥ������(2023/04/06)
Q_ksub1 = (1e-5)*eye(4);      % ״̬����������
R_Acc_ksub1 = (1e-2)*eye(3);  % ���ٶȼ���������������
R_Mag_ksub1 = (1e-2)*eye(3);  % ���ٶȼ���������������
h_Acc_k_Fun = @(q)([2*(q(2)*q(4) - q(1)*q(3)); 2*(q(3)*q(4) + q(1)*q(2)); q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2]);
h_Mag_k_Fun = @(q,mny,mnz)([2*(q(2)*q(3) + q(1)*q(4))*mny + 2*(q(2)*q(4) - q(1)*q(3))*mnz;
                            (q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2)*mny + 2*(q(3)*q(4) + q(1)*q(2))*mnz;
                            2*(q(3)*q(4) - q(1)*q(2))*mny + (q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2)*mnz]);

%% ---------------3��DUKF--------------
for k = 1:Marg_Number
    % -----����һ��״̬ת�ƾ���-----           
    Wxyz_ksub1 = Gyro_Set(k,:)*pi/180;   % ת����   
    Wx = Wxyz_ksub1(1);   Wy = Wxyz_ksub1(2);   Wz = Wxyz_ksub1(3);  
    PHI_k_ksub1 = eye(4) + 0.5*Sample_Interval*[0, -Wx, -Wy, -Wz; Wx, 0, Wz, -Wy; Wy, -Wz, 0, Wx; Wz, Wy, -Wx, 0];
    % -----DUKF-----
    [X_k,P_k] = Dukf(PHI_k_ksub1, X_ksub1, P_ksub1, Acc_Set, Mag_Set, Q_ksub1, R_Acc_ksub1, R_Mag_ksub1, k, h_Acc_k_Fun, h_Mag_k_Fun);
    Eul = Qnb2Eul(X_k);
    Eul_Set(k,:) = Eul;
    X_ksub1= X_k;
    P_ksub1 = P_k;
end
%%
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');