%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��DCF�ں�Gyro\Acc\Mag (���������˲�)
%
% By Taoran Zhao
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------һ���궨���ݲ����㾲̬��̬��---------------
clc;                            % ����������
clear;                          % ��������
addpath(genpath('../../'));     % �������ļ�������m�ļ�
% gvar;                         % ���ص������
CalibParm_No1;                  % ����1��ģ��궨����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
% -----Eul_AccMag_Set��ͼ-----
% figure(1)
% subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
% subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
% subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%% ---------------����DCF��ʼ��---------------
Sample_Interval = 0.01;                                 % Gyro�������
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % ȡǰ3��ƽ��Ϊ��ʼ��̬ŷ����
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % ��ʼ��̬��Ԫ��(��)
% Eul_Set = zeros(Marg_Number,3);

Pos_ksub1 = [31.90209*pi/180;117.17*pi/180;15];         % ��ʼ��γ����(��GNSS���)���Ϸ�
gn = CalcGn(Pos_ksub1); 

h_Acc_k_Fun = @(q)([2*(q(2)*q(4) - q(1)*q(3)); 2*(q(3)*q(4) + q(1)*q(2)); q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2]);
h_Mag_k_Fun = @(q,mny,mnz)([2*(q(2)*q(3) + q(1)*q(4))*mny + 2*(q(2)*q(4) - q(1)*q(3))*mnz;
                            (q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2)*mny + 2*(q(3)*q(4) + q(1)*q(2))*mnz;
                            2*(q(3)*q(4) - q(1)*q(2))*mny + (q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2)*mnz]);
u_a = 0.2;
u_m = 0.2;
%% ---------------����DCF--------------
for k = 1:Marg_Number
    W = Gyro_Set(k,:)*pi/180;  
    Wx = W(1);   Wy = W(2);   Wz = W(3); 
    Qnb_k_Gyro = (eye(4)+0.5*Sample_Interval*...
        [0,   -Wx,  -Wy,  -Wz; Wx,  0,    Wz,   -Wy; Wy,  -Wz,  0,    Wx; Wz,  Wy,   -Wx,  0 ])*Qnb_ksub1;
    Qnb_k_Gyro = NormlzQnb(Qnb_k_Gyro);
    % -----1��һ�������˲�-----                                                                          
    a = NormlzV3(Acc_Set(k,:)');                                           % ���ٶ����� ��һ��                                     
    a_hat = NormlzV3(h_Acc_k_Fun(Qnb_k_Gyro));                             % ���ٶȹ��� ��һ��
    n_a0 = cross(a_hat, a); 
    n_a = NormlzV3(n_a0);
    dot_a = dot(a_hat, a);
    if abs(dot_a - 1) < 1e-15
        dot_a = 1;
    end  
    Del_a = -acos(dot_a); 
    Q_ae0 = n_a*sin(u_a*Del_a/2);
    Q_ae = [cos(u_a*Del_a/2); Q_ae0];                                      % �����Ԫ��
    Q_a = QnbMulQnb(Qnb_k_Gyro, Q_ae);
    % -----2�����������˲�-----
    mb = NormlzV3(Mag_Set(k,:)');
    mn_hat0 = QnbMulV(Q_a, mb);
    mn_hat0(3) = 0;
    mn_hat = NormlzV3(mn_hat0);
    mn = [0; 1; 0];
    n_m0 = cross(mn_hat, mn); 
    n_m = NormlzV3(n_m0);
    dot_m = dot(mn_hat, mn);
    Del_m = acos(dot_m);    
    Q_me0 = n_m*sin(u_m*Del_m/2);
    Q_me = [cos(u_m*Del_m/2);  Q_me0];
    Qnb_k = QnbMulQnb(Q_a, Q_me);
    Qnb_k = NormlzQnb(Qnb_k);    
    Eul = Qnb2Eul(Qnb_k);
    Eul_Set(k,:) = Eul;
    Qnb_ksub1 = Qnb_k;
end
%%
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');