%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function���������״̬Kalman�˲��ں϶���̬��̬��(�ͳɱ���)
%
% notice��λ�ò��ô������ϵ(������γ��)
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------һ���궨���ݲ����㾲̬��̬��---------------
clc;                            % ����������
clear;                          % ��������
addpath(genpath('../../'));     % �������ļ�������m�ļ�
gvar;
CalibParm_No1;                  % ����1��ģ��궨����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
% -----��ͼ-----
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('������ Pitch'),xlabel('ʱ�� t/s'),ylabel('�Ƕ� A/��'),legend('\theta');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('����� Roll '),xlabel('ʱ�� t/s'),ylabel('�Ƕ� A/��'),legend('\gamma');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('����� Yaw '),xlabel('ʱ�� t/s'),ylabel('�Ƕ� A/��'),legend('\psi');
sgtitle('��̬������');
%% ---------------����ESKF��ʼ��---------------
% Sample_Interval = 0.01;                                 % Gyro�������
% ----------1����ʼavp----------
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % ȡǰ3��ƽ��Ϊ��ʼ��̬ŷ����
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % ��ʼ��̬��Ԫ��
V_ksub1 = [0;0;0];                                      % ��ʼ�ٶ�
Place_ksub1 = [0; 0; 0];                                % ��ʼ����ֱ������ϵλ��
Pos_Init = [31.90209*pi/180; 117.17*pi/180; 15];        % ��ʼ��γ����(��GNSS���)���Ϸ� [γ�� ���� �߶�]
% gn = CalcGn(Pos_Init);   
Pos_ksub1 = Pos_Init;
Qnb_Set = zeros(4,Marg_Number);                         % Ԥ����Qnb_Set�ڴ�
Eul_Set = zeros(Marg_Number,3);
V_Set = zeros(Marg_Number,3);
Pos_Set = zeros(Marg_Number,3);
Place_Set = zeros(Marg_Number,3);
% ----------2��kalman��ʼ��----------
X_ksub1 = zeros(15,1);
P_ksub1 = eye(15);
Q_ksub1 = (1e-9)*eye(15);                     % ״̬����������
R_ksub1 = (1e-1)*eye(3);                     % ���ٶȼ���������������
H_k = [eye(3),zeros(3,12)];
Beta_ksub1 = 1;
% b = 0.95;

%% ---------------����ESKF--------------
for k = 1:Marg_Number
    % ----------1���ߵ�����----------
    [Qnb_k_0, V_k_0, Pos_k_0, fn_ksub1] = SinsUpdate...
        (Qnb_ksub1, V_ksub1, Pos_ksub1, Acc_Set, Gyro_Set, Qnb_Set, k);    % �õ�δ��������̬��λ�á��ٶȸ���
    Eul_k_0 = Qnb2Eul(Qnb_k_0);           % δ��������̬ŷ����
    % ----------2��eskf���õ�����ֵ----------
    Eul_AccMag = Eul_AccMag_Set(k,:)';    % kʱ�� ��̬��̬ŷ����
    [X_k, P_k, R_k, Beta_k] = Eskf...
        (fn_ksub1, Qnb_ksub1, V_ksub1, Pos_ksub1, X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, R_ksub1, Eul_AccMag, Qnb_k_0);
    % ----------3��avp����ֵ----------
    Eul_k = Eul_k_0 - X_k(1:3);             % ������̬ŷ����
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    Qnb_Set(:,k) = Qnb_k;
    V_k = V_k_0 - X_k(4:6);                 % �����ٶ�
    V_Set(k,:) = V_k;
    Pos_k = Pos_k_0 - X_k(7:9);         % ����λ��
    Pos_Set(k,:) = Pos_k;               % [γ�� ���� �߶�]
    Place_k = Pos2Place(Pos_k, Pos_Init);
    Place_Set(k,:) = Place_k;
    % ----------4��״̬��0----------
    X_k(1:9) = 0;
    
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    Qnb_ksub1 = Qnb_k;
    V_ksub1 = V_k;
    Pos_ksub1 = Pos_k;
end
% ----------��ͼ----------
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('������ Pitch'),xlabel('ʱ�� t/s'),ylabel('A/��'),legend('\theta');
subplot(3,1,2),plot(Eul_Set(:,2)),title('����� Roll '),xlabel('ʱ�� t/s'),ylabel('A/��'),legend('\gamma');
subplot(3,1,3),plot(Eul_Set(:,3)),title('����� Yaw '),xlabel('ʱ�� t/s'),ylabel('A/��'),legend('\psi');
sgtitle('��̬��');
figure(3)
subplot(3,1,1),plot(V_Set(:,1)),title('�����ٶ�'),xlabel('ʱ�� t/s'),ylabel(' V/(m/s )'),legend('V\it_E');
subplot(3,1,2),plot(V_Set(:,2)),title('�����ٶ�'),xlabel('ʱ�� t/s'),ylabel(' V/(m/s )'),legend('V\it_N');
subplot(3,1,3),plot(V_Set(:,3)),title('�����ٶ�'),xlabel('ʱ�� t/s'),ylabel(' V/(m/s )'),legend('V\it_U');
sgtitle('�ٶ�');
figure(4)
subplot(3,1,1),plot(Pos_Set(:,1)),title('γ��'),xlabel('ʱ�� t/s'),ylabel('Pos/m'),legend('L');
subplot(3,1,2),plot(Pos_Set(:,2)),title('����'),xlabel('ʱ�� t/s'),ylabel('Pos/m'),legend('\lambda');
subplot(3,1,3),plot(Pos_Set(:,3)),title('�߶�'),xlabel('ʱ�� t/s'),ylabel('Pos/m'),legend('h');
sgtitle('λ��(�������ϵ)');
figure(5)
subplot(3,1,1),plot(Place_Set(:,1)),title('����'),xlabel('ʱ�� t/s'),ylabel(' Place/m'),legend('X');
subplot(3,1,2),plot(Place_Set(:,2)),title('����'),xlabel('ʱ�� t/s'),ylabel(' Place/m'),legend('Y');
subplot(3,1,3),plot(Place_Set(:,3)),title('����'),xlabel('ʱ�� t/s'),ylabel(' Place/m'),legend('Z');
sgtitle('λ��(����ֱ������ϵ)');


