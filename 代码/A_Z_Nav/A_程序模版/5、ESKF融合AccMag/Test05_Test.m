%% ---------------1���궨���ݲ����㾲̬��̬��---------------
clc;                            % ����������
clear;                          % ��������
addpath(genpath('../../'));     % �������ļ�������m�ļ�
CalibParm_No1;                  % ����1��ģ��궨����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);
% -----��ͼ-----
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('Yaw');
%% ---------------2��ESKF��ʼ��---------------
Sample_Interval = 0.01;                                 % Gyro�������
% -----��ʼavp-----
Eul_AccMag_Init = mean(Eul_AccMag_Set(1:300,:));        % ȡǰ3��ƽ��Ϊ��ʼ��̬ŷ����
Qnb_ksub1 = Eul2Qnb(Eul_AccMag_Init);                   % ��ʼ��̬��Ԫ��
V_ksub1=[0;0;0];

Place_ksub1 = [31.90209*pi/180;117.17*pi/180;15];         % ��ʼ��γ����(��GNSS���)���Ϸ�
gn = CalcGn(Place_ksub1);   
Qnb_Set = zeros(4,Marg_Number);                         % Ԥ����Qnb_Set�ڴ�
Eul_Set = zeros(Marg_Number,3);
V_Set = zeros(Marg_Number,3);
Place_Set = zeros(Marg_Number,3);
% -----kalman��ʼ��-----
X_ksub1=zeros(15,1);
P_ksub1=eye(15);
Q_ksub1=(1e-9)*eye(15);                    % ״̬����������
R_ksub1 = (1e-1)*eye(3);                     % ���ٶȼ���������������
H_k = [eye(3),zeros(3,12)];
Beta_ksub1 = 1;
b = 0.95;

%% ---------------3��ESKF--------------
for k = 1:Marg_Number
    % -----(1)�ߵ�����-----
    [Qnb_k_0, V_k_0, Place_k_0, fn_ksub1] = SinsUpdateOld...
        (Qnb_ksub1, V_ksub1, Place_ksub1, gn, Acc_Set, Gyro_Set, Qnb_Set, Sample_Interval, k);      % �õ�δ��������̬��λ�á��ٶȸ���
    Eul_k_0 = Qnb2Eul(Qnb_k_0);           % δ��������̬ŷ����
    % -----(2)eskf���õ�����ֵ-----
    Eul_AccMag = Eul_AccMag_Set(k,:)';
    [X_k, P_k, R_k, Beta_k] = Eskf0(Sample_Interval, fn_ksub1, Qnb_k_0 , X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, b, R_ksub1, Eul_AccMag);
    % -----(3)avp����ֵ-----
    Eul_k = Eul_k_0 - X_k(1:3);             % ������̬ŷ����
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    Qnb_Set(:,k) = Qnb_k;
    V_k = V_k_0 - X_k(4:6);                 % �����ٶ�
    V_Set(k,:) = V_k;
    Place_k = Place_k_0 - X_k(7:9);         % ����λ��
    Place_Set(k,:) = Place_k;
    % -----(4)״̬��0-----
    X_k(1:9) = 0;
    
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    Qnb_ksub1 = Qnb_k;
    V_ksub1 = V_k;
    Place_ksub1 = Place_k;
end
% -----��ͼ-----
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('Pitch');
subplot(3,1,2),plot(Eul_Set(:,2)),title('Roll');
subplot(3,1,3),plot(Eul_Set(:,3)),title('Yaw');
figure(3)
subplot(3,1,1),plot(V_Set(:,1)),title('VE');
subplot(3,1,2),plot(V_Set(:,2)),title('VN');
subplot(3,1,3),plot(V_Set(:,3)),title('VU');
figure(4)
subplot(3,1,1),plot(Place_Set(:,1)),title('L');
subplot(3,1,2),plot(Place_Set(:,2)),title('��');
subplot(3,1,3),plot(Place_Set(:,3)),title('h');