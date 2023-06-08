%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function���������״̬Kalman�˲��ں϶���̬��̬��(�ͳɱ���)
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ---------------һ���궨���ݲ����㾲̬��̬��---------------
clc;                            % ����������
clear;                          % ��������
addpath(genpath('../../'));     % �������ļ�������m�ļ�
CalibParm_No1;                  % ����1��ģ��궨����
[Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul...
    ('marg1', 'Sheet1', Calib_Parm.Wp, Calib_Parm.p, Calib_Parm.mb, Calib_Parm.R);

% ---------------����ESKF��ʼ��---------------
Sample_Interval = 0.01;                                 % Gyro�������
% ----------1����ʼavp----------
Eul_AccMag_0 = mean(Eul_AccMag_Set(1:300,:));           % ȡǰ3���ֵΪ��ʼ��̬ŷ����
Qnb_0 = Eul2Qnb(Eul_AccMag_0);                          % ��ʼ��̬��Ԫ��
Qnb_ksub1 = Qnb_0;                                      % k-1ʱ�� Qnb
Qnb_Set = zeros(4,Marg_Number);                         % Ԥ�����ڴ�
Eul_Set = zeros(Marg_Number,3);
% ----------2��kalman��ʼ��----------
X_ksub1 = zeros(6,1);
P_ksub1 = eye(numel(X_ksub1));
Q_ksub1 = (1e-9)*eye(numel(X_ksub1));                   % ״̬����������
R_ksub1 = (1e-1)*eye(3);                                % ���ٶȼ���������������
H_k = [eye(3),zeros(3,3)];
Beta_ksub1 = 1;
b = 0.95;

%% ---------------����ESKF--------------
for k = 1:Marg_Number
    % ----------1����̬����----------
    if k <= 2
        % ǰ����һ��Rungekuta
        Wxyz_ksub1 = Gyro_Set(k,:)*pi/180;% ת����
        Wx = Wxyz_ksub1(1); Wy = Wxyz_ksub1(2); Wz = Wxyz_ksub1(3);   
        Qnb_k_0 = (eye(4)+0.5*Sample_Interval*[0,   -Wx,  -Wy,  -Wz; Wx,  0,    Wz,   -Wy; Wy,  -Wz,  0,    Wx; Wz,  Wy,   -Wx,  0 ])*Qnb_ksub1;  
    else
        % �����Ľ�Rungekuta
        Qnb_ksub2 = Qnb_Set(:, k - 2);
        Wxyz_ksub1_2_ksub3_Set = Gyro_Set(k - 2 : k, :);
        T = 2*Sample_Interval;
        Wxyz1 = Wxyz_ksub1_2_ksub3_Set(1,:)*pi/180;
        Wxyz2 = Wxyz_ksub1_2_ksub3_Set(2,:)*pi/180;
        Wxyz3 = Wxyz_ksub1_2_ksub3_Set(3,:)*pi/180;
        Wx1 = Wxyz1(1); Wy1 = Wxyz1(2); Wz1 = Wxyz1(3);    % t
        Wx2 = Wxyz2(1); Wy2 = Wxyz2(2); Wz2 = Wxyz2(3);    % t + 1/2T 
        Wx3 = Wxyz3(1); Wy3 = Wxyz3(2); Wz3 = Wxyz3(3);    % t + T
        k1 = 1/2*[0,   -Wx1,  -Wy1,  -Wz1; Wx1,  0,    Wz1,   -Wy1; Wy1,  -Wz1,  0,    Wx1; Wz1,  Wy1,   -Wx1,  0   ]*Qnb_ksub2;
        k2 = 1/2*[0,   -Wx2,  -Wy2,  -Wz2; Wx2,  0,    Wz2,   -Wy2; Wy2,  -Wz2,  0,    Wx2; Wz2,  Wy2,   -Wx2,  0   ]*(Qnb_ksub2 + k1/2);
        k3 = 1/2*[0,   -Wx2,  -Wy2,  -Wz2; Wx2,  0,    Wz2,   -Wy2; Wy2,  -Wz2,  0,    Wx2; Wz2,  Wy2,   -Wx2,  0   ]*(Qnb_ksub2 + k2/2);
        k4 = 1/2*[0,   -Wx3,  -Wy3,  -Wz3; Wx3,  0,    Wz3,   -Wy3; Wy3,  -Wz3,  0,    Wx3; Wz3,  Wy3,   -Wx3,  0   ]*(Qnb_ksub2 + k3);
        Qnb_k_0 = Qnb_ksub2 + T/6*(k1 + 2*k2 + 2*k3 + k4);  % 2ʱ��ǰ  k-2
    end
    Qnb_k_0 = NormlzQnb(Qnb_k_0);                                          % kʱ�� δУ����Qnb
    Eul_k_0 = Qnb2Eul(Qnb_k_0);                                            % kʱ�� δУ������̬ŷ����
    % ----------2��eskf���õ�����ֵ----------
    Eul_AccMag = Eul_AccMag_Set(k,:)';              % kʱ�� ��̬��̬ŷ����

    Eul_ksub1 = Qnb2Eul(Qnb_ksub1);                 % k-1ʱ�� ��̬ŷ����(�Ƕ�)
    Eul_Rad_ksub1 = Eul_ksub1*pi/180;               % k-1ʱ�� ��̬ŷ����(����)
    Pitch = Eul_Rad_ksub1(1);  
    Yaw = Eul_Rad_ksub1(3);
    Cnb_ksub1 = Qnb2Cnb(Qnb_ksub1);                 % k-1ʱ�� Cnb
    Mre = [-cos(Yaw) -sin(Yaw) 0; sin(Yaw)/cos(Pitch) -cos(Yaw)/cos(Pitch) 0; -tan(Pitch)*sin(Yaw) tan(Pitch)*cos(Yaw) -1];    % A = Mr2e*R
    % -----(1)PHI_k_ksub1-----
    Ma1 = Mre*(-Cnb_ksub1);
    %                A            eb(or1)            
    phi_k_ksub1 = [  zeros(3),    Ma1;           % A
                     zeros(3),    zeros(3)];     % eb(or1)                
    PHI_k_ksub1 = eye(6) + phi_k_ksub1*Sample_Interval;                    % ��ɢ��
    % ----------3��Kalman Filter----------
    % -----(1)״̬һ��Ԥ��-----
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;
    % -----(2)״̬һ��Ԥ����������-----
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);
    % -----(3)����Ӧ����R_k-----
    Z_k = Eul_k_0 - Eul_AccMag;
    if abs(Z_k(3)) > 90
        Z_k(3) = X_k_ksub1(3);
    end
    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1; 
    Beta_k = Beta_ksub1/(Beta_ksub1 + b);
    R_k = (1 - Beta_k)*R_ksub1 + Beta_k*(Z_error_k_ksub1*Z_error_k_ksub1' - H_k*P_k_ksub1*H_k');
    % -----(4)�˲�����-----
    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_k;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    % -----(5)״̬����-----
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    % -----(6)״̬���ƾ��������-----
    P_k = (eye(6) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    % ----------3��avp����ֵ----------
    Eul_k = Eul_k_0 - X_k(1:3);             % ������̬ŷ����
    Eul_Set(k,:) = Eul_k;
    Qnb_k = Eul2Qnb(Eul_k);        
    Qnb_Set(:,k) = Qnb_k;
    % ----------4��״̬��0----------
    X_k(1:3) = 0;
    % ----------5����������----------
    X_ksub1 = X_k;
    P_ksub1 = P_k;
    Beta_ksub1 = Beta_k;
    R_ksub1 = R_k;
    Qnb_ksub1 = Qnb_k;
end
%% ----------������ͼ----------
figure(1)
subplot(3,1,1),plot(Eul_AccMag_Set(:,1)),title('������ Pitch'),xlabel('ʱ�� t/s'),ylabel('�Ƕ� A/��'),legend('\theta');
subplot(3,1,2),plot(Eul_AccMag_Set(:,2)),title('����� Roll '),xlabel('ʱ�� t/s'),ylabel('�Ƕ� A/��'),legend('\gamma');
subplot(3,1,3),plot(Eul_AccMag_Set(:,3)),title('����� Yaw '),xlabel('ʱ�� t/s'),ylabel('�Ƕ� A/��'),legend('\psi');
sgtitle('��̬������');
figure(2)
subplot(3,1,1),plot(Eul_Set(:,1)),title('������ Pitch'),xlabel('ʱ�� t/s'),ylabel('A/��'),legend('\theta');
subplot(3,1,2),plot(Eul_Set(:,2)),title('����� Roll '),xlabel('ʱ�� t/s'),ylabel('A/��'),legend('\gamma');
subplot(3,1,3),plot(Eul_Set(:,3)),title('����� Yaw '),xlabel('ʱ�� t/s'),ylabel('A/��'),legend('\psi');
sgtitle('��̬��');



