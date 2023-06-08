function [X_k, P_k, R_k, Beta_k] = Eskf(fn_ksub1, Qnb_ksub1, V_ksub1, Pos_ksub1, X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, R_ksub1, Eul_AccMag, Qnb_k_0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��ESKF�˲�2.0
%
% Prototype: Qnb_k = Rungekuta(Qnb_ksub1, Sample_Interval, k, Gyro_Set, Qnb_Set)
% Inputs: Sample_Interval - �������
%         fn_ksub1 - fn
%         Qnb_k_0 - kʱ��δ������Ԫ��
%         X_ksub1 - k - 1ʱ��״̬����
%         P_ksub1 - k - 1ʱ�������
%         Q_ksub1 - k - 1ʱ�������
%         H_k - k - 1ʱ���������
%         Beta_ksub1 - k - 1ʱ�̽�������
%         b - k - 0.95
%         R_ksub1 - k - 1ʱ����������
%         Eul_AccMag - kʱ�̾�̬��̬��
% Output: X_k - kʱ��״̬����
%         P_k - kʱ�����
%         R_k - kʱ����������
%         Beta_k - kʱ�̽�������
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gvar;                                           % �������
    Sample_Interval = 0.01;                         % �������
    %% ---------------һ������״̬ת�ƾ���k-1ʱ��PHI---------------
    % ----------1��A----------
    Eul_ksub1 = Qnb2Eul(Qnb_ksub1);                 % k-1ʱ�� ��̬ŷ����(�Ƕ�)
    Eul_Rad_ksub1 = Eul_ksub1*pi/180;               % k-1ʱ�� ��̬ŷ����(����)
    Pitch = Eul_Rad_ksub1(1);  
    Yaw = Eul_Rad_ksub1(3);
    Cnb_ksub1 = Qnb2Cnb(Qnb_ksub1);                       % k-1ʱ�� Cnb
%     Qnb_ksub1, V_ksub1, Pos_ksub1,ѡ��ʸ��תΪŷ����
    Mre = [-cos(Yaw) -sin(Yaw) 0; sin(Yaw)/cos(Pitch) -cos(Yaw)/cos(Pitch) 0; -tan(Pitch)*sin(Yaw) tan(Pitch)*cos(Yaw) -1];    % A = Mr2e*R
    Mer = [-cos(Yaw) cos(Pitch)*sin(Yaw) 0;-sin(Yaw) -cos(Pitch)*cos(Yaw) 0;0  -sin(Pitch) -1];                                % R = Me2r*A
    % -----(1)Ma1-----
    Ma1 = Mre*(-Cnb_ksub1);
    % ----------2��V----------
    % -----(1)Mva-----
    Mva = [0, -fn_ksub1(3), fn_ksub1(2); fn_ksub1(3), 0, -fn_ksub1(1); -fn_ksub1(2), fn_ksub1(1), 0]*Mer;
    % -----(2)Mv2-----
    Mv2 = Cnb_ksub1;
    % ----------3��P----------
    % -----(1)Mpv-----
    Mpv = zeros(3,3);
    L = Pos_ksub1(1);                                                  % k-1ʱ�� γ��
    h = Pos_ksub1(3);                                                  % k-1ʱ�� �߶�
    sinL = sin(L); 
    cosL = cos(L);
    key = (1-e2*sinL*sinL);
    sqkey = sqrt(key);
    RMh_ksub1 = Re*(1-e2)/(sqkey*key) + h;                             % k-1ʱ�� RMh
    RNh_ksub1 = Re/sqkey + h;                                          % k-1ʱ�� RNh
    cosLRNh_ksub1 = cosL*RNh_ksub1;                                    % k-1ʱ�� cosL*RNh
    Mpv(1,2) = 1/RMh_ksub1;
    Mpv(2,1) = 1/cosLRNh_ksub1;
    Mpv(3,3) = 1;  
    % -----(2)Mpp-----
    Mpp = zeros(3,3);
    VE = V_ksub1(1);
    VN = V_ksub1(3);
    tanL = tan(L);
    Mpp(1,3) = -VN/(RMh_ksub1*RMh_ksub1);
    Mpp(2,1) = VE*tanL/(cosL*RNh_ksub1);
    Mpp(2,3) = -VE/(cosL*RNh_ksub1*RNh_ksub1);
    % ----------4��phi_k_ksub1----------
    %                A            V            P            eb(or1)        db(or2)
    phi_k_ksub1 = [  zeros(3),    zeros(3),    zeros(3),    Ma1 ,        zeros(3) ;     % A
                     Mva,         zeros(3),    zeros(3),    zeros(3),    Mv2;           % V
                     zeros(3),    Mpv,         Mpp,         zeros(3),    zeros(3);      % P
                     zeros(3),    zeros(3),    zeros(3),    zeros(3),    zeros(3);      % eb(or1)
                     zeros(3),    zeros(3),    zeros(3),    zeros(3),    zeros(3);];    % db(or2)
    PHI_k_ksub1 = eye(15) + phi_k_ksub1*Sample_Interval;                   % ��ɢ��
    %% ---------------����Kalman Filter---------------
    % ----------1��״̬һ��Ԥ��----------
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;
    % ----------2��״̬һ��Ԥ����������----------
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);
    % ----------3������Ӧ����R_k----------
    Eul_k = Qnb2Eul(Qnb_k_0);
    Z_k = Eul_k - Eul_AccMag;
    if abs(Z_k(3)) > 90
        Z_k(3) = X_k_ksub1(3);
    end
    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1; 
    b = 0.95;
    Beta_k = Beta_ksub1/(Beta_ksub1 + b);
    R_k = (1 - Beta_k)*R_ksub1 + Beta_k*(Z_error_k_ksub1*Z_error_k_ksub1' - H_k*P_k_ksub1*H_k');
    % ----------4���˲�����----------
    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_k;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    % ----------5��״̬����----------
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    % ----------6��״̬���ƾ��������----------
    P_k = (eye(15) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    
    
    
    
    
    
    