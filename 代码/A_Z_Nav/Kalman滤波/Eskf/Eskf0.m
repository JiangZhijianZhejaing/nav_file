function [X_k, P_k, R_k, Beta_k] = Eskf(Sample_Interval, fn_ksub1, Qnb_k_0 , X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, b, R_ksub1, Eul_AccMag)
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
    %% ����״̬ת�ƾ���PHI  
    Eul = Qnb2Eul(Qnb_k_0);
    Eul_Rad = Eul*pi/180; % ����
    Pitch = Eul_Rad(1);  
    Yaw = Eul_Rad(3);
    Cnb = Qnb2Cnb(Qnb_k_0);
    
    Mr2e = [-cos(Yaw) -sin(Yaw) 0; sin(Yaw)/cos(Pitch) -cos(Yaw)/cos(Pitch) 0; -tan(Pitch)*sin(Yaw) tan(Pitch)*cos(Yaw) -1];    % A = Mr2e*R
    Me2r = [-cos(Yaw) cos(Pitch)*sin(Yaw) 0;-sin(Yaw) -cos(Pitch)*cos(Yaw) 0;0  -sin(Pitch) -1];                                % R = Me2r*A
    Ma1 = Mr2e*(-Cnb);
    Mva = [0, -fn_ksub1(3), fn_ksub1(2); fn_ksub1(3), 0, -fn_ksub1(1); -fn_ksub1(2), fn_ksub1(1), 0]*Me2r;
    Mv2 = Cnb;
    Mpv = eye(3);

    %                A            v            p            eb           db
    phi_k_ksub1 = [  zeros(3),    zeros(3),    zeros(3),    Ma1 ,        zeros(3) ;
                     Mva,         zeros(3),    zeros(3),    zeros(3),    Mv2;
                     zeros(3),    Mpv,         zeros(3),    zeros(3),    zeros(3);
                     zeros(3),    zeros(3),    zeros(3),    zeros(3),    zeros(3);
                     zeros(3),    zeros(3),    zeros(3),    zeros(3),    zeros(3);];
    
    PHI_k_ksub1 = eye(15) + phi_k_ksub1*Sample_Interval;
    %%
    X_k_ksub1 = PHI_k_ksub1*X_ksub1;

    % P��Ԥ��
    P_k_ksub1 = PHI_k_ksub1*P_ksub1*PHI_k_ksub1' + Q_ksub1;
    P_k_ksub1 = P2diagP(P_k_ksub1);
 
    Z_k = Eul - Eul_AccMag;


    if abs(Z_k(3)) > 90
        Z_k(3) = X_k_ksub1(3);
    end

    Z_error_k_ksub1 = Z_k - H_k*X_k_ksub1;   
    Beta_k = Beta_ksub1/(Beta_ksub1 + b);
    R_k = (1 - Beta_k)*R_ksub1 + Beta_k*(Z_error_k_ksub1*Z_error_k_ksub1' - H_k*P_k_ksub1*H_k');

    PXZ_k_ksub1 = P_k_ksub1*H_k';
    PZZ_k_ksub1 = H_k*PXZ_k_ksub1 + R_k;
    K_k = PXZ_k_ksub1/PZZ_k_ksub1;
    
    X_k = X_k_ksub1 + K_k*(Z_error_k_ksub1);
    
    P_k = (eye(15) - K_k*H_k)*P_k_ksub1;
    P_k = P2diagP(P_k);
    
    
    
    
    
    
    