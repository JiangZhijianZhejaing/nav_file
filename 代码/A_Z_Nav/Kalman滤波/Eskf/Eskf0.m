function [X_k, P_k, R_k, Beta_k] = Eskf(Sample_Interval, fn_ksub1, Qnb_k_0 , X_ksub1, P_ksub1, Q_ksub1, H_k, Beta_ksub1, b, R_ksub1, Eul_AccMag)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function£ºESKFÂË²¨2.0
%
% Prototype: Qnb_k = Rungekuta(Qnb_ksub1, Sample_Interval, k, Gyro_Set, Qnb_Set)
% Inputs: Sample_Interval - ²ÉÑù¼ä¸ô
%         fn_ksub1 - fn
%         Qnb_k_0 - kÊ±¿ÌÎ´ÐÞÕýËÄÔªÊý
%         X_ksub1 - k - 1Ê±¿Ì×´Ì¬¹À¼Æ
%         P_ksub1 - k - 1Ê±¿ÌÎó²îÕó
%         Q_ksub1 - k - 1Ê±¿ÌÎó²îÕó
%         H_k - k - 1Ê±¿ÌÁ¿²â¾ØÕó
%         Beta_ksub1 - k - 1Ê±¿Ì½¥ÏûÒò×Ó
%         b - k - 0.95
%         R_ksub1 - k - 1Ê±¿ÌÁ¿²âÔëÉù
%         Eul_AccMag - kÊ±¿Ì¾²Ì¬×ËÌ¬½Ç
% Output: X_k - kÊ±¿Ì×´Ì¬¹À¼Æ
%         P_k - kÊ±¿ÌÎó²î
%         R_k - kÊ±¿ÌÁ¿²âÔëÉù
%         Beta_k - kÊ±¿Ì½¥ÏûÒò×Ó
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ¼ÆËã×´Ì¬×ªÒÆ¾ØÕóPHI  
    Eul = Qnb2Eul(Qnb_k_0);
    Eul_Rad = Eul*pi/180; % »¡¶È
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

    % PÕóÔ¤²â
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
    
    
    
    
    
    
    