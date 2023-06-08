function [Qnb_k, V_k, Pos_k, fn_ksub1] = SinsUpdate(Qnb_ksub1, V_ksub1, Pos_ksub1, Acc_Set, Gyro_Set, Qnb_Set, k) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function�������ߵ���ֵ�����㷨(�ͳɱ�)
%
% notice��
%
% Prototype: [Qnb_k, V_k, Pos_k, fn_ksub1] = SinsUpdate(Qnb_ksub1, V_ksub1, Pos_ksub1, gn, Acc_Set, Gyro_Set, Qnb_Set, Sample_Interval, k)
% Inputs: Qnb_ksub1 - k-1ʱ����̬��Ԫ��
%         V_ksub1 - k-1ʱ���ٶ�
%         Place_ksub1 - k-1ʱ�̵���ֱ������ϵ
%         gn - �����������ٶ�
%         Acc_Set - ���ٶȼ�
%         Gyro_Set - ���ٶȼ�
%         Qnb_Set - ���е���̬��Ԫ����
%         Sample_Interval - �������
%         k - ��ǰʱ��
% Output: Qnb_k - kʱ����̬��Ԫ��
%         V_k - kʱ���ٶ�
%         Place_k - kʱ�̵���ֱ������ϵλ��
%         fn_ksub1 - fn
%
% By Taoran Zhao
% 2023/04/13
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gvar;                                                        % �������
    Sample_Interval = 0.01;                                      % �������
    %% ---------------һ�����㵱�ص��򵼺�����---------------
    % ----------1������k-1ʱ�̵���gn----------                                                 
    L = Pos_ksub1(1);                                                  % k-1ʱ�� γ��
    h = Pos_ksub1(3);                                                  % k-1ʱ�� �߶�
    sinL = sin(L);  
    gLh = g0*(1 + 5.27094e-3*sinL^2 + 2.32718e-5*sinL^4) - 3.086e-6*h; % grs80����ģ��
    gn = [0;0;-gLh];                                                   % k-1ʱ�� gn
    %% ---------------�����ߵ�����---------------
    % ----------1���ٶȸ���----------
    fb_ksub1_Normed = NormlzV3(Acc_Set(k,:)');                            
    fb_ksub1 = -gn(3)*fb_ksub1_Normed;                                 % k-1ʱ�� fb
    Cnb_ksub1 = Qnb2Cnb(Qnb_ksub1);                                    % k-1ʱ�� Cnb
    fn_ksub1 = Cnb_ksub1*fb_ksub1;                                     % k-1ʱ�� fn
    Motion_Acceleration_ksub1 = fn_ksub1 + gn;                         % k-1ʱ�� �۳���������˶����ٶ�
    V_k = V_ksub1 + Motion_Acceleration_ksub1*Sample_Interval;         % kʱ�� V
    % ----------2��λ�ø���----------
    % -----(1)����Mpv-----
    Mpv = zeros(3,3);
    key = (1-e2*sinL*sinL);
    sqkey = sqrt(key);
    RMh_ksub1 = Re*(1-e2)/(sqkey*key) + h;                             % k-1ʱ�� RMh
    RNh_ksub1 = Re/sqkey + h;                                          % k-1ʱ�� RNh
    cosLRNh_ksub1 = cos(L)*RNh_ksub1;                                  % k-1ʱ�� cosL*RNh
    Mpv(1,2) = 1/RMh_ksub1;
    Mpv(2,1) = 1/cosLRNh_ksub1;
    Mpv(3,3) = 1;
    Pos_k = Pos_ksub1 + Mpv*V_k*Sample_Interval;                       % kʱ�� Pos
%     Pos123 = [Pos_k(1)*180/pi Pos_k(2)*180/pi Pos_k(3)];
    % ----------3����̬����----------
    if k <= 2
        % ǰ����һ��Rungekuta
        Wxyz_ksub1 = Gyro_Set(k,:)*pi/180;% ת����
        Wx = Wxyz_ksub1(1); Wy = Wxyz_ksub1(2); Wz = Wxyz_ksub1(3);   
        Qnb_k = (eye(4)+0.5*Sample_Interval*[0,   -Wx,  -Wy,  -Wz; Wx,  0,    Wz,   -Wy; Wy,  -Wz,  0,    Wx; Wz,  Wy,   -Wx,  0 ])*Qnb_ksub1;  
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
        Qnb_k = Qnb_ksub2 + T/6*(k1 + 2*k2 + 2*k3 + k4);  % 2ʱ��ǰ  k-2
    end
    Qnb_k = NormlzQnb(Qnb_k);    
    
    
    
