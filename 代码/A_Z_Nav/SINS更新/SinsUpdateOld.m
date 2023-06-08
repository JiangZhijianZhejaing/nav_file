function [Qnb_k, V_k, Place_k, fn_ksub1] = SinsUpdateOld(Qnb_ksub1, V_ksub1, Place_ksub1, gn, Acc_Set, Gyro_Set, Qnb_Set, Sample_Interval, k) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function�������ߵ���ֵ�����㷨(�ͳɱ�)�ɰ�
%
% Prototype: Qnb_k = Rungekuta(Qnb_ksub1, Sample_Interval, k, Gyro_Set, Qnb_Set)
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
% 2023/03/30
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %�ٶȸ���
     fb_ksub1_Normed=Acc_Set(k,:)';
     fb_ksub1 = -gn(3)*fb_ksub1_Normed;                  % fb
     Cnb_ksub1 = Qnb2Cnb(Qnb_ksub1);
    fn_ksub1 = Cnb_ksub1*fb_ksub1;
    Motion_Acceleration_ksub1 = fn_ksub1 + gn;         %����W^n
    V_k = V_ksub1 + Motion_Acceleration_ksub1*Sample_Interval;
% �ٶȸ���
    % (Qnb_ksub1, V_ksub1, Place_ksub1, gn, Acc_Set, Gyro_Set, Qnb_Set, Sample_Interval, k) 
    fb_ksub1_Normed = Acc_Set(k,:)';                            
    fb_ksub1 = -gn(3)*fb_ksub1_Normed;                  % fb
    Cnb_ksub1 = Qnb2Cnb(Qnb_ksub1);
    fn_ksub1 = Cnb_ksub1*fb_ksub1;
    Motion_Acceleration_ksub1 = fn_ksub1 + gn;
    V_k = V_ksub1 + Motion_Acceleration_ksub1*Sample_Interval;
    % λ�ø���
    Place_k = Place_ksub1 + (V_ksub1+V_k)/2*Sample_Interval;
    % ��̬����
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
    
    
    
