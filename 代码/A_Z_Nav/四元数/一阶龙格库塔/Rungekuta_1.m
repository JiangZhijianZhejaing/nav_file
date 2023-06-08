function Qnb_k = Rungekuta_1(Wxyz_ksub1, Sample_Interval, Qnb_ksub1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��һ�����������
%
% Prototype: Qnb_k = Rungekuta_1(Wxyz_ksub1, Sample_Interval, Qnb_ksub1)
% Inputs: Wxyz_ksub1 - k-1ʱ�̽��ٶ�(�Ƕ�)
%         Sample_Interval - �������
%         Qnb_ksub1 - k-1ʱ����̬��Ԫ��
% Output: Qnb_k - kʱ����̬��Ԫ��
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Wx = Wxyz_ksub1(1)*pi/180;   % ת����
    Wy = Wxyz_ksub1(2)*pi/180;
    Wz = Wxyz_ksub1(3)*pi/180;   % ����(��Ϊ�ı亽��ƫ��Ϊ��)
    Qnb_k = (eye(4)+0.5*Sample_Interval*[0,   -Wx,  -Wy,  -Wz; Wx,  0,    Wz,   -Wy; Wy,  -Wz,  0,    Wx; Wz,  Wy,   -Wx,  0   ])*Qnb_ksub1;  
    Qnb_k = NormlzQnb(Qnb_k);