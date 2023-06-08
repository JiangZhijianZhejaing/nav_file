function Qnb_k = Rungekuta_1(Wxyz_ksub1, Sample_Interval, Qnb_ksub1)
    Wx = Wxyz_ksub1(1)*pi/180;   % 转弧度
    Wy = Wxyz_ksub1(2)*pi/180;
    Wz = Wxyz_ksub1(3)*pi/180;   % 负号(人为改变航向北偏东为正)
    Qnb_k = (eye(4)+0.5*Sample_Interval*[0,   -Wx,  -Wy,  -Wz; Wx,  0,    Wz,   -Wy; Wy,  -Wz,  0,    Wx; Wz,  Wy,   -Wx,  0   ])*Qnb_ksub1;  
    Qnb_k = NormlzQnb(Qnb_k);