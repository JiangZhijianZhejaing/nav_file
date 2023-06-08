function [Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Set] = IDAndCamEul(File_Name, Sheet_Name, Wp, p, mb, R)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function��1��MARG�������궨 
%           2�����㾲̬ŷ����(�Ƕ�)
%
% Prototype: [Gyro_Set, Acc_Set, Mag_Set, Marg_Number, Eul_AccMag_Deg_Set] = IDAndCamEul(File_Name, Sheet_Name, Wp, p, mb, R)
% Inputs: File_Name - �ļ���
%         Sheet_Name - ����
%         Wp - �����Ǳ궨����
%         p - ���ٶȼƱ궨����
%         mb - �Ŵ������궨����
%         R - ��������
% Output: Gyro_Set - ���ٶȼ�
%         Acc_Set - ���ٶȼ�
%         Mag_Set - ��ʸ����
%         Marg_Number - ������
%         Eul_AccMag_Deg_Set - ��̬��̬�Ǽ�
%
% By Taoran Zhao
% 2023/03/29
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    format long;
    Marg_Raw_Set = xlsread(File_Name, Sheet_Name);          % ������Դ�����ԭʼ����
    Marg_Number = size(Marg_Raw_Set,1);                     % MARG������
    Gyro_Raw_Set = Marg_Raw_Set(:,1:3);                     % ������ԭʼ���ݼ�
    Acc_Raw_Set = Marg_Raw_Set(:,4:6);                      % ���ٶȼ�ԭʼ���ݼ���X����ٶ�ԭʼ���ݵ���������YZ�෴�����ڴ�����Ӹ��ţ�
    Mag_Raw_Set = Marg_Raw_Set(:,7:9);                      % �Ŵ�����ԭʼ���ݼ�

    %% �����Ǳ궨
    Gyro_offset = mean(Gyro_Raw_Set(1:300,:));              % �����ÿ�������󲻱䣬��ÿ���������ǹ̶�ֵ
    Gyro_Raw_Set = Gyro_Raw_Set - Gyro_offset;              % �۳�ÿ�ο�����������
    GyroMax = max(Gyro_Raw_Set(1:300,:));
    GyroMin = min(Gyro_Raw_Set(1:300,:));
    for i = 1 : Marg_Number
        if Gyro_Raw_Set(i,1) <= GyroMax(1) && Gyro_Raw_Set(i,1) >= GyroMin(1)
           Gyro_Raw_Set(i,1) = 0; 
        end
        if Gyro_Raw_Set(i,2) <= GyroMax(2) && Gyro_Raw_Set(i,2) >= GyroMin(2)
           Gyro_Raw_Set(i,2) = 0; 
        end
        if Gyro_Raw_Set(i,3) <= GyroMax(3) && Gyro_Raw_Set(i,3) >= GyroMin(3)
           Gyro_Raw_Set(i,3) = 0; 
        end      
    end
    Gyro_Set_0 = (Gyro_Raw_Set*Wp);         % ������ʵ�����ֵ��deg/s������������x������ʡ���y������ʡ���z�������
    Gyro_Set(:,1) = -Gyro_Set_0(:,2);       % Wx    X��Y����
    Gyro_Set(:,2) = -Gyro_Set_0(:,1);       % Wy
    Gyro_Set(:,3) = Gyro_Set_0(:,3);        % Wz    Z*(-1)
    
    %% ���ٶȼƱ궨
    Acc_Raw_set_add1 = horzcat(Acc_Raw_Set, ones(Marg_Number, 1));  % ��һ��1
    % p =[   0.002903387627154  -0.000060755386809   0.000000894230601			
    %   -0.000031834924067   0.002946774674143   0.000004641374621			
    %    0.000003733332459  -0.000009064807629   0.002855073329104			
    %   -0.001121527428827  -0.004231396444889  -0.081810863260599];             % ���ٶȼƱ궨����
    Acc_Set_0 = Acc_Raw_set_add1*p;  % ������ٶȣ���һ����
    Acc_Set(:,1) = -Acc_Set_0(:,2);  % fbx    X��Y���� 
    Acc_Set(:,2) = Acc_Set_0(:,1);  % fby    
    Acc_Set(:,3) = Acc_Set_0(:,3);  % fbz    Z*(-1)
    
    %% �Ŵ������궨
    headingData = (mb*Mag_Raw_Set')';
    Mag_Set_0 = [headingData(:,1) - R(1), headingData(:,2) - R(2), headingData(:,3) - R(3)]; % ����ʵ�ʴ�ͨ��
    Mag_Set(:,1) = Mag_Set_0(:,2);   % mbx    X��Y����
    Mag_Set(:,2) = Mag_Set_0(:,1);   % mby 
    Mag_Set(:,3) = -Mag_Set_0(:,3);  % mbz    Z*(-1)

    %% ���㾲̬��̬��
    Pitch(1:Marg_Number) = 0;
    Roll(1:Marg_Number) = 0;
    for i = 1 : Marg_Number
        fbx = Acc_Set(i, 1);     
        fby = Acc_Set(i, 2);
        fbz = Acc_Set(i, 3);
        % ������ 
        Pitch(i) = atan(fby/sqrt((fbx)^2+(fbz)^2));     % y�����ֵ��- 
        % ����� 
        Roll_main = atan(-fbx/fbz);   
        if  fbz >= 0 
           Roll(i) = Roll_main;       % -90�㡫+90��
        elseif  (Roll_main <= 0) && (fbz < 0)
           Roll(i) = pi + Roll_main;  % 90~180��
        else
           Roll(i) = -pi + Roll_main; % -180~-90�� (Roll_main > 0) && (fz > 0)
        end  
    end  
    Pitch_Acc_Set = Pitch*180/pi; 
    Roll_Acc_Set = Roll*180/pi;       % ����->�Ƕ�
    % ����� 
    for i = 1:Marg_Number
        % ���궨��ĴŴ�����ͶӰ��ˮƽ����ϵ
        mbx = Mag_Set(i,1);
        mby = Mag_Set(i,2);
        mbz = Mag_Set(i,3);
        % תˮƽ����ϵCSCY:l
        mlx = mbx*cos(Roll(i)) + mbz*sin(Roll(i));
        mly = mbx*sin(Pitch(i))*sin(Roll(i)) + mby*cos(Pitch(i)) - mbz*sin(Pitch(i))*cos(Roll(i));
        Yaw_main = -atan(mlx/mly);
        if (mlx <= 0) && (mly >=0)        % 0�㵽90��
            Yaw(i) = Yaw_main;
        elseif (mlx <= 0) && (mly <0)     % 90�㵽180��
            Yaw(i) = pi + Yaw_main;
        elseif (mlx > 0) && (mly < 0)     % 180�㵽270��
            Yaw(i) = pi + Yaw_main; 
        elseif (mlx > 0) && (mly >= 0)     % 270�㵽360��
            Yaw(i) = 2*pi + Yaw_main;
        end
    end
    Yaw_Mag_Set = Yaw * 180/pi;                % ��ƫ��Ϊ��������->�Ƕ�
    % #########################################################################
    % 3����̬��̬��(deg)
    Eul_AccMag_Set(:,1) = Pitch_Acc_Set;   % ���ŷ����(��λ��deg��
    Eul_AccMag_Set(:,2) = Roll_Acc_Set;
    Eul_AccMag_Set(:,3) = Yaw_Mag_Set;      

