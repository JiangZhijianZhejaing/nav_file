function [X_k,P_k] = Dukf(PHI_k_ksub1, X_ksub1, P_ksub1, Acc_Set, Mag_Set, Q_ksub1, R_Acc_ksub1, R_Mag_ksub1, k, h_Acc_k_Fun, h_Mag_k_Fun) 
    % -----(1)UKFȨֵ����-----
    L = numel(X_ksub1);        % ״̬����ά��
    alpha = 1; ki = -1; beta = 2;
    lambda = alpha^2*(L + ki) - L;
    c = L + lambda;
    Wm = [lambda/c, 1/(2*c) + zeros(1, 2*L)]; 
    Wc = Wm;  
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);
    % -----(2)����X_k��sigma������-----
    A = (chol(c*P_ksub1))';
    Y = ones(L).*X_ksub1;  
    X_ksub1_set = [X_ksub1, Y + A, Y - A]; 
    % -----(3)����X_k_ksub1��sigma������-----
    N = size(X_ksub1_set,2); % 9
    X_k_ksub1 = zeros(L,1);
    X_k_ksub1_set = zeros(L,N);
    for i = 1 : N
        X_k_ksub1_set(:,i) = PHI_k_ksub1*X_ksub1_set(:,i);        % ��X_k��sigma�㼯ͨ��״̬ת�ƺ���ӳ�䵽X_k_ksub1��sigma�㼯
        X_k_ksub1 = X_k_ksub1 + Wm(i)*X_k_ksub1_set(:,i);         % ���㴫�ݺ�ľ�ֵ
    end
    X_k_ksub1 = NormlzQnb(X_k_ksub1);
    % -----(4)���㴫�ݺ��Э���� -----
    P_k_ksub1 = zeros(4);
    for i = 1 : N
        P_k_ksub1 = P_k_ksub1 + Wc(i)*(X_k_ksub1_set(:,i) - X_k_ksub1)*(X_k_ksub1_set(:,i) - X_k_ksub1)';
    end
    P_k_ksub1 = P_k_ksub1 + Q_ksub1;   
    P_k_ksub1 = P2diagP(P_k_ksub1);                              % �Գƻ�
    %% ---------------����������ٶ�У��(No.1��)---------------
    Z_Acc_k = Acc_Set(k,:)';
    M1 = numel(Z_Acc_k);
    % -----(1)���ٶ��������-----
    Z_Acc_k_ksub1 = zeros(M1,1);
    Z_Acc_k_ksub1_set = zeros(M1,N);
    for i = 1 : N
        Z_Acc_k_ksub1_set(:,i) = h_Acc_k_Fun(X_k_ksub1_set(:,i));          % ��X_k_ksub1��sigma�㼯ͨ��״̬ת�ƺ���ӳ�䵽Z_Acc_k_ksub1��sigma�㼯
        Z_Acc_k_ksub1 = Z_Acc_k_ksub1 + Wm(i)*Z_Acc_k_ksub1_set(:,i);      % ���㴫�ݺ�ľ�ֵ
    end
    % -----(2)����Kalman����-----
    PZZ_Acc_k_ksub1 = zeros(3);
    for i = 1 : N
        PZZ_Acc_k_ksub1 = PZZ_Acc_k_ksub1 + Wc(i)*(Z_Acc_k_ksub1_set(:,i) - Z_Acc_k_ksub1)*(Z_Acc_k_ksub1_set(:,i) - Z_Acc_k_ksub1)';
    end
    PZZ_Acc_k_ksub1 = PZZ_Acc_k_ksub1 + R_Acc_ksub1;                       % P_z1 
    PXZ_Acc_k_ksub1 = zeros(4,3);
    for i = 1 : N
        PXZ_Acc_k_ksub1 = PXZ_Acc_k_ksub1 + Wc(i)*(X_k_ksub1_set(:,i) - X_k_ksub1)*(Z_Acc_k_ksub1_set(:,i) - Z_Acc_k_ksub1)';
    end                                                                    % P_xz1
    K_Acc_k = PXZ_Acc_k_ksub1/PZZ_Acc_k_ksub1;                             % ����Kalman����
    Z_Acc_k_ksub1_err = Z_Acc_k - Z_Acc_k_ksub1;
    del_X_Acc = K_Acc_k*(Z_Acc_k_ksub1_err);
    X_del_Acc = del_X_Acc.*[1;1;1;0];
    X_Acc_k = X_k_ksub1 + X_del_Acc; 
    P_Acc_k = P_k_ksub1 - K_Acc_k*PZZ_Acc_k_ksub1*K_Acc_k';
    %% ---------------���������Ŵ�����У��(No.2��)---------------
    Z_Mag_k = Mag_Set(k,:)';
    M2 = numel(Z_Mag_k);
    % -----(1)�ų�ǿ���������-----
    Z_Mag_k_ksub1 = zeros(M2,1);
    Z_Mag_k_ksub1_set = zeros(M2,N);
    Cnb = Qnb2Cnb(X_k_ksub1);
    mn = Cnb*Mag_Set(k,:)';
    mny = mn(2);
    mnz = mn(3);
    for i = 1 : N
        Z_Mag_k_ksub1_set(:,i) = h_Mag_k_Fun(X_k_ksub1_set(:,i), mny, mnz); 
        Z_Mag_k_ksub1 = Z_Mag_k_ksub1 + Wm(i)*Z_Mag_k_ksub1_set(:,i);
    end
    % -----(2)����Kalman����-----
    PZZ_Mag_k_ksub1 = zeros(3);
    for i = 1 : N
        PZZ_Mag_k_ksub1 = PZZ_Mag_k_ksub1 + Wc(i)*(Z_Mag_k_ksub1_set(:,i) - Z_Mag_k_ksub1)*(Z_Mag_k_ksub1_set(:,i) - Z_Mag_k_ksub1)';
    end
    PZZ_Mag_k_ksub1 = PZZ_Mag_k_ksub1 + R_Mag_ksub1;                       % P_z2
    PXZ_Mag_k_ksub1 = zeros(4,3);
    for i = 1 : N
        PXZ_Mag_k_ksub1 = PXZ_Mag_k_ksub1 + Wc(i)*(X_k_ksub1_set(:,i) - X_k_ksub1)*(Z_Mag_k_ksub1_set(:,i) - Z_Mag_k_ksub1)';
    end                                                                    % P_xz2
    K_Mag_k = PXZ_Mag_k_ksub1/PZZ_Mag_k_ksub1;                             % ����Kalman����
    Z_Mag_k_ksub1_err = Z_Mag_k - Z_Mag_k_ksub1;
    del_X_Mag = K_Mag_k*(Z_Mag_k_ksub1_err);
    X_del_Mag = del_X_Mag.*[1;0;0;1];
    X_k = X_Acc_k + X_del_Mag; 
    P_k = P_Acc_k - K_Mag_k*PZZ_Mag_k_ksub1*K_Mag_k';      



