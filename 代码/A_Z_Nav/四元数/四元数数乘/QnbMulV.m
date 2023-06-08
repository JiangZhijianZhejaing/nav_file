function vo = QnbMulV(q, vi)  % 四元数乘矢量，即三维矢量的四元数坐标变换
    qi = [0;vi];
    qo = QnbMulQnb(QnbMulQnb(q,qi),QnbConj(q));
    vo = qo(2:4,1);
    % vo = q2mat(q)*vi;
