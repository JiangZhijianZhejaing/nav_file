function vo = QnbMulV(q, vi)  % ��Ԫ����ʸ��������άʸ������Ԫ������任
    qi = [0;vi];
    qo = QnbMulQnb(QnbMulQnb(q,qi),QnbConj(q));
    vo = qo(2:4,1);
    % vo = q2mat(q)*vi;
