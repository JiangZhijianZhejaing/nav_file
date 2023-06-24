function Data_LowPass = Fft_IdealLowPassFilter(Data, Cutoff_Freq)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：FFT频谱分析 + 理想低通滤波
%
% Prototype: Data_LowPass = Fft_Low_Pass_Filter(Data, Cutoff_Freq)
% Inputs: Data - 原始数据
%         Cutoff_Freq - 截止频率
% Output: Data_LowPass - 低通后的数据
%
% Notice：见《储志伟.基于MARG传感器的微型航姿系统[D].中国科学技术大学,2018.》第四章
%
% By Taoran Zhao
% 2023/04/26 2023/05/06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------一、时域(原始)---------------
L = size(Data,2);                                 % 数据长度
Fs = 100;                                       % 采样率100Hz
Time = 0 : 1/Fs : (L - 1)/Fs;                   % 时间序列
Amp_Time_Raw = Data;                      % 时域幅度(原始)
% 绘制原始信号时域图像
subplot(2, 2, 1);
plot(Time, Amp_Time_Raw);
xlabel('时间 (s)');
ylabel('幅度');
title('时域（Raw）');
% ---------------二、频域(原始)---------------
hz = (0 : L-1)*(Fs/L);                          % 计算频率坐标
Amp_Fre_Raw = fft(Amp_Time_Raw);    % 执行FFT变换
% 将FFT结果取绝对值并将图形绘制在频率域上
Amp_Fre_Raw_abs = abs(Amp_Fre_Raw); % 获取幅值
Hz = hz(1:fix(L/2));                            % 单边频率
Amp_Fre_Raw_abs_Half = Amp_Fre_Raw_abs(1:fix(L/2));
subplot(2, 2, 2);
plot(Hz, Amp_Fre_Raw_abs_Half);
xlabel('频率 (Hz)');
ylabel('幅度');
title('频域（Raw）');
% ---------------三、频域(低通滤波)---------------
Coefficient = zeros(1,L);           % 系数矩阵
% 截止阈值（低于Cutoff_Freq Hz通过，高于Cutoff_Freq Hz忽略）
for k = 1:L
    if hz(1,k) <= Cutoff_Freq || hz(1,k) >= 100 - Cutoff_Freq
        Coefficient(1,k) = 1;
    else
        Coefficient(1,k) = 0;
    end
end
Amp_Fre_LowPass = Coefficient.*Amp_Fre_Raw;
Amp_Fre_LowPass_Half = abs(Amp_Fre_LowPass(1:fix(L/2)));
% 绘制经过滤波后的单边频谱图像
subplot(2, 2, 3);
plot(Hz, Amp_Fre_LowPass_Half);
xlabel('频率 (Hz)');
ylabel('幅度');
title('频谱（LowPass）');
% ---------------四、时域(低通滤波)---------------
Data_LowPass = ifft(Amp_Fre_LowPass);
% 绘制降噪后的时域图像
subplot(2, 2, 4);
plot(Time, Data_LowPass);
xlabel('时间(s)');
ylabel('幅度');
title('时域（LowPass）');
