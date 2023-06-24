% function Data_LowPass = RBFNN(Data, Cutoff_Freq)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function：FFT频谱分析 + 理想低通滤波
%
% Prototype: Data_LowPass = Fft_Low_Pass_Filter(Data, Cutoff_Freq)
% Inputs: Data - 原始数据
%         Cutoff_Freq - 截止频率
% Output: Data_LowPass - 低通后的数据
%
% By Taoran Zhao
% 2023/04/26 2023/05/06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 使用RBFNN实现分类问题
% 此处使用鸢尾花数据集进行演示

% 导入数据集
load fisheriris;

% 将鸢尾花数据集分成训练集和测试集
trainData = [meas(1:40,:); meas(51:90,:); meas(101:140,:)]; % 前40个为setosa，中间40个为versicolor，后40个为virginica
trainLabels = [ones(40,1); 2 * ones(40,1); 3 * ones(40,1)]; % 设置训练集的标签
testData = [meas(41:50,:); meas(91:100,:); meas(141:150,:)]; % 剩余10个为setosa，中间10个为versicolor，后10个为virginica
testLabels = [ones(10,1); 2 * ones(10,1); 3 * ones(10,1)]; % 设置测试集的标签








% 训练RBF神经网络模型
numOutputNeurons = 3; % 设置输出神经元的数量
numHiddenNeurons = 8; % 设置隐层神经元的数量
centers = randperm(length(trainLabels),numHiddenNeurons); % 随机选择一些特征向量作为RBF神经元的中心
sigma = mean(pdist(trainData(centers,:))); % 计算RBF神经元的尺度参数
Phi = exp(-pdist2(trainData,trainData(centers,:))/sigma); % 计算RBF网络隐层输出，使用高斯函数
Phi = [ones(size(Phi,1),1) Phi]; % 对隐层输出增加一个偏置项
W = inv(Phi'*Phi)*Phi'* ind2vec(trainLabels')'; % 计算权值

% 使用RBF神经网络进行预测
Phi_test = exp(-pdist2(testData,trainData(centers,:))/sigma); % 计算测试集的RBF网络隐层输出
Phi_test = [ones(size(Phi_test,1),1) Phi_test]; % 对隐层输出增加一个偏置项
predictedLabels = vec2ind(Phi_test*W); % 使用权值进行预测

% 计算分类准确度
accuracy = sum(predictedLabels == testLabels) / length(testLabels);
disp(['Classification accuracy: ' num2str(accuracy*100) '%']);
