function SerialRx_Timer
scomss = instrfind
t=timer;
t.StartDelay = 0.1;  %��ʱ1�뿪ʼ
t.ExecutionMode = 'fixedRate';  %����ѭ��ִ��
t.Period = 2;  %ѭ�����2��
t.TasksToExecute = 1;  %ѭ������3��
t.TimerFcn = @SerialRx;  %��ʼִ��
start(t)
end

function SerialRx(a,b,c)
fwrite(scomss,181,'int16');
end