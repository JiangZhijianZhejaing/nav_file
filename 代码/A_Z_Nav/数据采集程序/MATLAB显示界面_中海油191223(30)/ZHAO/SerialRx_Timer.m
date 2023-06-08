function SerialRx_Timer
scomss = instrfind
t=timer;
t.StartDelay = 0.1;  %延时1秒开始
t.ExecutionMode = 'fixedRate';  %启用循环执行
t.Period = 2;  %循环间隔2秒
t.TasksToExecute = 1;  %循环次数3次
t.TimerFcn = @SerialRx;  %开始执行
start(t)
end

function SerialRx(a,b,c)
fwrite(scomss,181,'int16');
end