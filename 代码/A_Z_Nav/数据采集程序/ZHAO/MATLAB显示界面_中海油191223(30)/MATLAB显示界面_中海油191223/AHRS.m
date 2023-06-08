%%%-------  Function Description  --------%%%
%{
AHRS显示界面：（源程序：Micrium_F411_V11&206）--可发送/连续发送数据
接收保存：1.卡尔曼滤波姿态； 2.加速度、陀螺仪、磁传感器原始数据；3.温度传感器数据。          
输出显示：1.加速度、陀螺仪、磁传感器原始数据； 2.姿态数据。          
数据协议：30个数据/组。其中数据头3个，校验位1个，姿态数据6个，温度数据1个，
          加速度、陀螺仪、磁传感器原始数据共18个，。
%}
function varargout = AHRS(varargin)
% AHRS MATLAB code for AHRS.fig
%      AHRS, by itself, creates a new AHRS or raises the existing
%      singleton*.
%
%      H = AHRS returns the handle to a new AHRS or the handle to
%      the existing singleton*.
%
%      AHRS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AHRS.M with the given input arguments.
%
%      AHRS('Property','Value',...) creates a new AHRS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AHRS_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AHRS_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AHRS

% Last Modified by GUIDE v2.5 25-Jul-2022 14:50:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AHRS_OpeningFcn, ...
                   'gui_OutputFcn',  @AHRS_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before AHRS is made visible.
function AHRS_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AHRS (see VARARGIN)
warning off all;
javaFrame = get(hObject,'JavaFrame');
javaFrame.setFigureIcon(javax.swing.ImageIcon('icon.jpg'));%改变窗口左上角图标
hasData = false;  %表征串口是否接收到数据
isShow = false;   %表征是否正在进行数据显示，即是否正在执行函数dataDisp
isStopDisp = false;%表征是否按下了【停止显示】按钮
isWork = true;%表征是否正在工作
%y_start = str2num(get(handles.y_start,'string')); %默认自动显示y轴最小刻度
%y_end = str2num(get(handles.y_end,'string')); %默认自动显示y轴最小刻度
global scom;
magx_raw = 0;
magy_raw = 0;
magz_raw = 0;
accx_raw = 0;
accy_raw = 0;
accz_raw = 0;
gyrox_raw = 0;
gyroy_raw = 0;
gyroz_raw = 0;
% gyrox_raw1 = 0;
% gyroy_raw1 = 0;
% gyroz_raw1 = 0;
%****额外数据***
% extra_x = 0;
% extra_y = 0;
% extra_z = 0;
% checkbit = 0;  %校验位
%**************
Pitch = 0;
Roll = 0;
Yaw = 0;
HX=[];
HY=[];
HZ=[];
AX=[];
AY=[];
AZ=[];
GX=[];
GY=[];
GZ=[];
% GX1=[];
% GY1=[];
% GZ1=[];
EX=[];
EY=[];
EZ=[];
PITCH=[];
ROLL=[];
YAW=[];
CKB = [];  %保存校验位

%以上大写为估计航姿
N=[];%每次读取的字节数

%%将上述参数作为应用数据，存入窗口对象内
setappdata(hObject,'hasData',hasData);
setappdata(hObject,'magx_raw',magx_raw);
setappdata(hObject,'magy_raw',magy_raw);
setappdata(hObject,'magz_raw',magz_raw);
setappdata(hObject,'accx_raw',accx_raw);
setappdata(hObject,'accy_raw',accy_raw);
setappdata(hObject,'accz_raw',accz_raw);
setappdata(hObject,'gyrox_raw',gyrox_raw);
setappdata(hObject,'gyroy_raw',gyroy_raw);
setappdata(hObject,'gyroz_raw',gyroz_raw);
% setappdata(hObject,'extra_x',extra_x);
% setappdata(hObject,'extra_y',extra_y);
% setappdata(hObject,'extra_z',extra_z);
setappdata(hObject,'Pitch',Pitch);
setappdata(hObject,'Roll',Roll);
setappdata(hObject,'Yaw',Yaw);
setappdata(hObject,'isShow',isShow);
setappdata(hObject,'isStopDisp',isStopDisp);
setappdata(hObject,'isWork',isWork);
setappdata(hObject,'HX',HX);
setappdata(hObject,'HY',HY);
setappdata(hObject,'HZ',HZ);
setappdata(hObject,'AX',AX);
setappdata(hObject,'AY',AY);
setappdata(hObject,'AZ',AZ);
% setappdata(hObject,'GX1',GX1);
% setappdata(hObject,'GY1',GY1);
% setappdata(hObject,'GZ1',GZ1);
setappdata(hObject,'EX',EX);
setappdata(hObject,'EY',EY);
setappdata(hObject,'EZ',EZ);
setappdata(hObject,'PITCH',PITCH);
setappdata(hObject,'ROLL',ROLL);
setappdata(hObject,'YAW',YAW);
% setappdata(hObject,'y_start',y_start);
% setappdata(hObject,'y_end',y_end);
setappdata(hObject,'CKB',CKB);
setappdata(hObject,'N',N);

%     global XData; %定义一全局变量，坐标轴X轴的数据
%     global YData1;%用来存储Yaw线的数组
%     global YData2;%用来存储Pitch线的数组
%     global YData3;%用来存储Roll线的数组
%     global Xlim;%坐标轴X轴的范围
    global L1;%用来存储Yaw线的句柄
    global L2;%用来存储Pitch线的句柄
    global L3;%用来存储Roll线的句柄
    XData = 0;%初始化坐标轴的数据为0
    YData1 = 0;
    YData2 = 0;
    YData3 = 0;
    Xlim = 0;
    L1 = plot(handles.axes1,XData,YData1,'c','EraseMode','none','MarkerSize',5);
    hold on;
    L2 = plot(handles.axes1,XData,YData2,'b','EraseMode','none','MarkerSize',5);
    hold on;
    L3 = plot(handles.axes1,XData,YData3,'m','EraseMode','none','MarkerSize',5);
    
    h = legend('Yaw','Pitch','Roll',2);
    % legend('boxoff') ;
    set(h,'Interpreter','none')
    set(h,'FontSize',6);
    set(handles.axes1,'XLim',[Xlim Xlim+1000],'YLim',[-180 +180]);%设定坐标轴的范围
    grid on; %绘出网格


% Choose default command line output for AHRS
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AHRS wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = AHRS_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on selection change in com.
function com_Callback(hObject, eventdata, handles)
% hObject    handle to com (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns com contents as cell array
%        contents{get(hObject,'Value')} returns selected item from com

% --- Executes during object creation, after setting all properties.
function com_CreateFcn(hObject, eventdata, handles)
% hObject    handle to com (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function mag_x_Callback(hObject, eventdata, handles)
% hObject    handle to mag_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mag_x as text
%        str2double(get(hObject,'String')) returns contents of mag_x as a double

% --- Executes during object creation, after setting all properties.
function mag_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mag_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function mag_y_Callback(hObject, eventdata, handles)
% hObject    handle to mag_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mag_y as text
%        str2double(get(hObject,'String')) returns contents of mag_y as a double

% --- Executes during object creation, after setting all properties.
function mag_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mag_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function mag_z_Callback(hObject, eventdata, handles)
% hObject    handle to mag_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mag_z as text
%        str2double(get(hObject,'String')) returns contents of mag_z as a double

% --- Executes during object creation, after setting all properties.
function mag_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mag_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function acc_x_Callback(hObject, eventdata, handles)
% hObject    handle to acc_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of acc_x as text
%        str2double(get(hObject,'String')) returns contents of acc_x as a double

% --- Executes during object creation, after setting all properties.
function acc_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to acc_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function acc_y_Callback(hObject, eventdata, handles)
% hObject    handle to acc_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of acc_y as text
%        str2double(get(hObject,'String')) returns contents of acc_y as a double

% --- Executes during object creation, after setting all properties.
function acc_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to acc_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function acc_z_Callback(hObject, eventdata, handles)
% hObject    handle to acc_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of acc_z as text
%        str2double(get(hObject,'String')) returns contents of acc_z as a double

% --- Executes during object creation, after setting all properties.
function acc_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to acc_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function gyro_x_Callback(hObject, eventdata, handles)
% hObject    handle to gyro_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gyro_x as text
%       str2double(get(hObject,'String')) returns contents of gyro_x as a double

% --- Executes during object creation, after setting all properties.
function gyro_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gyro_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function gyro_y_Callback(hObject, eventdata, handles)
% hObject    handle to gyro_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gyro_y as text
%        str2double(get(hObject,'String')) returns contents of gyro_y as a double

% --- Executes during object creation, after setting all properties.
function gyro_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gyro_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function gyro_z_Callback(hObject, eventdata, handles)
% hObject    handle to gyro_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gyro_z as text
%        str2double(get(hObject,'String')) returns contents of gyro_z as a double

% --- Executes during object creation, after setting all properties.
function gyro_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gyro_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in rate.
function rate_Callback(hObject, eventdata, handles)
% hObject    handle to rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns rate contents as cell array
%        contents{get(hObject,'Value')} returns selected item from rate

% --- Executes during object creation, afte r setting all properties.
function rate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in jiaoyan.
function jiaoyan_Callback(hObject, eventdata, handles)
% hObject    handle to jiaoyan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns jiaoyan contents as cell array
%        contents{get(hObject,'Value')} returns selected item from jiaoyan

% --- Executes during object creation, after setting all properties.
function jiaoyan_CreateFcn(hObject, eventdata, handles)
% hObject    handle to jiaoyan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in data_bits.
function data_bits_Callback(hObject, eventdata, handles)
% hObject    handle to data_bits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns data_bits contents as cell array
%        contents{get(hObject,'Value')} returns selected item from data_bits

% --- Executes during object creation, after setting all properties.
function data_bits_CreateFcn(hObject, eventdata, handles)
% hObject    handle to data_bits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in stop_bits.
function stop_bits_Callback(hObject, eventdata, handles)
% hObject    handle to stop_bits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns stop_bits contents as cell array
%        contents{get(hObject,'Value')} returns selected item from stop_bits

% --- Executes during object creation, after setting all properties.
function stop_bits_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stop_bits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in start_serial.
function start_serial_Callback(hObject, eventdata, handles)
% hObject    handle to start_serial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%     global Checkbox1;%用来判定是否选中了Yaw复选框
%     global Checkbox2;%用来判定是否选中了Pitch复选框
%     global Checkbox3;%用来判定是否选中了Roll复选框
    global scom;
%     Checkbox1 = 0;%初始化为未选中
%     Checkbox2 = 0;
%     Checkbox3 = 0;
    
if get(hObject,'value')
    % %获取串口端口名
    com_n = GetSerialCom();  %自动获取串口号
    set(handles.com,'value',str2double(com_n(end)));
    %com_n = sprintf('COM%d',get(handles.com,'value'));
    rates = [300 600 1200 2400 4800 9600 19200 38400 43000 56000 57600 115200];
    baud_rate = rates(get(handles.rate,'value'));
    % %获取校验位
    switch get(handles.jiaoyan,'value')
        case 1
            jiaoyan = 'none';
        case 2
            jiaoyan = 'odd';
        case 3
            jiaoyan = 'even';
    end
    %％获取数据位个数
    data_bits = 9 - get(handles.data_bits,'value');
    %%获取停止位个数
    stop_bits = get(handles.stop_bits,'value');
    %%创建串口对象
    
    scom = serial(com_n);
    scom.InputBufferSize=13107200;%接收缓冲区大小 12M
    scom.OutputBufferSize=5120;%发送缓冲区大小 5K
    %%配置串口属性
    set(scom,'BaudRate',baud_rate,'Parity',jiaoyan,'DataBits',...
        data_bits,'StopBits',stop_bits,'BytesAvailableFcnCount',300,...  %%%接送数据个数，此处22字节,响应每次只读取前22字节也是一种方案，但是响应太频繁使得图像显示和保存数据的数组占用内存迅速占满！或者去4400，减少响应频次！
        'BytesAvailableFcnMode','byte','BytesAvailableFcn',{@bytes,handles},...
        'TimerPeriod',0.01,'TimerFcn',{@dataDisp,handles});
    %%将串口对象的句柄作为用户数据，存入窗口对象
    set(handles.figure1,'UserData',scom);
    try
        fopen(scom);
        %msgbox('串口已打开！');
    catch
        msgbox('无效串口！');
        set(hObject,'value',0); %弹起本按钮
        return;
    end
    set(hObject,'String','关闭串口');%%设置为关闭选择
else  %若关闭串口
    t = timerfind;
    if ~isempty(t)
        stop(t);
        delete(t);
    end
    %%停止并删除串口对象
    scoms = instrfind;
    stopasync(scoms);
    fclose(scoms);
    delete(scoms);
    set(hObject,'String','打开串口');
end
    
% Hint: get(hObject,'Value') returns toggle state of start_serial

%%%自动识别串口号
function com = GetSerialCom()

Skey = 'HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM';
[~, list] = dos(['REG QUERY ' Skey]);
%if ischar(list) && strcmp('ERROR',list(1:5))  %% strcmp 两个字符串相同返回1
if size(list,2) == 1
    com = 'COM2';    %默认为com2
else
    list = strread(list,'%s','delimiter',' '); %#ok<FPARK> requires strread()
    com = cell2mat(list(numel(list)));
end


function dataDisp(obj,event,handles)

% 串口的TimerFcn回调函数
% 串口数据显示
% 获取参数
hasData = getappdata(handles.figure1,'hasData');
isStopDisp = getappdata(handles.figure1,'isStopDisp');
magx_raw = getappdata(handles.figure1,'magx_raw');
magy_raw = getappdata(handles.figure1,'magy_raw');
magz_raw = getappdata(handles.figure1,'magz_raw');
accx_raw = getappdata(handles.figure1,'accx_raw');
accy_raw = getappdata(handles.figure1,'accy_raw');
accz_raw = getappdata(handles.figure1,'accz_raw');
gyrox_raw = getappdata(handles.figure1,'gyrox_raw');
gyroy_raw = getappdata(handles.figure1,'gyroy_raw');
gyroz_raw = getappdata(handles.figure1,'gyroz_raw');
% gyrox_raw1 = getappdata(handles.figure1,'gyrox_raw1');
% gyroy_raw1 = getappdata(handles.figure1,'gyroy_raw1');
% gyroz_raw1 = getappdata(handles.figure1,'gyroz_raw1');
Pitch = getappdata(handles.figure1,'Pitch');
Roll = getappdata(handles.figure1,'Roll');
Yaw = getappdata(handles.figure1,'Yaw');

%若串口没有接收到数据，先尝试接收串口数据
% if ~hasData
%     bytes(obj,event,handles);
% end
%%若串口有数据，显示串口数据
if hasData && ~isStopDisp
    %执行显示数据模块时，不接受串口数据，即不执行BytesAvailableFcn回调函数
    setappdata(handles.figure1,'isShow',true);
    %%显示数据
    set(handles.mag_x,'string',magx_raw);
    set(handles.mag_y,'string',magy_raw);
    set(handles.mag_z,'string',magz_raw);
    set(handles.acc_x,'string',accx_raw);
    set(handles.acc_y,'string',accy_raw);
    set(handles.acc_z,'string',accz_raw);
    set(handles.gyro_x,'string',gyrox_raw);
    set(handles.gyro_y,'string',gyroy_raw);
    set(handles.gyro_z,'string',gyroz_raw);
    set(handles.pitch,'string',Pitch);
    set(handles.roll,'string',Roll);
    set(handles.yaw,'string',Yaw);
    
    %更新hasData标志，表示串口数据已显示
    setappdata(handles.figure1,'hasData',false);
    %%给数据显示模块解锁
    setappdata(handles.figure1,'isShow',false);
end

function bytes(obj,~,handles)

%     global XData;
%     global YData1;
%     global YData2;
%     global YData3;
%     global Xlim;
%     global Checkbox1;
%     global Checkbox2;
%     global Checkbox3;
    global L1;
    global L2;
    global L3;
    
%串口的BytesAvailableFcn回调函数
%tic%计时开始
%strRec = getappdata(handles.figure1,'strRec'); %获取串口要显示的数据
isStopDisp = getappdata(handles.figure1,'isStopDisp');%是否按下了停止显示的按钮
isShow = getappdata(handles.figure1,'isShow');%是否正在执行显示数据的操作
isWork = getappdata(handles.figure1,'isWork');%是否正在工作
%若正在执行数据显示操作，暂不接收串口数据
% if isShow
%     return;
% end
%%串口接收数据
%获取参数  
HX = getappdata(handles.figure1,'HX');
HY = getappdata(handles.figure1,'HY');
HZ = getappdata(handles.figure1,'HZ');
AX = getappdata(handles.figure1,'AX');
AY = getappdata(handles.figure1,'AY');
AZ = getappdata(handles.figure1,'AZ');
GX = getappdata(handles.figure1,'GX');
GY = getappdata(handles.figure1,'GY');
GZ = getappdata(handles.figure1,'GZ');
% GX1 = getappdata(handles.figure1,'GX1');
% GY1 = getappdata(handles.figure1,'GY1');
% GZ1 = getappdata(handles.figure1,'GZ1');
EX = getappdata(handles.figure1,'EX');
EY = getappdata(handles.figure1,'EY');
EZ = getappdata(handles.figure1,'EZ');
PITCH = getappdata(handles.figure1,'PITCH');
ROLL = getappdata(handles.figure1,'ROLL');
YAW = getappdata(handles.figure1,'YAW');
CKB = getappdata(handles.figure1,'CKB');
N = getappdata(handles.figure1,'N');
% y_start = getappdata(handles.figure1,'y_start');
% y_end = getappdata(handles.figure1,'y_end');

%persistent Xhat;
%获取串口可获取的数据字数
n = get(obj,'BytesAvailable');
len = 30;  % 每组数据的长度
if (n > 2*len)
   %out=dec2hex(fread(obj,22,'uint8'));
   %nc = fix(n/40)*40;
   out0=fread(obj,n,'uint8');
   while ~(out0(1)==250 && out0(2)==250 && out0(3)==254)
       out0(1)=[];
   end
   %out=dec2hex(out0);
   out=out0';
   count_n = fix(size(out,2)/len)-1;
   %count_n = 0;
 if isWork 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   for iii = 0:count_n
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%直接接收卡尔曼滤波航姿%%%%%%%%%%%%%%%%%%%%%%%%%%%      
 pitch_raw = out(6+iii*len)*256+out(5+iii*len);
 if (pitch_raw > 32768)
     pitch_raw = pitch_raw - 65536;
 end
 Pitch = pitch_raw/100;
 roll_raw = out(8+iii*len)*256+out(7+iii*len);
 if (roll_raw > 32768)
     roll_raw = roll_raw - 65536;
 end
 Roll = roll_raw/100;
 yaw_raw = out(10+iii*len)*256+out(9+iii*len);
 Yaw = yaw_raw/100;
 %%%%%%%%%%%%%%%%%%%%%%%%接收额外数据（此时为角速度，可调整）%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
 if (out(4+iii*len) == mod(sum(out(5+iii*len:len+iii*len)),256))
     checkbit = 1;
 else
     checkbit = 0;
 end
   
%    Extra_x = strcat(out(1,18+iii*40),out(2,18+iii*40),out(1,17+iii*40),out(2,17+iii*40));
   extra_x = out(30+iii*len)*256+out(29+iii*len);
   if (extra_x > 32768)
       extra_x = extra_x - 65536;
   end
   extra_x = extra_x*0.125;
   extra_y =0;
   extra_z =0;
%{  
   extra_y = out(32+iii*len)*256+out(31+iii*len);
%    if (extra_y > 32768)
%        extra_y = extra_y - 65536;
%    end
   extra_y = extra_y/1000;
    
   extra_z = out(34+iii*len)*256+out(33+iii*len);
   if (extra_z > 32768)
      extra_z = extra_z - 65536;
   end
   extra_z = extra_z/1000;
   %}
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%接收传感器原始数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   accx_raw = out(12+iii*len)*256+out(11+iii*len);
   if (accx_raw > 32768)
       accx_raw = accx_raw - 65536;
   end   
   accy_raw = out(14+iii*len)*256+out(13+iii*len);
   if (accy_raw > 32768)
       accy_raw = accy_raw - 65536;
   end   
   accz_raw = out(16+iii*len)*256+out(15+iii*len);
   if (accz_raw > 32768)
       accz_raw = accz_raw - 65536;
   end
   
   gyrox_raw = out(18+iii*len)*256+out(17+iii*len);
   if (gyrox_raw > 32768)
       gyrox_raw = gyrox_raw - 65536;
   end
   gyroy_raw = out(20+iii*len)*256+out(19+iii*len);
   if (gyroy_raw > 32768)
       gyroy_raw = gyroy_raw - 65536;
   end
   gyroz_raw =out(22+iii*len)*256+out(21+iii*len);
   if (gyroz_raw > 32768)
       gyroz_raw = gyroz_raw - 65536;
   end
   
   magx_raw = out(24+iii*len)*256+out(23+iii*len);
   if (magx_raw > 32768)
       magx_raw = magx_raw - 65536;
   end
   magy_raw = out(26+iii*len)*256+out(25+iii*len);
   if (magy_raw > 32768)
       magy_raw = magy_raw - 65536;
   end
   magz_raw = out(28+iii*len)*256+out(27+iii*len);
   if (magz_raw > 32768)
       magz_raw = magz_raw - 65536;
   end
%       gyrox_raw1 = out(32+iii*len)*256+out(31+iii*len);
%    if (gyrox_raw1 > 32768)
%        gyrox_raw1 = gyrox_raw1 - 65536;
%    end
%     gyroy_raw1 = out(34+iii*len)*256+out(33+iii*len);
%    if (gyroy_raw1 > 32768)
%        gyroy_raw1 = gyroy_raw1 - 65536;
%    end
%     gyroz_raw1 = out(36+iii*len)*256+out(35+iii*len);
%    if (gyroz_raw1 > 32768)
%        gyroz_raw1 = gyroz_raw1 - 65536;
%    end
%     GX1=[GX1;gyrox_raw1];
%     GY1=[GY1;gyroy_raw1];
%     GZ1=[GZ1;gyroz_raw1];
   
    GX=[GX;gyrox_raw];
    GY=[GY;gyroy_raw];
    GZ=[GZ;gyroz_raw];
    HX=[HX;magx_raw];
    HY=[HY;magy_raw];
    HZ=[HZ;magz_raw];
    AX=[AX;accx_raw];
    AY=[AY;accy_raw];
    AZ=[AZ;accz_raw];
    
    PITCH=[PITCH;Pitch];
    ROLL=[ROLL;Roll];
    YAW=[YAW;Yaw];
    EX=[EX;extra_x];
    EY=[EY;extra_y];
    EZ=[EZ;extra_z];
    CKB=[CKB;checkbit];
    N = [N;n];

    %更新要显示数据
    setappdata(handles.figure1,'magx_raw',magx_raw);   
    setappdata(handles.figure1,'magy_raw',magy_raw);
    setappdata(handles.figure1,'magz_raw',magz_raw);
    setappdata(handles.figure1,'accx_raw',accx_raw);
    setappdata(handles.figure1,'accy_raw',accy_raw);
    setappdata(handles.figure1,'accz_raw',accz_raw);
    setappdata(handles.figure1,'gyrox_raw',gyrox_raw);
    setappdata(handles.figure1,'gyroy_raw',gyroy_raw);
    setappdata(handles.figure1,'gyroz_raw',gyroz_raw);
    setappdata(handles.figure1,'Pitch',Pitch);
    setappdata(handles.figure1,'Roll',Roll);
    setappdata(handles.figure1,'Yaw',Yaw);

    %更新要保存的全局数据extra_x
    setappdata(handles.figure1,'GX',GX);
    setappdata(handles.figure1,'GY',GY);
    setappdata(handles.figure1,'GZ',GZ);
%      setappdata(handles.figure1,'GX1',GX1);
%     setappdata(handles.figure1,'GY1',GY1);
%     setappdata(handles.figure1,'GZ1',GZ1);
    setappdata(handles.figure1,'AX',AX);
    setappdata(handles.figure1,'AY',AY);
    setappdata(handles.figure1,'AZ',AZ);
    setappdata(handles.figure1,'HX',HX);
    setappdata(handles.figure1,'HY',HY);
    setappdata(handles.figure1,'HZ',HZ);
    setappdata(handles.figure1,'PITCH',PITCH);
    setappdata(handles.figure1,'ROLL',ROLL);
    setappdata(handles.figure1,'YAW',YAW);
    setappdata(handles.figure1,'EX',EX);
    setappdata(handles.figure1,'EY',EY);
    setappdata(handles.figure1,'EZ',EZ);
    setappdata(handles.figure1,'CKB',CKB);
    setappdata(handles.figure1,'N',N);
    %更新数据接收计数
    set(handles.DataSize,'string',size(AX,1));
    set(handles.temperature,'string',[num2str(extra_x) '  ℃']);
    
    % 每运行一次本函数X轴的数据+1
    %Xlim = Xlim+1;size(YAW,1)
    %XData =[XData ; Xlim];
    Xlim = size(YAW,1);
    XData = (1:Xlim)';
    YData1 = YAW;  % YAW;
    YData2 = PITCH;  % PITCH;
    YData3 = ROLL;  % ROLL;
    %    YData1 = [0;HX];         %更改Y轴显示值
    %    YData2 = [0;HY];
    %    YData3 = [0;HZ];
    
    % 限定坐标轴X Y的数组长度不能超过100，防止数据过多时易导致内存消耗过大卡死
    if (length(XData)) > 1000
        XData = XData(end-999:end);%数组长度一旦超过100就丢弃第1列的值
        YData1 = YData1(end-999:end);
        YData2 = YData2(end-999:end);
        YData3 = YData3(end-999:end);
    end
    
    if get(handles.checkbox1,'value')
        Checkbox1 = 1;
        set(L1,'Xdata',XData,'YData',YData1);
    else
        Checkbox1 = 0;
    end
    if get(handles.checkbox2,'value')
        Checkbox2 = 1;
        set(L2,'Xdata',XData,'YData',YData2);
    else
        Checkbox2 = 0;
    end
    if get(handles.checkbox3,'value')
        Checkbox3 = 1;
        set(L3,'Xdata',XData,'YData',YData3);
    else
        Checkbox3 = 0;
    end
    
    % 求出Y轴数据的最小最大值
    min_1 = min(YData1);
    min_2 = min(YData2);
    min_3 = min(YData3);
    max_1 = max(YData1);
    max_2 = max(YData2);
    max_3 = max(YData3);
    % 通过判定复选框的勾选值来采用相应的最小最大值，并求出整个Y轴数据的最小最大值
    if Checkbox1 && Checkbox2 && Checkbox3
        min_array = [min_1 min_2 min_3];
        max_array = [max_1 max_2 max_3];
    elseif Checkbox1 && Checkbox2
        min_array = [min_1 min_2];
        max_array = [max_1 max_2];
    elseif Checkbox2 && Checkbox3
        min_array = [min_2 min_3];
        max_array = [max_2 max_3];
    elseif Checkbox1 && Checkbox3
        min_array = [min_1 min_3];
        max_array = [max_1 max_3];
    elseif Checkbox1
        min_array = min_1;
        max_array = max_1;
    elseif Checkbox2
        min_array = min_2;
        max_array = max_2;
    elseif Checkbox3
        min_array = min_3;
        max_array = max_3;
    else
        min_array = -180 ;
        max_array = 180 ;
    end
    min_all = min(min_array);
    max_all = max(max_array);
    % 更新坐标轴范围
    if (Checkbox1 || Checkbox2 || Checkbox3) && (length(XData)<1000)
        set(handles.axes1,'XLim',[0  1000],'YLim',[min_all-1  max_all+1]);
        %set(handles.axes1,'XLim',[0  1000],'YLim',[y_start  y_end]);
        %set(handles.axes1,'XLim',[0  1000],'YLim',[-10  10]);            %更改Y轴显示范围
    end
    if (Checkbox1 || Checkbox2 || Checkbox3) && (length(XData)==1000)
        set(handles.axes1,'XLim',[Xlim-999  Xlim+1],'YLim',[min_all-1  max_all+1]);
        %set(handles.axes1,'XLim',[Xlim-999  Xlim+1],'YLim',[y_start  y_end]);
        %set(handles.axes1,'XLim',[Xlim-999  Xlim+1],'YLim',[-10  10]);    %更改Y轴显示范围
    end
    setappdata(handles.figure1,'hasData',true);%更新hasData参数，表明串口有数据需要显示
    %     toc
   end
 end
end

%toc%计时结束


% --- Executes on button press in stop_disp.
function stop_disp_Callback(hObject, eventdata, handles)
% hObject    handle to stop_disp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')
    isWork = false;
    isStopDisp = true;
    set(hObject,'String','继续显示');
else
    isWork = true;
    isStopDisp = false;
    set(hObject,'String','暂停显示');
end
setappdata(handles.figure1,'isStopDisp',isStopDisp);
setappdata(handles.figure1,'isWork',isWork);
% Hint: get(hObject,'Value') returns toggle state of stop_disp


% --- Executes on button press in qingkong.
function qingkong_Callback(hObject, eventdata, handles)
%%清空要保存的数据
setappdata(handles.figure1,'AX',[]);
setappdata(handles.figure1,'AY',[]);
setappdata(handles.figure1,'AZ',[]);
setappdata(handles.figure1,'HX',[]);
setappdata(handles.figure1,'HY',[]);
setappdata(handles.figure1,'HZ',[]);
setappdata(handles.figure1,'GX',[]);
setappdata(handles.figure1,'GY',[]);
setappdata(handles.figure1,'GZ',[]);
% setappdata(handles.figure1,'GX1',[]);
% setappdata(handles.figure1,'GY1',[]);
% setappdata(handles.figure1,'GZ1',[]);
setappdata(handles.figure1,'PITCH',[]);
setappdata(handles.figure1,'ROLL',[]);
setappdata(handles.figure1,'YAW',[]);
setappdata(handles.figure1,'EX',[]);
setappdata(handles.figure1,'EY',[]);
setappdata(handles.figure1,'EZ',[]);
setappdata(handles.figure1,'CKB',[]);
setappdata(handles.figure1,'N',[]);

global L1;
global L2;
global L3;
set(L1,'Xdata',0,'YData',0);
set(L2,'Xdata',0,'YData',0);
set(L3,'Xdata',0,'YData',0);
hold on;
set(handles.axes1,'XLim',[0 1000],'YLim',[-180  180]);%设定坐标轴的范围
%清空显示
setappdata(handles.figure1,'hasData',true);%更新hasData参数，表明串口有数据需要显示
setappdata(handles.figure1,'magx_raw',0);
setappdata(handles.figure1,'magy_raw',0);
setappdata(handles.figure1,'magz_raw',0);
setappdata(handles.figure1,'accx_raw',0);
setappdata(handles.figure1,'accy_raw',0);
setappdata(handles.figure1,'accz_raw',0);
setappdata(handles.figure1,'gyrox_raw',0);
setappdata(handles.figure1,'gyroy_raw',0);
setappdata(handles.figure1,'gyroz_raw',0);
% setappdata(handles.figure1,'gyrox_raw1',0);
% setappdata(handles.figure1,'gyroy_raw1',0);
% setappdata(handles.figure1,'gyroz_raw1',0);
setappdata(handles.figure1,'Pitch',0);
setappdata(handles.figure1,'Roll',0);
setappdata(handles.figure1,'Yaw',0);
set(handles.DataSize,'string',0);
set(handles.temperature,'string',0);

isStopDisp = false;
setappdata(handles.figure1,'isStopDisp',isStopDisp);
disp([datestr(now,'yyyy.mm.dd HH:MM:SS') ' :  Data cleared !']);
 
% hObject    handle to qingkong (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in save_data.
function save_data_Callback(hObject, eventdata, handles)
%hObject    handle to save_data (see GCBO)
 isStopDisp = true;
 setappdata(handles.figure1,'isStopDisp',isStopDisp);  
 isWork = false;
 setappdata(handles.figure1,'isWork',isWork);
 if ~get(handles.stop_disp,'Value')    % 判断是否按下“停止显示”按钮
     set(handles.stop_disp,'Value',1);    % 相当于按下“停止显示”按钮
     set(handles.stop_disp,'String','继续显示');
     %get(handles.stop_disp,'Value')
 end

GX=getappdata(handles.figure1,'GX');
GY=getappdata(handles.figure1,'GY');
GZ=getappdata(handles.figure1,'GZ');
% GX1=getappdata(handles.figure1,'GX1');
% GY1=getappdata(handles.figure1,'GY1');
% GZ1=getappdata(handles.figure1,'GZ1');
AX=getappdata(handles.figure1,'AX');
AY=getappdata(handles.figure1,'AY');
AZ=getappdata(handles.figure1,'AZ');
HX=getappdata(handles.figure1,'HX');
HY=getappdata(handles.figure1,'HY');
HZ=getappdata(handles.figure1,'HZ');
EX=getappdata(handles.figure1,'EX');
EY=getappdata(handles.figure1,'EY');
EZ=getappdata(handles.figure1,'EZ');
PITCH=getappdata(handles.figure1,'PITCH');
ROLL=getappdata(handles.figure1,'ROLL');
YAW=getappdata(handles.figure1,'YAW');
CKB=getappdata(handles.figure1,'CKB');
N=getappdata(handles.figure1,'N');
%WENDU=getappdata(gcf,'WENDU');

[fName,pName]=uiputfile({'*.xls';'*.*'},'save as','Z+');              %创建文件保存对话框；
    str_1=[pName fName];                                                    %获取文件的完整路径和文件名；
if isequal(fName,0)||isequal(pName,0)%如果用户选择“取消”，则退出
    return;
else
    fpath=fullfile(pName,fName);
end
data = [GX,GY,GZ,AX,AY,AZ,HX,HY,HZ,EX,EY,EZ,PITCH,ROLL,YAW,N,CKB];                           % 将数据组集到data

[m, n] = size(data);            

data_cell = mat2cell(data, ones(m,1), ones(n,1));    % 将data切割成m*n的cell矩阵

title = {'Gyro_x','Gyro_y','Gyro_z','Acc_X','Acc_Y','Acc_Z','Mag_X','Mag_Y','Mag_Z','EX','EY','EZ','PITCH','ROLL','YAW','N','CheckBit'};                          % 添加变量名称

result = [title; data_cell];                                            % 将变量名称和数值组集到result
xlswrite(str_1,result,'sheet1','A1');
disp([datestr(now,'yyyy.mm.dd HH:MM:SS') ' :  Data have saved !']);    %显示数据保存完成信息
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
function yaw_Callback(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw as text
%        str2double(get(hObject,'String')) returns contents of yaw as a double

% --- Executes during object creation, after setting all properties.
function yaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function pitch_Callback(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitch as text
%        str2double(get(hObject,'String')) returns contents of pitch as a double

% --- Executes during object creation, after setting all properties.
function pitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function roll_Callback(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll as text
%        str2double(get(hObject,'String')) returns contents of roll as a double

% --- Executes during object creation, after setting all properties.
function roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%查找定时器
t = timerfind;
if ~isempty(t)
    stop(t);
    delete(t);
end
%查找串口对象
scoms = instrfind;
%尝试停止、关闭删除串口对象
try
    stopasync(scoms);
    fclose(scoms);
    delete(scoms);
end
% Hint: delete(hObject) closes the figure
delete(hObject);

% --- Executes on button press in start_calibration.
function start_calibration_Callback(hObject, eventdata, handles)

%%清空要保存的数据

    setappdata(handles.figure1,'GY',[]);
    setappdata(handles.figure1,'GZ',[]);
    setappdata(handles.figure1,'GX',[]);
%     setappdata(handles.figure1,'GY1',[]);
%     setappdata(handles.figure1,'GZ1',[]);
%     setappdata(handles.figure1,'GX1',[]);
    setappdata(handles.figure1,'AY',[]);
    setappdata(handles.figure1,'AZ',[]);
    setappdata(handles.figure1,'AX',[]);
    setappdata(handles.figure1,'HY',[]);
    setappdata(handles.figure1,'HZ',[]);
    setappdata(handles.figure1,'HX',[]);
    setappdata(handles.figure1,'EY',[]);
    setappdata(handles.figure1,'EZ',[]);
    setappdata(handles.figure1,'EX',[]);
    setappdata(handles.figure1,'PITCH',[]);
    setappdata(handles.figure1,'ROLL',[]);
    setappdata(handles.figure1,'YAW',[]);
    setappdata(handles.figure1,'N',[]); 
    setappdata(handles.figure1,'CKB',[]); 
     
    global L1;
    global L2;
    global L3;
    set(L1,'Xdata',0,'YData',0);
    set(L2,'Xdata',0,'YData',0);
    set(L3,'Xdata',0,'YData',0);
    hold on;   
    set(handles.axes1,'XLim',[0 1000],'YLim',[-180  180]);%设定坐标轴的范围
    
    %清空显示
    setappdata(handles.figure1,'hasData',true);%更新hasData参数，表明串口有数据需要显示
    setappdata(handles.figure1,'magx_raw',0);   
    setappdata(handles.figure1,'magy_raw',0);
    setappdata(handles.figure1,'magz_raw',0);
    setappdata(handles.figure1,'accx_raw',0);
    setappdata(handles.figure1,'accy_raw',0);
    setappdata(handles.figure1,'accz_raw',0);
    setappdata(handles.figure1,'gyrox_raw',0);
    setappdata(handles.figure1,'gyroy_raw',0);
    setappdata(handles.figure1,'gyroz_raw',0);
%     setappdata(handles.figure1,'gyrox_raw1',0);
%     setappdata(handles.figure1,'gyroy_raw1',0);
%     setappdata(handles.figure1,'gyroz_raw1',0);
    setappdata(handles.figure1,'Pitch',0);
    setappdata(handles.figure1,'Roll',0);
    setappdata(handles.figure1,'Yaw',0);
    set(handles.DataSize,'string',0);
    set(handles.temperature,'string',0);
    
    isStopDisp = false;
    setappdata(handles.figure1,'isStopDisp',isStopDisp);  
    isWork = true;
    setappdata(handles.figure1,'isWork',isWork);
    disp('Start Calibration ...');
% hObject    handle to start_calibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in end_calibration.
function end_calibration_Callback(hObject, eventdata, handles)

     isStopDisp = true;
     setappdata(handles.figure1,'isStopDisp',isStopDisp);  
     isWork = false;
     setappdata(handles.figure1,'isWork',isWork);
     disp('End Calibration !');
 
     HX=getappdata(handles.figure1,'HX');
     HY=getappdata(handles.figure1,'HY');
     HZ=getappdata(handles.figure1,'HZ');
        format long 
data = xlsread('Z+','sheet1');
m=ones(3,3);
m(1,1)=data(1,7);
m(1,2)=data(1,8);
m(1,3)=data(1,9);
m(2,1)=data(1,4);
m(2,2)=data(1,5);
m(2,3)=data(1,6);
m(3,1)=data(1,1);
m(3,2)=data(1,2);
m(3,3)=data(1,3);

row = size(data,1);

 m1=m/30000 %20211214
% m1=[   0.999988441810144   0.012261948043869  -0.000082648567323			
%   -0.002848942387917   1.000313489465641  -0.038778982105228			
%    0.005956987602399  -0.006434921957468   1.001002978755818			
% ];     
% %      cbddata = importdata('magcbdpara.txt');
% %      k = cbddata(1,:);
% %      b = cbddata(2,:);
% %      mt = cbddata(3:5,:);
%        k = [1554.6554,1602.3387,1582.6172];  %20211214特性
%       
% %      b = [46.372,487.0462,-228.9087]; %20170828特性   
% %      mt =[  0.999989627859896  -0.020131737118345   0.026940870168659
% %          0.003740393767438   0.999746621188537  -0.004415810576421
% %          -0.002598774151633  -0.010070083549846   0.999627275604017];   %20170828特性
% % 202112141
% 
% 
%       kInv = inv(diag(k));     
%     cbddata=inv(m1') *kInv   %%%错误！！
%    
%     
%     mb = 1.0e-03*cbddata
%    mb = 1.0e-03 * [    0.335854371405141  -0.00571203749220    0.001508155163148    
%                       -0.000960894251368   0.349495575642744  -0.024135832708992     
%                        0.001690322800047  -0.002439247971384   0.332800486376622			
%   ];  %% 20191210,CNOC#1
 mb = 1.0e-03 *m1;

     ellip_data = [HX,HY,HZ];
     data = ellip_data*mb;
    
     [cc, r] = sphereFit(data)
     cc=[-13062 flo2int(cc)]    %%% 发送float型cc，以0xfa 0xcc开头，低8位在前即0xccfa     
     %cc = [-13227 cc*1000];   %%% 发送cc小数点后3位% 发送的数据以0x55 0xcc开头，低8位在前即0xcc55
     scomss = instrfind;  %%%查找串口对象
     fwrite(scomss,cc,'int16');  %%%%%%%%注意这16位写入时低8位在前，高8位在后！  

     msgbox('标定已完成！');
      delete('Z+.xls');
     delete('spherefit.xlsx');
     
     %%清空要保存的数据
     setappdata(handles.figure1,'HX',[]);
     setappdata(handles.figure1,'HY',[]);
     setappdata(handles.figure1,'HZ',[]);
     setappdata(handles.figure1,'AX',[]);
     setappdata(handles.figure1,'AY',[]);
     setappdata(handles.figure1,'AZ',[]);
     setappdata(handles.figure1,'GY',[]);
     setappdata(handles.figure1,'GZ',[]);
     setappdata(handles.figure1,'GX',[]);
%      setappdata(handles.figure1,'GY1',[]);
%      setappdata(handles.figure1,'GZ1',[]);
%      setappdata(handles.figure1,'GX1',[]);
     setappdata(handles.figure1,'EY',[]);
     setappdata(handles.figure1,'EZ',[]);
     setappdata(handles.figure1,'EX',[]);
     setappdata(handles.figure1,'PITCH',[]);
     setappdata(handles.figure1,'ROLL',[]);
     setappdata(handles.figure1,'YAW',[]);
     setappdata(handles.figure1,'N',[]);
     setappdata(handles.figure1,'CKB',[]); 
      
     global L1;
     global L2;
     global L3;
     set(L1,'Xdata',0,'YData',0);
     set(L2,'Xdata',0,'YData',0);
     set(L3,'Xdata',0,'YData',0);
     hold on;
     set(handles.axes1,'XLim',[0 1000],'YLim',[-180  180]);%设定坐标轴的范围
     
     %清空显示
     setappdata(handles.figure1,'hasData',true);%更新hasData参数，表明串口有数据需要显示
     setappdata(handles.figure1,'magx_raw',0);
     setappdata(handles.figure1,'magy_raw',0);
     setappdata(handles.figure1,'magz_raw',0);
     setappdata(handles.figure1,'accx_raw',0);
     setappdata(handles.figure1,'accy_raw',0);
     setappdata(handles.figure1,'accz_raw',0);
     setappdata(handles.figure1,'gyrox_raw',0);
     setappdata(handles.figure1,'gyroy_raw',0);
     setappdata(handles.figure1,'gyroz_raw',0);
%      setappdata(handles.figure1,'gyrox_raw1',0);
%      setappdata(handles.figure1,'gyroy_raw1',0);
%      setappdata(handles.figure1,'gyroz_raw1',0);
     
     setappdata(handles.figure1,'Pitch',0);
     setappdata(handles.figure1,'Roll',0);
     setappdata(handles.figure1,'Yaw',0);
     set(handles.DataSize,'string',0);
     set(handles.temperature,'string',0);
     
     isStopDisp = false;
     isWork = true;
     setappdata(handles.figure1,'isStopDisp',isStopDisp);
     setappdata(handles.figure1,'isWork',isWork);
% hObject    handle to end_calibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1

% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2

% --- Executes on button press in checkbox3.
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3

function y_start_Callback(hObject, eventdata, handles)

% y_start = getappdata(handles.figure1,'y_start');
y_start = str2num(get(handles.y_start,'string'));
 setappdata(handles.figure1,'y_start',y_start);
% hObject    handle to y_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_start as text
%        str2double(get(hObject,'String')) returns contents of y_start as a double

% --- Executes during object creation, after setting all properties.
function y_start_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function y_end_Callback(hObject, eventdata, handles)

y_end = str2num(get(handles.y_end,'string'));
 setappdata(handles.figure1,'y_end',y_end);
% hObject    handle to y_end (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_end as text
%        str2double(get(hObject,'String')) returns contents of y_end as a double

% --- Executes during object creation, after setting all properties.
function y_end_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_end (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
 
%%%%%%%******************  数据发送  ********************************
% --- Executes on button press in bt_rxdata.
function bt_rxdata_Callback(hObject, eventdata, handles)
% hObject    handle to bt_rxdata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
scomss = instrfind;  %%%查找串口对象
rxd = hex2dec(get(handles.rxdata,'string'));
% 若要发送的数据不为空，发送数据
if ~isempty(rxd)
    % 获取串口的传输状态，若串口没有正在写数据，写入数据 %if ~(strcmp(str, 'write') || strcmp(str, 'read&write'))
    str = get(scomss, 'TransferStatus');   
    if strcmp(str, 'idle') 
        fwrite(scomss, rxd, 'uint8', 'async'); %数据写入串口
    end
end
%fwrite(scomss,rxd,'int16');

% --- Executes on button press in TimerRx.  【定时发送】按钮的Callback回调函数
function TimerRx_Callback(hObject, eventdata, handles)
% hObject    handle to TimerRx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%%% 若按下【自动发送】按钮，启动定时器；否则，停止并删除定时器 get(handles.TimerRx,'value')
if get(hObject, 'value')
    t1 = 0.001 * str2double(get(handles.Rx_perio, 'string'));  %获取定时器周期
    t = timer('ExecutionMode','fixedrate', 'Period', t1, 'TimerFcn',...
        {@bt_rxdata_Callback, handles}); %创建定时器
    set(handles.Rx_perio, 'Enable', 'off'); %禁用设置定时器周期的Edit Text对象
    %set(handles.sends, 'Enable', 'inactive'); %禁用数据发送编辑区
    start(t);  %启动定时器
else
    set(handles.Rx_perio, 'Enable', 'on'); %启用设置定时器周期的Edit Text对象
    %set(handles.sends, 'Enable', 'on');   %启用数据发送编辑区
    t = timerfind;  %查找定时器
    if ~isempty(t)
        stop(t); %停止定时器
        delete(t); %删除定时器
    end
end
% Hint: get(hObject,'Value') returns toggle state of TimerRx



function Rx_perio_Callback(hObject, eventdata, handles)
% hObject    handle to Rx_perio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Rx_perio as text
%        str2double(get(hObject,'String')) returns contents of Rx_perio as a double


% --- Executes during object creation, after setting all properties.
function Rx_perio_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Rx_perio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rxdata_Callback(hObject, eventdata, handles)

% Hints: get(hObject,'String') returns contents of rxdata as text
%        str2double(get(hObject,'String')) returns contents of rxdata as a double


% --- Executes during object creation, after setting all properties.
function rxdata_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rxdata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in gyr_calibration.
function gyr_calibration_Callback(hObject, eventdata, handles)
% hObject    handle to gyr_calibration (see GCBO)data = xlsread('Z+','sheet1');
    isStopDisp = true;
     setappdata(handles.figure1,'isStopDisp',isStopDisp);  
     isWork = false;
     setappdata(handles.figure1,'isWork',isWork);
     disp('End Calibration !');
     format long
 data = xlsread('Z+','sheet1');
 
row = size(data,1);
gyro_x=zeros(row,1);
gyro_y=zeros(row,1);
gyro_z=zeros(row,1);


gyro_x=data(row,1);
gyro_y=data(row,2);
gyro_z=data(row,3);

gyro_X=mean(gyro_x)
gyro_Y=mean(gyro_y)
gyro_Z=mean(gyro_z)

cc2=[gyro_X gyro_Y gyro_Z]
cc2=[-13062 flo2int(cc2)] 

       scomss = instrfind;  %%%查找串口对象
     fwrite(scomss,cc2,'int16');  %%%%%%%%注意这16位写入时低8位在前，高8位在后！  

     msgbox('校准已完成！');


row = size(data,1);

% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to bt_rxdata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
scomss = instrfind;  %%%查找串口对象
rxd1 =str2double( get(handles.edit17,'string') )
rxd2=[rxd1 1 1]
magdec=[ -13210 flo2int(rxd2)]
% 若要发送的数据不为空，发送数据
if ~isempty(rxd1)
    % 获取串口的传输状态，若串口没有正在写数据，写入数据 %if ~(strcmp(str, 'write') || strcmp(str, 'read&write'))
    str = get(scomss, 'TransferStatus');   
    if strcmp(str, 'idle')
      
        fwrite(scomss, magdec, 'int16'); %数据写入串口
    end
end% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit17_Callback(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double


% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object deletion, before destroying properties.
