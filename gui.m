function varargout = gui(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
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

function makeGlobal(my_robot)
global x
x = my_robot;

function r = getGlobal
global x
r = x;

function makeGlobalc(config)
global y
y = config;

function c = getGlobalc
global y
c =y;

function make_th1(th)
global t1
t1 = th;

function angle = get_th1
global t1;
angle = t1;

function make_th2(th)
global t2;
t2 = th;

function angle = get_th2
global t2
angle = t2;

function make_th3(th)
global t3
t3 = th;

function angle = get_th3
global t3
angle = t3;

function make_th4(th)
global t4
t4 = th;

function angle = get_th4
global t4
angle = t4;

function make_th5(th)
global t5
t5 = th;

function angle = get_th5
global t5
angle = t5;

function makeX(cord)
global x_target
x_target = cord;

function r = getX
global x_target
r = x_target;

function makeY(cord)
global y_target
y_target = cord;

function r = getY
global y_target
r = y_target;

function makeZ(cord)
global z_target
z_target = cord;

function r = getZ
global z_target
r = z_target;


function setDefault
path = 'C:/Users/josep/Desktop/JOSEE 5.1/Project/coding/urdf/urdf/assemblyurdf.SLDASM/urdf/assemblyurdf.SLDASM.urdf';
my_robot = importrobot(path);
config = homeConfiguration(my_robot);
makeGlobal(my_robot)
makeGlobalc(config)
show(my_robot,config);
xlim([-0.4000 0.4000])
ylim([-0.4000 0.4000])

view([110 40])
function gui_OpeningFcn(hObject, eventdata, handles, varargin)
setDefault

% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

% Choose default command line output for gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

function varargout = gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function display_robot
r = getGlobal;
config = getGlobalc;

th1 = get_th1;
th2 = get_th2;
th3 = get_th3;
th4 = get_th4;
th5 = get_th5;

config(1).JointPosition = deg2rad(th1); %Assign theta 1 to first joint
config(2).JointPosition = deg2rad(th2);
config(3).JointPosition = deg2rad(th3);
config(4).JointPosition = deg2rad(th4);
config(5).JointPosition = deg2rad(th5);

show(r,config);
view([110 40])

xlim([-0.4000 0.4000])
ylim([-0.4000 0.4000])

function slider1_Callback(hObject, eventdata, handles)
th = get(hObject,'Value');
make_th1(th);
display_robot

function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slider2_Callback(hObject, eventdata, handles)
th = get(hObject,'Value');
make_th2(th)
display_robot

function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function slider3_Callback(hObject, eventdata, handles)
th = get(hObject,'Value');
make_th3(th)
display_robot

function slider3_CreateFcn(hObject, eventdata, handles)

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slider4_Callback(hObject, eventdata, handles)
th = get(hObject,'Value');
make_th4(th)
display_robot

function slider4_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function slider5_Callback(hObject, eventdata, handles)
th = get(hObject,'Value');
make_th5(th)
display_robot

function slider5_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit1_Callback(hObject, eventdata, handles)
x_target = str2double(get(hObject,'String'));
makeX(x_target)

function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit2_Callback(hObject, eventdata, handles)
y_target = str2double(get(hObject,'String'));
makeY(y_target)

function edit2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit3_Callback(hObject, eventdata, handles)
z_target = str2double(get(hObject,'String'));
makeZ(z_target)

function edit3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
 
function angles = ikine(x,y,z,ori)
a1 = 1;
a2 = 1;
a3 = 3;
a4 = 2; 

r3 = sqrt(x*x + y*y);
r2 = z - a1;
r1 = sqrt(r2*r2 + (r3-a2)*(r3-a2));
d = r2/(r3-a2);
phi2 = atan(d);
phi1 = acos((a4*a4 - a3*a3 - r1*r1)/(-2*a3*r1));
phi3 = acos((r1*r1 - a3*a3 - a4*a4)/(-2*a3*a4));
theta1 = rad2deg(atan(y/x));
theta2 = rad2deg(phi2 - phi1);
theta3 = rad2deg(deg2rad(180) - phi3);

angles.theta1 = theta1;
angles.theta2 = theta2;
angles.theta3 = theta3;

function move(mat,count)
framesPerSecond = 24;

r = rateControl(framesPerSecond);
my_robot = getGlobal;
config = getGlobalc;


for i=1:count
th1 = mat(i,1);
th2 = mat(i,2);
th3 = mat(i,3);
th5 = mat(i,5);

config(1).JointPosition = deg2rad(th1);
config(2).JointPosition = deg2rad(th2);
config(3).JointPosition = deg2rad(th3);
config(4).JointPosition = deg2rad(0);
config(5).JointPosition = deg2rad(th5);


show(my_robot,config);
view([110 40]);
waitfor(r);
end
function console
    fig = uifigure;
    uialert(fig,'Cant Reach the Point\nOut of workspace','ERROR');
function radiobutton1_Callback(hObject, eventdata, handles)
x = getX;
y = getY;
z = getZ;

angles = ikine(x,y,z);

th1 = (angles.theta1);
th2 = (angles.theta2);
th3 = (angles.theta3);
th4 = 0;
th5 = -100;

start = 0;
count = 50;
angle1 = linspace(start,th1,count);
angle2 = linspace(start,th2,count);
angle3 = linspace(start,th3,count);
angle4 = linspace(start,th4,count);
angle5 = linspace(start,th5,count);

    
if isnan(angle1)
    console
elseif isnan(angle2)
    console
elseif isnan(angle3)
    console
elseif isnan(angle4)
    console
elseif isnan(angle5)
    console
else
mat = [angle1;angle2;angle3;angle4;angle5]'
move(mat,count);

end
