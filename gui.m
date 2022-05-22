%auto build function do not change
%code build from the GUI
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

%%The following functions makes the specified variables global,

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


function makeMatrixGlobal(homing_matrix)
    global matrixH
    matrixH = homing_matrix;

function r = getMatrix
    global matrixH
    r = matrixH;

function basic_view
    %Set the axis limit
    xlim([-0.4000 0.2000]) 
    ylim([-0.5000 0.1000])  
    zlim([-0.3 0.3])
    %set the view azimuth angles
    view([110 40])

%default view for the robot vie
function setDefault
    path = 'C:/Users/josep/Desktop/JOSEE 5.1/Project/coding/urdf/urdf/assemblyurdf.SLDASM/urdf/assemblyurdf.SLDASM.urdf';
    my_robot = importrobot(path); %importing the urdf file as my_robot
    config = homeConfiguration(my_robot);%default view for the robot
    makeGlobal(my_robot)
    makeGlobalc(config)
    show(my_robot,config);%Show the robot
    basic_view

function gui_OpeningFcn(hObject, eventdata, handles, varargin)
    setDefault
    % Choose default command line output for gui
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

function varargout = gui_OutputFcn(hObject, eventdata, handles) 
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
    config(2).JointPosition = deg2rad(th2); %Assign theta 2 to first joint
    config(3).JointPosition = deg2rad(th3); %Assign theta 3 to first joint
    config(4).JointPosition = deg2rad(th4); %Assign theta 4 to first joint
    config(5).JointPosition = deg2rad(th5); %Assign theta 5 to first joint

    show(r,config);
    basic_view

function slider1_Callback(hObject, eventdata, handles)
    th = get(hObject,'Value');
    make_th1(th);
    display_robot

function slider1_CreateFcn(hObject, eventdata, handles)
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end

function slider2_Callback(hObject, eventdata, handles)
    th = get(hObject,'Value');
    make_th2(th)
    display_robot

function slider2_CreateFcn(hObject, eventdata, handles)
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end

%what happens when slider 3 value changes
function slider3_Callback(hObject, eventdata, handles)
    th = get(hObject,'Value');
    make_th3(th)
    display_robot

%function to create slider 3
function slider3_CreateFcn(hObject, eventdata, handles)
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end

%what happens when slider 4 value changes
function slider4_Callback(hObject, eventdata, handles)
    th = get(hObject,'Value');
    make_th4(th)
    display_robot

%creating slider 4
function slider4_CreateFcn(hObject, eventdata, handles)
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end

%what happens when slider 5 value changes
function slider5_Callback(hObject, eventdata, handles)
    th = get(hObject,'Value');
    make_th5(th)
    display_robot

%creating slider 5
function slider5_CreateFcn(hObject, eventdata, handles)
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end

%what happens when input is provided in btn1
function edit1_Callback(hObject, eventdata, handles)
    x_target = str2double(get(hObject,'String'));
    makeX(x_target)

%creating btn1
function edit1_CreateFcn(hObject, eventdata, handles)

    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

%what happens when input is provided in btn2
function edit2_Callback(hObject, eventdata, handles)
    y_target = str2double(get(hObject,'String'));
    makeY(y_target)

%creating btn 2
function edit2_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

%what happens when input is provided in btn
function edit3_Callback(hObject, eventdata, handles)
    z_target = str2double(get(hObject,'String'));
    makeZ(z_target)

%creating btn 3
function edit3_CreateFcn(hObject, eventdata, handles)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

%returns homogenous transformation matrix given the variables(DH
%parameters)
function HTM = DHP(th,alph,d,r)
    th = deg2rad(th);
    alph = deg2rad(alph);
    MATX = [
        cos(th) -sin(th)*cos(alph) sin(th)*sin(alph) r*cos(th)
        sin(th) cos(th)*cos(alph) -cos(th)*sin(alph) r*sin(th)
        0 sin(alph) cos(alph) d
        0 0 0 1
    ]; %the homogenous transformation matrix
    HTM.MATX = MATX;

%returns the joint angels given the end effector position (x,y,z)
%ori is the orientation of the end effector with respect to the base frame
function angles = ikine(x,y,z,ori)
    a1 = 1.1903;
    a2 = 0.7120;
    a3 = 1.3455;
    a4 = 0.8184; 
    a5 = 0.2467;
    a6 = 0.7794;

    x = x - a6;
    a4 = a4 + a5;

    H0_5 = ori;

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

    HTM1 = DHP(theta1,90,a1,a2);
    HTM2 = DHP(theta2,0,0,a3);
    HTM3 = DHP(theta3+90,90,0,0);

    H0_1 = HTM1.MATX;
    H1_2 = HTM2.MATX;
    H2_3 = HTM3.MATX;

    H0_3 = H0_1*H1_2*H2_3;

    H3_5 = inv(H0_3)\H0_5;

    theta4 = rad2deg(asin(H3_5(1,3)));
    theta5 = rad2deg(asin(H3_5(3,1)));

    angles.theta1 = theta1;
    angles.theta2 = theta2;
    angles.theta3 = theta3;
    angles.theta4 = theta4;
    angles.theta5 = theta5;

%function to move the robot
%mat is a matrix carring the values for the joint angle variables
%e.g = mat = [angle1 angle2 angle3 angle4 angle5]
function move(mat,count)
    framesPerSecond = 24; %frames per second for the animation

    r = rateControl(framesPerSecond);
    my_robot = getGlobal;
    config = getGlobalc;

    %starting the animation using a for loop
    for i=1:count 
        th1 = mat(i,1);
        th2 = mat(i,2);
        th3 = mat(i,3);
        th4 = mat(i,4);
        th5 = mat(i,5);

        %configure the robot joint with the obtain theta values
        config(1).JointPosition = deg2rad(th1);
        config(2).JointPosition = deg2rad(th2);
        config(3).JointPosition = deg2rad(th3);
        config(4).JointPosition = deg2rad(th4);
        config(5).JointPosition = deg2rad(th5);

        show(my_robot,config,'PreservePlot',true);

        basic_view

        waitfor(r);
    end

%generates the homing matrix, flips the matrix used to take the robot to
%the target position
function new_mat = home_mat(mat)
    angle1 = flip(mat(:,1))'';
    angle2 = flip(mat(:,2))'';
    angle3 = flip(mat(:,3))'';
    angle4 = flip(mat(:,4))'';
    angle5 = flip(mat(:,5))'';

    new_mat = [angle1 angle2 angle3 angle4 angle5];

%what happens when radio button is pressed
function radiobutton1_Callback(hObject, eventdata, handles)
    x = getX;
    y = getY;
    z = getZ;

    %orientationo end effector
    orientation = [
        1 0 0 0
        0 0 -1 0
        0 1 0 0
        0 0 0 1
        ];

    %obtain the joint angles
    angles = ikine(x,y,z,orientation);

    th1 = (angles.theta1);
    th2 = 90 + (angles.theta2);
    th3 = -90 + (angles.theta3);
    th4 = (angles.theta4);
    th5 = -(90 - (angles.theta5));

    start = 0;
    count = 50;
    angle1 = linspace(start,th1,count);
    angle2 = linspace(start,th2,count);
    angle3 = linspace(start,th3,count);
    angle4 = linspace(start,th4,count);
    angle5 = linspace(start,th5,count);

    mat = [angle1;angle2;angle3;angle4;angle5]';
    homing_matrix = home_mat(mat);
    makeMatrixGlobal(homing_matrix);
    move(mat,count);

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over slider1.
function slider1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function disp_message(message)
    myicon = imread("avatar.png");
    msgbox(message,"Success","custom",myicon); 

%what happens when button is pressed
function pushbutton1_Callback(hObject, eventdata, handles)
    matrix = getMatrix;
    if isempty(matrix)
        py.print("Robot Homed")

    else
        count = size(matrix(:,1));
        move(matrix,count)
        clearvars matrix
        msg = 'Robot Homed Successfuly';
        disp_message(msg)    
    end
    % hObject    handle to pushbutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

%test joint angles
function test_joint(joint,n)
    framesPerSecond = 100;

    r = rateControl(framesPerSecond);
    my_robot = getGlobal;
    config = getGlobalc;
    if (n == 3)
        config(5).JointPosition = deg2rad(-90);
    elseif (n == 2)
        config(5).JointPosition = deg2rad(-90);
    end

    for i=1:140
        th1 = joint(1,i);
        config(n).JointPosition = deg2rad(th1);

        show(my_robot,config,'PreservePlot',true);

        basic_view

        waitfor(r);
    end
    
    config(5).JointPosition = deg2rad(0);
    show(my_robot,config,'PreservePlot',true);
    basic_view
    message = 'Joint Test Successful!';
    disp_message(message)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)

    count = 35;

    half_1 = linspace(0,-50,count);
    half_2 = flip(half_1);
    half_3 = linspace(0,50,count);
    half_4 = flip(half_3);

    mat = [half_1 half_2 half_3 half_4];
    global joint_number
    n = joint_number;

    test_joint(mat,n);

% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
    global joint_number
    joint_number = 1;

function radiobutton3_Callback(hObject, eventdata, handles)
    global joint_number
    joint_number = 2;


function radiobutton4_Callback(hObject, eventdata, handles)
    global joint_number
    joint_number = 3;

% --- Executes on button press in radiobutton5.
function radiobutton5_Callback(hObject, eventdata, handles)
    global joint_number
    joint_number = 4;


% --- Executes on button press in radiobutton6.
function radiobutton6_Callback(hObject, eventdata, handles)
    global joint_number
    joint_number = 5;
