%% TRAJECTORY OPTIMIZATION%%
% Copyrights: Theyagarajan SELVAM, Sarvesh RAVICHANDRAN, Rajesh Kannan RAVINDRAN Centrale Nantes

%Trajectory Optimization is done using GUIDE in Matlab


function varargout = Trajectory_optimization(varargin)
%% GUI Functions

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Trajectory_optimization_OpeningFcn, ...
                   'gui_OutputFcn',  @Trajectory_optimization_OutputFcn, ...
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

% --- Executes just before Trajectory_optimization is made visible.
function Trajectory_optimization_OpeningFcn(hObject, eventdata, handles, varargin)
%Setting the Grid
xlim([0 10]);
ylim([0 10]);
grid on;
title('Trajectory Optimization','Fontweight','Bold', 'FontSize',20); 
hold on;
% Choose default command line output for Trajectory_optimization
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = Trajectory_optimization_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in T1button.
function T1button_Callback(hObject, eventdata, handles)
%% PushButton to execute the 1st Task

cla(gca);
Lb = [0 0];   %Lower Bound
Ub = [10 10]; %Upper bound
Gp1 = [8 8];  %Guess Point
A = [1 9];    %Start point
B = [9 1];    %Final point
Cp1 = [5 5];  %Center
Rp1 = 2;      %Radius
N = [randi([0,10],5,2);randi([0,10],5,2)]; %Random points is given using N
but1 = get(handles.popupmenu1,'Value');

%pop up menu is called and executed based on selection
if(but1==1)
    msgbox('Make a Selection in the list');
elseif(but1== 2)
    optimiz(Lb,Ub,Gp1,A,B,Cp1,Rp1)
elseif(but1== 3)
    optimiz(Lb,Ub,randi([1,9],2,2),A,B,Cp1,Rp1)
elseif (but1==4)
    optimiz(Lb,Ub,N,A,B,Cp1,Rp1)
end


% --- Executes on button press in T2button.
function T2button_Callback(hObject, eventdata, handles)
%% PushButton to execute the 2nd Task
cla(gca);
Lb = [0 0];                 %Lower bound
Ub = [10 10];               %Upper bound
A = [1 9];                  %Start point
B = [9 1];                  %Final point
Gp2 = randi([1 8],3,2);     %Guess points given randomly
Cp2 = [3 7; 5 5];           %Center
Rp2 = [1 1];                %Radius
optimiz(Lb,Ub,Gp2,A,B,Cp2,Rp2) 


% --- Executes on button press in T3button.
function T3button_Callback(hObject, eventdata, handles)
%% PushButton to execute the 3nd Task
cla(gca);
%Lower bounds and Upper bounds - Without walls 
Lb1 = [0 0];  
Ub1 = [10 10];
%Lower bounds and Upper bounds - With walls
Lb2 = [0 0 ; 0 0;    4 4;     4 4; 4 4;   6 0; 6 0; 6 0 ];
Ub2 = [4 10 ; 4 10;  6 6;  6 6; 6 6;   10 10; 10 10; 10 10 ];

A = [1 9];  %Start point
B = [9 1];  %Final point

%Without walls
Gp3_1 = [7 8; 2 4; 8 5;9 8; 5 2; 7 1];     %Guess points
Cp3_1 = [3 7; 6 4; 8 3; 2 3; 7 9];         %Center
Rp3_1 = [1 0.75 1 0.5 0.5];                %Radius

%With walls
Gp3_2 = [7 8; 1 3; 8 6; 7 4;3 1; 8 2];     %Guess points
Cp3_2 = [2 7; 5 5; 2 4; 7 3.5; 8 8];       %Center
Rp3_2 = [0.75 0.6 1 0.75 0.75];            %Radius

%Radio button is called and executed based on selection
Rbut = get(handles.panel,'SelectedObject');
selected = get(Rbut,'String');
switch selected
    case 'Without walls'
        optimiz(Lb1,Ub1,Gp3_1,A,B,Cp3_1,Rp3_1)
    case 'With walls'
        fill([4 4 6 6],[0 4 4 0],'b');
        fill([4 4 6 6],[6 10 10 6],'b');
        optimiz(Lb2,Ub2,Gp3_2,A,B,Cp3_2,Rp3_2)
end


function f = objectives(x, A, B)
%% Objective function for the trajectory

X = [A; x; B];  %x is the Guess points
f=0;
    for i = 1:length(X)-1
        f = f + norm(X(i, :) - X(i+1, :));
    end
   

function D = distance_segment_circle(A,B,C,n)
%% Function to discretize the points and find the distance
% A et B two point of the segement
% C center of the circle R the radius
% n number of discretization
    for i=0:n
        M=A+((i/n)*(B-A)); % M is a given point between A and B
        d(i+1)=norm(M-C);
    end
    D = min(d);   
    
   
function [g,h,z] = const(x, A, B, C, R, n, z)
%% Constraint function for the trajectory  
 
    Pts = [A;  x ; B];
    k=0 ;
    z = waitbar(0.5/1,z,'Please Wait...');
    for ci = 1:1:length(C)
        if numel(C)==2
              for si=1:length(Pts)-1
              k=k+1;
              g(k)=R(1)-distance_segment_circle(Pts(si,:),Pts(si+1,:),C(1,:),n);             
              end
              
        else
            for si=1:length(Pts)-1
              k=k+1;
              g(k)=R(ci)-distance_segment_circle(Pts(si,:),Pts(si+1,:),C(ci,:),n);
            end     
        end
    end    
    z = waitbar(0.6/1,z, 'Please Wait...');
h=[];


function [h] = cir(C,R)
%% Function for plotting circles

t= 0:0.001:2*pi;
    if numel(C)~=2
        for k = 1:length(C)
        h=fill(C(k,1)+R(k)*cos(t),C(k,2)+R(k)*sin(t),'g');
        hold on;
        end
    else
        h=fill(C(1,1)+R(1)*cos(t),C(1,2)+R(1)*sin(t),'g');        
        hold on;
    end

% 
function [op1,op2] = optimiz(Lb,Ub,Gp,A,B,Cp,Rp)
%% Optimization function that optimizes the trajectory using fmincon
    Ps = [A ; Gp ; B];
    legend('off');
    x0 = Gp;
    n = 1000;
    
    z = waitbar(0,'Evaluating...', 'Name', 'Trajectory Optimization');
    for i =0.1:0.1:0.4
            waitbar(i/1,z);
            pause(0.1);
    end
    obj = @(x)objectives(x, A, B);
    cons = @(x)const(x,A,B,Cp,Rp,n,z);
    
    %Optimizing the trajectory using fmincon
    options = optimset('Display', 'iter', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxIter', 500, 'MaxFunEvals', 1000);
    x = fmincon(obj, x0, [], [], [], [], Lb, Ub, cons, options);
    
    for i = 0.7:0.1:1
       waitbar(i/1,z,'Almost done..!');
       pause(0.1);
    end
    delete(z);
    Points_Ps= struct(); %contains discritized points in Ps

    n=0:0.01:1;
    %Loop to discretize the points in Ps
    for i = 1 : length(Ps)-1
        for j = 1 : length(n)
            Points_Ps(i).M(j,:) = Ps(i,:)+(Ps(i+1,:)-Ps(i,:))*n(j);
        end
    end
 
    Points_F= struct(); % contains discritized points in F
   
    F = [Ps(1,:) ; x ; Ps(length(Ps),:)];
    n=0:0.01:1;
    %Loop to discretize the points in F
    for i = 1 : length(F)-1
        for j = 1 : length(n)
            Points_F(i).N(j,:) = F(i,:)+(F(i+1,:)-F(i,:))*n(j);
        end
    end
    
    text(0.5,9.3,'Start','FontSize',10);
    text(9,0.7,'Goal','FontSize',10);
    cir(Cp,Rp)
    
    %Selection menu 
    Button = questdlg('Do you want moving trajectory', ...
                     'Optimization', ...
                     'Yes', 'No','No');
    if(strcmp(Button,'Yes')==1)
        
        %Plot the discritized points of F to create a moving line
        for i = 1 : length(Points_F) 
            for j = 1 : length(n)-1
                op2 = plot([Points_F(i).N(j,1),Points_F(i).N(j+1,1)],[Points_F(i).N(j,2),Points_F(i).N(j+1,2)],'LineWidth',1.25,'color','b');
                op1 = plot([Points_Ps(i).M(j,1),Points_Ps(i).M(j+1,1)],[Points_Ps(i).M(j,2),Points_Ps(i).M(j+1,2)],'--','LineWidth',1.25,'color','r');
                scatter(F(:,1),F(:,2),'b');
                scatter(Ps(:,1),Ps(:,2),'r');
                drawnow();                                
            end
        end
       
    else
         
        for i = 1:length(Ps)-1
           op1=plot([Ps(i,1) Ps(i+1,1)],[Ps(i,2) Ps(i+1,2)],'--','Linewidth',1, 'color','r'); 
           scatter(Ps(:,1),Ps(:,2),'r');
        end

        %Plot the points in F
        for i = 1:length(F)-1
           op2 = plot([F(i,1) F(i+1,1)],[F(i,2) F(i+1,2)],'Linewidth',1,'color','b');
           scatter(F(:,1),F(:,2),'b');
        end         
    end     
    legend([op1,op2],'Non-Optimised Path','Optimised Path','Location','northeast');
    msgbox('Task completed');
 
% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)

% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% End of Program 




