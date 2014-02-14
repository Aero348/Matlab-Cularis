function varargout = jsbsimgui_sim(varargin)
% JSBSIMGUI_SIM M-file for jsbsimgui_sim.fig
%      JSBSIMGUI_SIM, by itself, creates a new JSBSIMGUI_SIM or raises the existing
%      singleton*.
%
%      H = JSBSIMGUI_SIM returns the handle to a new JSBSIMGUI_SIM or the handle to
%      the existing singleton*.
%
%      JSBSIMGUI_SIM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in JSBSIMGUI_SIM.M with the given input arguments.
%
%      JSBSIMGUI_SIM('Property','Value',...) creates a new JSBSIMGUI_SIM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before jsbsimgui_sim_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to jsbsimgui_sim_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help jsbsimgui_sim

% Last Modified by GUIDE v2.5 20-Oct-2010 21:22:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @jsbsimgui_sim_OpeningFcn, ...
                   'gui_OutputFcn',  @jsbsimgui_sim_OutputFcn, ...
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


% --- Executes just before jsbsimgui_sim is made visible.
function jsbsimgui_sim_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to jsbsimgui_sim (see VARARGIN)

% Choose default command line output for jsbsimgui_sim
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes jsbsimgui_sim wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = jsbsimgui_sim_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% Simulation Pane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on selection change in listbox3.
function Sim_model_Callback(hObject, eventdata, handles)
% hObject    handle to listbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    index = get(hObject,'Value');
    %contents = get(hObject,'String');
    new_sim_model = handles.sim_model_contents{index};
    set(handles.simulink_model, 'String', new_sim_model);
    handles.NameOfModel = new_sim_model;
    guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns listbox3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox3
% --- Executes during object creation, after setting all properties.
function Sim_model_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.sim_model_contents = get(hObject,'String');
    guidata(hObject,handles);
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in Sim_pos_rot.
function Sim_pos_rot_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_pos_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    index = get(hObject,'Value');
    %contents = get(hObject,'String');
    handles.pos_rot_value = index;
    set(handles.Sim_pos_rot_value, 'String', num2str(handles.pos_rot_value));
    guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns Sim_pos_rot contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Sim_pos_rot


% --- Executes during object creation, after setting all properties.
function Sim_pos_rot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_pos_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.pos_rot_contents = get(hObject,'String');
    guidata(hObject,handles);
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Sim_pos_rot_value_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_pos_rot_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.pos_rot_value = str2double(get(hObject,'String'));
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of Sim_pos_rot_value as text
%        str2double(get(hObject,'String')) returns contents of Sim_pos_rot_value as a double


% --- Executes during object creation, after setting all properties.
function Sim_pos_rot_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_pos_rot_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Sim_pos_rot_value = hObject;
    handles.pos_rot_value = 3;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in Sim_pos_trans.
function Sim_pos_trans_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_pos_trans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     index = get(hObject,'Value');
    %contents = get(hObject,'String');
    handles.pos_trans_value = index;
    set(handles.Sim_pos_trans_value, 'String', num2str(handles.pos_trans_value));
    guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns Sim_pos_trans contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Sim_pos_trans


% --- Executes during object creation, after setting all properties.
function Sim_pos_trans_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_pos_trans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.pos_trans_contents = get(hObject,'String');
    guidata(hObject,handles);
   
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Sim_pos_trans_value_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_pos_trans_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     handles.pos_trans_value = str2double(get(hObject,'String'));
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of Sim_pos_trans_value as text
%        str2double(get(hObject,'String')) returns contents of Sim_pos_trans_value as a double


% --- Executes during object creation, after setting all properties.
function Sim_pos_trans_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_pos_trans_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Sim_pos_trans_value = hObject;
     handles.pos_trans_value = 2;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in Sim_rate_rot.
function Sim_rate_rot_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_rate_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     index = get(hObject,'Value');
    %contents = get(hObject,'String');
    handles.rate_rot_value = index;
    set(handles.Sim_rate_rot_value, 'String', num2str(handles.rate_rot_value));
    guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns Sim_rate_rot contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Sim_rate_rot


% --- Executes during object creation, after setting all properties.
function Sim_rate_rot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_rate_rot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.rate_rot_contents = get(hObject,'value');
    guidata(hObject,handles);
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Sim_rate_rot_value_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_rat_rot_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     handles.rate_rot_value = str2double(get(hObject,'String'));
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of Sim_rat_rot_value as text
%        str2double(get(hObject,'String')) returns contents of Sim_rat_rot_value as a double


% --- Executes during object creation, after setting all properties.
function Sim_rate_rot_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_rat_rot_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Sim_rate_rot_value = hObject;
     handles.rate_rot_value = 3;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in Sim_rate_trans.
function Sim_rate_trans_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_rate_trans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     index = get(hObject,'Value');
    %contents = get(hObject,'String');
    handles.rate_trans_value = index;
    set(handles.Sim_rate_trans_value, 'String', num2str(handles.rate_trans_value));
    guidata(hObject,handles);
% Hints: contents = cellstr(get(hObject,'String')) returns Sim_rate_trans contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Sim_rate_trans


% --- Executes during object creation, after setting all properties.
function Sim_rate_trans_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_rate_trans (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
     handles.rate_trans_contents = get(hObject,'String');
    guidata(hObject,handles);
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Sim_rate_trans_value_Callback(hObject, eventdata, handles)
% hObject    handle to Sim_rate_trans_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.rate_trans_value = str2double(get(hObject,'String'));
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of Sim_rate_trans_value as text
%        str2double(get(hObject,'String')) returns contents of Sim_rate_trans_value as a double


% --- Executes during object creation, after setting all properties.
function Sim_rate_trans_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sim_rate_trans_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Sim_rate_trans_value = hObject;
    handles.rate_trans_value = 2;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Aircraft Pane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in Alpha_Lock.
function Alpha_Lock_Callback(hObject, eventdata, handles)
% hObject    handle to Alpha_Lock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.w_value = (atan(handles.theta_value))*handles.u_value;
    set(handles.IC_w,'Value',handles.w_value);%set slider to new value
    set(handles.IC_w_value, 'String', handles.w_value);


% --- Executes on button press in Param_Reset.
function Param_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to Param_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    % Extra code to toggle new and old values that is not yet functional.
    % More work to do.
    % Load u-fps with trimmed value
%     if(handles.param_reset_value)
%         handles.param_reset_value = 0;
%     elseif (~handles.param_reset_value)
%         handles.param_reset_value = 1;
%     end

    %if(handles.param_reset_value)
    % Load u-fps with trimmed value
    temp = handles.u_value;
    handles.u_value = handles.u_value_old;
    handles.u_value_old = temp;
    set(handles.IC_u,'Value',handles.u_value);%set slider to new value
    set(handles.IC_u_value, 'String', handles.u_value);
    % Load v-fps with trimmed value
    temp = handles.v_value;
    handles.v_value = handles.v_value_old;
    handles.v_value_old = temp;
    set(handles.IC_v,'Value',handles.v_value);%set slider to new value
    set(handles.IC_v_value, 'String', handles.v_value);
    % Load w-fps with trimmed value
    temp = handles.w_value;
    handles.w_value = handles.w_value_old;
    handles.w_value_old = temp;    
    set(handles.IC_w,'Value',handles.w_value);%set slider to new value
    set(handles.IC_w_value, 'String', handles.w_value);
    % Load p with trimmed value
    temp = handles.p_value;
    handles.p_value = handles.p_value_old;
    handles.p_value_old = temp;
    set(handles.IC_p,'Value',handles.p_value);%set slider to new value
    set(handles.IC_p_value, 'String', handles.p_value);
    % Load q with trimmed value
    temp = handles.q_value;
    handles.q_value = handles.q_value_old;
    handles.q_value_old = temp;    
    set(handles.IC_q,'Value',handles.q_value);%set slider to new value
    set(handles.IC_q_value, 'String', handles.q_value);
    % Load r with trimmed value
    temp = handles.r_value;
    handles.r_value = handles.r_value_old;
    handles.r_value_old = temp;    
    set(handles.IC_r,'Value',handles.r_value);%set slider to new value
    set(handles.IC_r_value, 'String', handles.r_value);
    % Load altitude with trimmed value
    temp = handles.h_value;
    handles.h_value = handles.h_value_old;
    handles.h_value_old = temp;    
    set(handles.IC_h,'Value',handles.h_value);%set slider to new value
    set(handles.IC_h_value, 'String', handles.h_value);
    % Load long with trimmed value
    temp = handles.long_value;
    handles.long_value = handles.long_value_old;
    handles.long_value_old = temp;    
    set(handles.IC_long,'Value',handles.long_value);%set slider to new value
    %set(handles.IC_long_value, 'String', handles.long_value);
    % Load lat with trimmed value
    temp = handles.lat_value;
    handles.lat_value = handles.lat_value_old;
    handles.lat_value_old = temp;    
    set(handles.IC_lat,'Value',handles.lat_value);%set slider to new value
    %set(handles.IC_lat_value, 'String', handles.lat_value);
    % Load phi with trimmed value
    temp = handles.phi_value;
    handles.phi_value = handles.phi_value_old;
    handles.phi_value_old = temp;    
    set(handles.IC_phi,'Value',handles.phi_value);%set slider to new value
    set(handles.IC_phi_value, 'String', handles.phi_value);
    % Load theta with trimmed value
    temp = handles.theta_value;
    handles.theta_value = handles.theta_value_old;
    handles.theta_value_old = temp;    
    set(handles.IC_theta,'Value',handles.theta_value);%set slider to new value
    set(handles.IC_theta_value, 'String', handles.theta_value);
    % Load psi with trimmed value
    temp = handles.psi_value;
    handles.psi_value = handles.psi_value_old;
    handles.psi_value_old = temp;    
    set(handles.IC_psi,'Value',handles.psi_value);%set slider to new value
    set(handles.IC_psi_value, 'String', handles.psi_value);
    % Load throttle with trimmed value
    temp = handles.throttle_value;
    handles.throttle_value = handles.throttle_value_old;
    handles.throttle_value_old = temp;
    set(handles.IC_throttle,'Value',handles.throttle_value);%set slider to new value
    set(handles.IC_throttle_value, 'String', handles.throttle_value);
    % Load aileron with trimmed value
    temp = handles.aileron_value;
    handles.aileron_value = handles.aileron_value_old;
    handles.aileron_value_old = temp;    
    set(handles.IC_aileron,'Value',handles.aileron_value);%set slider to new value
    set(handles.IC_aileron_value, 'String', handles.aileron_value);
    % Load elevator with trimmed value
    temp = handles.elevator_value;
    handles.elevator_value = handles.elevator_value_old;
    handles.elevator_value_old = temp;    
    set(handles.IC_elevator,'Value',handles.elevator_value);%set slider to new value
    set(handles.IC_elevator_value, 'String', handles.elevator_value);
    % Load rudder with trimmed value
    temp = handles.rudder_value;
    handles.rudder_value = handles.rudder_value_old;
    handles.rudder_value_old = temp;    
    set(handles.IC_rudder,'Value',handles.rudder_value);%set slider to new value
    set(handles.IC_rudder_value, 'String', handles.rudder_value);
    
%     elseif(~handles.param_reset_value)
%     set(handles.IC_u,'Value',handles.u_value);%set slider to new value
%     set(handles.IC_u_value, 'String', handles.u_value);
%     % Load v-fps with trimmed value
%     set(handles.IC_v,'Value',handles.v_value);%set slider to new value
%     set(handles.IC_v_value, 'String', handles.v_value);
%     % Load w-fps with trimmed value
%     set(handles.IC_w,'Value',handles.w_value);%set slider to new value
%     set(handles.IC_w_value, 'String', handles.w_value);
%     % Load p with trimmed value
%     set(handles.IC_p,'Value',handles.p_value);%set slider to new value
%     set(handles.IC_p_value, 'String', handles.p_value);
%     % Load q with trimmed value
%     set(handles.IC_q,'Value',handles.q_value);%set slider to new value
%     set(handles.IC_q_value, 'String', handles.q_value);
%     % Load r with trimmed value
%     set(handles.IC_r,'Value',handles.r_value);%set slider to new value
%     set(handles.IC_r_value, 'String', handles.r_value);
%     % Load altitude with trimmed value
%     set(handles.IC_h,'Value',handles.h_value);%set slider to new value
%     set(handles.IC_h_value, 'String', handles.h_value);
%     % Load long with trimmed value
%     set(handles.IC_long,'Value',handles.long_value);%set slider to new value
%     %set(handles.IC_long_value, 'String', handles.long_value);
%     % Load lat with trimmed value
%     set(handles.IC_lat,'Value',handles.lat_value);%set slider to new value
%     %set(handles.IC_lat_value, 'String', handles.lat_value);
%     % Load phi with trimmed value
%     set(handles.IC_phi,'Value',handles.phi_value);%set slider to new value
%     set(handles.IC_phi_value, 'String', handles.phi_value);
%     % Load theta with trimmed value
%     set(handles.IC_theta,'Value',handles.theta_value);%set slider to new value
%     set(handles.IC_theta_value, 'String', handles.theta_value);
%     % Load psi with trimmed value
%     set(handles.IC_psi,'Value',handles.psi_value);%set slider to new value
%     set(handles.IC_psi_value, 'String', handles.psi_value);
%     % Load throttle with trimmed value
%     set(handles.IC_throttle,'Value',handles.throttle_value);%set slider to new value
%     set(handles.IC_throttle_value, 'String', handles.throttle_value);
%     % Load aileron with trimmed value
%     set(handles.IC_aileron,'Value',handles.aileron_value);%set slider to new value
%     set(handles.IC_aileron_value, 'String', handles.aileron_value);
%     % Load elevator with trimmed value
%     set(handles.IC_elevator,'Value',handles.elevator_value);%set slider to new value
%     set(handles.IC_elevator_value, 'String', handles.elevator_value);
%     % Load rudder with trimmed value
%     set(handles.IC_rudder,'Value',handles.rudder_value);%set slider to new value
%     set(handles.IC_rudder_value, 'String', handles.rudder_value);
%     end
    guidata(hObject,handles);
        

    
% --- Executes during object creation, after setting all properties.
function Param_Reset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Param_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.param_reset_value = 0;
    guidata(hObject,handles);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% Trim Pane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in radiobutton3.
function Trim_pre_trim_enable_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.pretrim_enable_value = get(hObject,'Value');
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of radiobutton3

% --- Executes during object creation, after setting all properties.
function Trim_pre_trim_enable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.pretrim_enable_value = 1;
    set(hObject,'Value', handles.pretrim_enable_value)
    handles.Trim_pre_trim = hObject;
    guidata(hObject,handles);

% --- Executes on button press in Trim_pre_only.
function Trim_pre_only_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_pre_only (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.pretrim_only_value = get(hObject,'Value');
     if(handles.pretrim_only_value)
        handles.pretrim_enable_value = 1;
       set(handles.Trim_pre_trim_enable,'Value', handles.pretrim_enable_value)
    end
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of Trim_pre_only


% --- Executes during object creation, after setting all properties.
function Trim_pre_only_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trim_pre_only (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called    
    handles.pretrim_only_value = 0;
    set(hObject,'Value', handles.pretrim_only_value)
   
    handles.Trim_pre_only = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in Trim_linearize_enable.
function Trim_linearize_enable_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_linearize_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.linearization_value = get(hObject,'Value');
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of Trim_linearize_enable

% --- Executes during object creation, after setting all properties.
function Trim_linearize_enable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trim_linearize_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.linearization_value = 1;
    set(hObject,'Value', handles.linearization_value)
    handles.Trim_linearize_enable = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in Trim_eigen_enable.
function Trim_eigen_enable_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_eigen_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.eigen_value = get(hObject,'Value');
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of Trim_eigen_enable


% --- Executes during object creation, after setting all properties.
function Trim_eigen_enable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trim_eigen_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
     handles.eigen_value = 1;
    set(hObject,'Value', handles.eigen_value)
    handles.Trim_eigen_enable = hObject;
    guidata(hObject,handles);
    

% --- Executes on button press in trim_quick_linearize.
function trim_quick_linearize_Callback(hObject, eventdata, handles)
% hObject    handle to trim_quick_linearize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global JSBSimTrim;
    JSBSimTrim.SimModel = 'jsbsim_trim_mod';
    load_system('jsbsim_trim_mod');
    JSBSimTrim.ModelPath = strcat(JSBSimTrim.SimModel, '/JSBSim S-Function1');
    JSBSimTrim.SampleTime = handles.delta_t_value;
    JSBSimTrim.FinalTime = 60;
    % % Initial Conditions
    JSBSimTrim.Throttle = handles.throttle_value;
    JSBSimTrim.Aileron = handles.aileron_value;
    JSBSimTrim.Elevator = handles.elevator_value;
    JSBSimTrim.Rudder = handles.rudder_value;
    JSBSimTrim.Mix = handles.mixture_value;
    JSBSimTrim.SetRun = handles.set_run_value;
    JSBSimTrim.Flaps = handles.flaps_value;
    JSBSimTrim.Gear = handles.gear_value;

    % % Initial Condition and State Initialization (State Vector Order) to find
    JSBSimTrim.U = handles.u_value; % 1
    JSBSimTrim.V = handles.v_value; % 2
    JSBSimTrim.W = handles.w_value; % 3
    JSBSimTrim.P = handles.p_value; % 4
    JSBSimTrim.Q = handles.q_value; % 5
    JSBSimTrim.R = handles.r_value; % 6
    JSBSimTrim.Height = handles.h_value; % 7
    JSBSimTrim.Long = handles.long_value; %8
    JSBSimTrim.Lat = handles.lat_value; %9
    JSBSimTrim.Phi = handles.phi_value; %10
    JSBSimTrim.Theta = handles.theta_value; %11
    JSBSimTrim.Psi = handles.psi_value; %12

    % Wind velocities
    JSBSimTrim.Winds = [0 0 0];

    JSBSimTrim.Aircraft = strtok(handles.NameOfAircraft, char(39)); % get rid of the extra ' '
    JSBSimTrim.Verbosity = strtok(handles.NameOfVerbosity, char(39)); % get rid of the extra ' '
    JSBSimTrim.FlightViz = 0;
    JSBSimTrim.NAircraftStates = length(12);
    JSBSimTrim.NSimulinkStates = 12;
    JSBSimTrim.StateIdx = zeros(JSBSimTrim.NAircraftStates,1);

    JSBSimTrim.StateIdx = zeros(JSBSimTrim.NAircraftStates,1);
    JSBSimTrim.StateIdx(1) = 1;
    JSBSimTrim.StateIdx(2) = 2;
    JSBSimTrim.StateIdx(3) = 3;
    JSBSimTrim.StateIdx(4) = 4;
    JSBSimTrim.StateIdx(5) = 5;
    JSBSimTrim.StateIdx(6) = 6;
    JSBSimTrim.StateIdx(7) = 7;
    JSBSimTrim.StateIdx(8) = 8;
    JSBSimTrim.StateIdx(9) = 9;
    JSBSimTrim.StateIdx(10) = 10;
    JSBSimTrim.StateIdx(11) = 11;
    JSBSimTrim.StateIdx(12) = 12;
    guidata(hObject, handles);
    do_linearization(hObject, handles);

% --- Executes during object creation, after setting all properties.
function trim_quick_linearize_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trim_quick_linearize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    trim_quick_linearize_enable = hObject;
    guidata(hObject, handles);

% --- Executes on slider movement.
function Trim_tas_err_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = get(hObject,'Value'); % get the new slider value
    handles.tas_err_value = newval;
    handles.err_presets_value = 0;
    set(handles.Trim_err_presets,'Value',handles.err_presets_value);%pass the new value to the textbox
    guidata(hObject,handles); %save the handles struct updates
    set(handles.Trim_tas_err_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Trim_tas_err_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.tas_err_min = get(hObject,'Min'); %make the slider min, 
    handles.tas_err_max = get(hObject,'Max');%max available to other functions
    handles.Trim_tas_err = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function Trim_tas_err_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
    handles.err_presets_value = 0;
    set(handles.Trim_err_presets,'Value',handles.err_presets_value);%pass the new value to the radiobutton
      if isempty(newval)|| (newval > handles.tas_err_max) || (newval < handles.tas_err_min),
          %retrieve slider value and store in handles
            handles.tas_err_value = get(handles.Trim_tas_err,'Value');
             set(hObject,'String',num2str(handles.tas_err_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.Trim_tas_err,'Value',newval);%set slider to new value
            handles.tas_err_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function Trim_tas_err_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_tas_err_value = hObject; %allow textbox to be referenced by other functions
    handles.tas_err_value = 0.1;
    %set(handles.Trim_tas_err,'Value',handles.tas_err_value);%set slider to new value
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function Trim_alt_err_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = get(hObject,'Value'); % get the new slider value
    handles.alt_err_value = newval;
    handles.err_presets_value = 0;
    set(handles.Trim_err_presets,'Value',handles.err_presets_value);%pass the new value to the radiobutton
    guidata(hObject,handles); %save the handles struct updates
    set(handles.Trim_alt_err_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Trim_alt_err_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.alt_err_min = get(hObject,'Min'); %make the slider min, 
    handles.alt_err_max = get(hObject,'Max');%max available to other functions
    handles.Trim_alt_err = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function Trim_alt_err_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
     handles.err_presets_value = 0;
     set(handles.Trim_err_presets,'Value',handles.err_presets_value);%pass the new value to the radiobutton
      if isempty(newval)|| (newval > handles.alt_err_max) || (newval < handles.alt_err_min),
          %retrieve slider value and store in handles
            handles.alt_err_value = get(handles.Trim_alt_err,'Value');
             set(hObject,'String',num2str(handles.alt_err_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.Trim_alt_err,'Value',newval);%set slider to new value
            handles.alt_err_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function Trim_alt_err_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_alt_err_value = hObject; %allow textbox to be referenced by other functions
    handles.alt_err_value = 0.1;
    %set(handles.Trim_alt_err,'Value',handles.alt_err_value);%set slider to new value
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function Trim_bank_err_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = get(hObject,'Value'); % get the new slider value
    handles.bank_err_value = newval;
    handles.err_presets_value = 0;
    set(handles.Trim_err_presets,'Value',handles.err_presets_value);%pass the new value to the radiobutton
    guidata(hObject,handles); %save the handles struct updates
    set(handles.Trim_bank_err_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Trim_bank_err_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.bank_err_min = get(hObject,'Min'); %make the slider min, 
    handles.bank_err_max = get(hObject,'Max');%max available to other functions
    handles.Trim_bank_err = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function Trim_bank_err_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
    handles.err_presets_value = 0;
    set(handles.Trim_err_presets,'Value',handles.err_presets_value);%pass the new value to the radiobutton
      if isempty(newval)|| (newval > handles.bank_err_max) || (newval < handles.bank_err_min),
          %retrieve slider value and store in handles
            handles.bank_err_value = get(handles.Trim_bank_err,'Value');
             set(hObject,'String',num2str(handles.bank_err_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.Trim_bank_err,'Value',newval);%set slider to new value
            handles.bank_err_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function Trim_bank_err_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_bank_err_value = hObject; %allow textbox to be referenced by other functions
    handles.bank_err_value = 0.1*pi/180;
   % set(handles.Trim_bank_err,'Value',handles.bank_err_value);%set slider to new value
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in Trim_err_presets.
function Trim_err_presets_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_err_presets (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.err_presets_value = get(hObject,'Value');% returns toggle state of Trim_err_presets
    if(handles.err_presets_value)
    handles.tas_err_value = 0.001;
    set(handles.Trim_tas_err,'Value',handles.tas_err_value);%set slider to new value
    set(handles.Trim_tas_err_value, 'String', num2str(handles.tas_err_value));
    handles.alt_err_value = 0.001;
    set(handles.Trim_alt_err,'Value',handles.alt_err_value);%set slider to new value
    set(handles.Trim_alt_err_value, 'String', num2str(handles.alt_err_value));
    handles.bank_err_value = 0.1*pi/180;
    set(handles.Trim_bank_err,'Value',handles.bank_err_value);%set slider to new value
    set(handles.Trim_bank_err_value, 'String', num2str(handles.bank_err_value));
    end
    guidata(hObject,handles);
    
% Hint: get(hObject,'Value') returns toggle state of Trim_err_presets


% --- Executes during object creation, after setting all properties.
function Trim_err_presets_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trim_err_presets (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
     handles.err_presets_value = 0;
%     set(hObject,'Value', handles.err_presets_value)
    handles.Trim_err_presets = hObject;
%     handles.tas_err_value = 0.5;
%     set(handles.Trim_tas_err,'Value',handles.tas_err_value);%set slider to new value
%     set(handles.Trim_tas_err_value, 'String', num2str(handles.tas_err_value));
%     handles.alt_err_value = 0.5;
%     set(handles.Trim_alt_err,'Value',handles.alt_err_value);%set slider to new value
%     set(handles.Trim_alt_err_value, 'String', num2str(handles.alt_err_value));
%     handles.bank_err_value = 0.1*pi/180;
%     set(handles.Trim_bank_err,'Value',handles.bank_err_value);%set slider to new value
%     set(handles.Trim_bank_err_value, 'String', num2str(handles.bank_err_value));
    guidata(hObject,handles);

function Trim_thr_gain_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.thr_gain_value = str2double(get(hObject,'String')) % returns contents of edit5 as a double
    handles.gain_lo_value = 0;
    handles.gain_hi_value = 0;
    set(handles.Trim_lo_gain,'Value',handles.gain_lo_value);%pass the new value to the radiobutton
    set(handles.Trim_hi_gain,'Value',handles.gain_hi_value);%pass the new value to the radiobutton
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function Trim_thr_gain_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.thr_gain_value = -0.002;
    
    handles.Trim_thr_gain_value = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Trim_el_gain_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.el_gain_value = str2double(get(hObject,'String')) % returns contents of edit5 as a double
    handles.gain_lo_value = 0;
    handles.gain_hi_value = 0;
    set(handles.Trim_lo_gain,'Value',handles.gain_lo_value);%pass the new value to the radiobutton
    set(handles.Trim_hi_gain,'Value',handles.gain_hi_value);%pass the new value to the radiobutton
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function Trim_el_gain_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
     handles.Trim_el_gain_value = hObject;
     handles.el_gain_value = 0.001;
     
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Trim_ail_gain_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     handles.ail_gain_value = str2double(get(hObject,'String')) % returns contents of edit5 as a double
     handles.gain_lo_value = 0;
    handles.gain_hi_value = 0;
    set(handles.Trim_lo_gain,'Value',handles.gain_lo_value);%pass the new value to the radiobutton
    set(handles.Trim_hi_gain,'Value',handles.gain_hi_value);%pass the new value to the radiobutton
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function Trim_ail_gain_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_ail_gain_value = hObject;
    handles.ail_gain_value = -0.02;
    
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Trim_rud_gain_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     handles.ail_gain_value = str2double(get(hObject,'String')) % returns contents of edit5 as a double
    handles.gain_lo_value = 0;
    handles.gain_hi_value = 0;
    set(handles.Trim_lo_gain,'Value',handles.gain_lo_value);%pass the new value to the radiobutton
    set(handles.Trim_hi_gain,'Value',handles.gain_hi_value);%pass the new value to the radiobutton
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function Trim_rud_gain_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_ail_gain_value = hObject;
    handles.rud_gain_value = 0.02;
    
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function Trim_lo_gain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_lo_gain = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in Trim_lo_gain.
function Trim_lo_gain_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_lo_gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.gain_lo_value = get(hObject,'Value'); %returns toggle state
    if(handles.gain_lo_value)
        set(handles.Trim_hi_gain,'Value',0);%set radio button to new value
        handles.el_gain_value = 0.001;
        %set(handles.Trim_el_gain,'Value',KEl);%set slider to new value
        set(handles.Trim_el_gain_value, 'String', num2str(handles.el_gain_value));
        handles.ail_gain_value = -0.02;
        %set(handles.Trim_ail_gain,'Value',KAil);%set slider to new value
        set(handles.Trim_ail_gain_value, 'String', num2str(handles.ail_gain_value));
        handles.thr_gain_value = -0.002;
        %set(handles.Trim_thr_gain,'Value',KThr);%set slider to new value
        set(handles.Trim_thr_gain_value, 'String', num2str(handles.thr_gain_value));
        handles.rud_gain_value = 0.02;
        %set(handles.Trim_rud_gain,'Value',KRud);%set slider to new value
        set(handles.Trim_rud_gain_value, 'String', num2str(handles.rud_gain_value));
    end
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of Trim_lo_gain

% --- Executes during object creation, after setting all properties.
function Trim_hi_gain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_hi_gain = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in Trim_hi_gain.
function Trim_hi_gain_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_hi_gain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.gain_hi_value = get(hObject,'Value'); %returns toggle state
    if(handles.gain_hi_value)
        set(handles.Trim_lo_gain,'Value',0);%set radio button to new value
        handles.el_gain_value = 0.0001;
        %set(handles.Trim_el_gain,'Value',KEl);%set slider to new value
        set(handles.Trim_el_gain_value, 'String', num2str(handles.el_gain_value));
        handles.ail_gain_value = -0.01;
        %set(handles.Trim_ail_gain,'Value',KAil);%set slider to new value
        set(handles.Trim_ail_gain_value, 'String', num2str(handles.ail_gain_value));
        handles.thr_gain_value = -0.001;
        %set(handles.Trim_thr_gain,'Value',KThr);%set slider to new value
        set(handles.Trim_thr_gain_value, 'String', num2str(handles.thr_gain_value));
        handles.rud_gain_value = 0.01;
        %set(handles.Trim_rud_gain,'Value',KRud);%set slider to new value
        set(handles.Trim_rud_gain_value, 'String', num2str(handles.rud_gain_value));
    end
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of Trim_hi_gain



% --- Executes on slider movement.
function Trim_run_time_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_run_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = get(hObject,'Value'); % get the new slider value
    handles.run_time_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.Trim_run_time_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of
%        slider

% --- Executes during object creation, after setting all properties.
function Trim_run_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Trim_run_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.run_time_value = 10;
    handles.runtime_min = get(hObject,'Min'); %make the slider min, 
    handles.runtime_max = get(hObject,'Max');%max available to other functions
    handles.Trim_run_time = hObject;%allow slider to be referenced by other functions 
    set(hObject,'Value',handles.run_time_value);%pass the new value to the textbox
    guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function Trim_run_time_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.runtime_max) || (newval < handles.runtime_min),
          %retrieve slider value and store in handles
            handles.Trim_run_time_value = get(handles.Trim_run_time,'Value');
             set(hObject,'String',num2str(handles.run_time_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.Trim_run_time,'Value',newval);%set slider to new value
            handles.run_time_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function Trim_run_time_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_run_time_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function Trim_iteration_Callback(hObject, eventdata, handles)
% hObject    handle to slider14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     newval = get(hObject,'Value'); % get the new slider value
    handles.iteration_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.Trim_iteration_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function Trim_iteration_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.iteration_value = 10;
     handles.iteration_min = get(hObject,'Min'); %make the slider min, 
    handles.iteration_max = get(hObject,'Max');%max available to other functions
    handles.Trim_iteration = hObject;%allow slider to be referenced by other functions 
    set(hObject,'Value',handles.iteration_value);%pass the new value to the textbox
    guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function Trim_iteration_value_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.iteration_max) || (newval < handles.iteration_min),
          %retrieve slider value and store in handles
            handles.Trim_iteration_value = get(handles.Trim_iteration,'Value');
             set(hObject,'String',num2str(handles.iteration_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.Trim_iteration,'Value',newval);%set slider to new value
            handles.iteration_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function Trim_iteration_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.iteration_value = 10;
    set(hObject,'String',num2str(handles.iteration_value));
     handles.Trim_iteration_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function Trim_pre_trim_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.pre_trim_type_1_value = 1;
    handles.pre_trim_type_2_value = 0;
    set(hObject,'Value',handles.pre_trim_type_1_value);
    handles.Trim_pre_trim = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in radiobutton1.
function Trim_pre_trim_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
       handles.pre_trim_type_1_value = get(hObject,'Value');% returns toggle state of pre_trim 
       if(handles.pre_trim_type_1_value)
        handles.pre_trim_type_2_value = 0;
        set(handles.Trim_closed_loop,'Value',0);%set radio button to new value
       end
       guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of pre_trim


% --- Executes on slider movement.
function slider10_Callback(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider12_Callback(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider13_Callback(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end




function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider15_Callback(hObject, eventdata, handles)
% hObject    handle to slider15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider16_Callback(hObject, eventdata, handles)
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider17_Callback(hObject, eventdata, handles)
% hObject    handle to slider17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider18_Callback(hObject, eventdata, handles)
% hObject    handle to slider18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



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


% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4


% --- Executes on button press in radiobutton5.
function radiobutton5_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton5


% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3


% --- Executes during object creation, after setting all properties.
function Trim_closed_loop_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.Trim_closed_loop = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in radiobutton2.
function Trim_closed_loop_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.pre_trim_type_2_value = get(hObject,'Value');% returns toggle state of pre_trim 
    if(handles.pre_trim_type_2_value)
        handles.pre_trim_type_1_value = 0;
        set(handles.Trim_pre_trim,'Value',0);%set radio button to new value
    end
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of radiobutton2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % --- Executes on slider movement.
% function slider6_Callback(hObject, eventdata, handles)
% % hObject    handle to slider6 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hints: get(hObject,'Value') returns position of slider
% %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% 
% 
% % --- Executes during object creation, after setting all properties.
% function slider6_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to slider6 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: slider controls usually have a light gray background.
% if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor',[.9 .9 .9]);
% end
% 
% 
% % --- Executes on slider movement.
% function slider7_Callback(hObject, eventdata, handles)
% % hObject    handle to slider7 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hints: get(hObject,'Value') returns position of slider
% %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% 
% 
% % --- Executes during object creation, after setting all properties.
% function slider7_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to slider7 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: slider controls usually have a light gray background.
% if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor',[.9 .9 .9]);
% end
% 
% 
% % --- Executes on slider movement.
% function slider8_Callback(hObject, eventdata, handles)
% % hObject    handle to slider8 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hints: get(hObject,'Value') returns position of slider
% %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% 
% 
% % --- Executes during object creation, after setting all properties.
% function slider8_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to slider8 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: slider controls usually have a light gray background.
% if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor',[.9 .9 .9]);
% end
% 
% 
% % --- Executes on slider movement.
% function slider9_Callback(hObject, eventdata, handles)
% % hObject    handle to slider9 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hints: get(hObject,'Value') returns position of slider
% %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% 
% 
% % --- Executes during object creation, after setting all properties.
% function slider9_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to slider9 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: slider controls usually have a light gray background.
% if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor',[.9 .9 .9]);
% end

% Takes name of Simulink model that contains the JSBSimSFunction
function simulink_model_Callback(hObject, eventdata, handles)
% hObject    handle to simulink_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.NameOfModel = get(hObject,'String');
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of simulink_model as text
%        str2double(get(hObject,'String')) returns contents of simulink_model as a double


% --- Executes on button press in open_model.
% function open_model_Callback(hObject, eventdata, handles)
% hObject    handle to open_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function simulink_model_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simulink_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.simulink_model = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in load_model.
function load_model_Callback(hObject, eventdata, handles)
% hObject    handle to load_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    open_system(handles.NameOfModel); %open the simulink model
    handles.ModelPath = strcat(handles.NameOfModel, '/JSBSim S-Function');
    
    %******************Get the JSBSim SFunction parameters
    handles.NameOfAircraft = get_param(handles.ModelPath,'ac_name');
    handles.ic_states = get_param(handles.ModelPath,'IC_states');
    handles.ic_controls = get_param(handles.ModelPath,'IC_controls');                   
    handles.ic_delta = get_param(handles.ModelPath,'IC_delta_t');   
    handles.NameOfVerbosity = get_param(handles.ModelPath,'IC_verbosity');
    handles.ic_multiplier = get_param(handles.ModelPath,'IC_sim_multiplier');
    handles.ic_solver = get_param(handles.ModelPath,'IC_solver');
    handles.fv_trim_enable_value = get_param(handles.ModelPath,'IC_fv_trim');
    handles.fv_sim_enable_value = get_param(handles.ModelPath,'IC_fv_sim');
%     handles.ic_int_override = get_param(handles.ModelPath,'IC_int_override');
    %i = 0;
    
%     while true
%         [string_states, handles.ic_states] = strtok(handles.ic_states, '[]');
%         if isempty(string_states),  break;  end
%             disp(sprintf('%s', string_states))          
%     end
        
    for k = 1:14
        [str, handles.ic_states] = strtok(handles.ic_states);
            switch k
                case 2
                    %disp(sprintf('%s', str))
                    handles.u_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_u,'Value',handles.u_value);%set slider to new value
                    set(handles.IC_u_value, 'String', str);
                case 3
                    handles.v_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_v,'Value',handles.v_value);%set slider to new value
                    set(handles.IC_v_value, 'String', str);
                case 4
                    handles.w_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_w,'Value',handles.w_value);%set slider to new value
                    set(handles.IC_w_value, 'String', str);
                case 5
                    handles.p_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_p,'Value',handles.p_value);%set slider to new value
                    set(handles.IC_p_value, 'String', str);
                case 6
                    handles.q_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_q,'Value',handles.q_value);%set slider to new value
                    set(handles.IC_q_value, 'String', str);
                case 7
                    handles.r_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_r,'Value',handles.r_value);%set slider to new value
                    set(handles.IC_r_value, 'String', str);
                case 8
                    handles.h_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_h,'Value',handles.h_value);%set slider to new value
                    set(handles.IC_h_value, 'String', str);
                case 9
                    handles.long_value =  str2double(str);%save new value as IC parameter
                    %set(handles.IC_long,'Value',handles.long_value);%set slider to new value
                    set(handles.IC_long, 'String', str);
                case 10
                    handles.lat_value =  str2double(str);%save new value as IC parameter
                    %set(handles.IC_lat,'Value',handles.lat_value);%set slider to new value
                    set(handles.IC_lat, 'String', str);
                case 11
                    handles.phi_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_phi,'Value',handles.phi_value);%set slider to new value
                    set(handles.IC_phi_value, 'String', str);
                case 12
                    handles.theta_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_theta,'Value',handles.theta_value);%set slider to new value
                    set(handles.IC_theta_value, 'String', str);
                case 13
                    handles.psi_value =  str2double(str);%save new value as IC parameter
                    set(handles.IC_psi,'Value',handles.psi_value);%set slider to new value
                    set(handles.IC_psi_value, 'String', str);
            end
            %i = i + 1;;
    end
    %j = 0;
    
%     while true
%         %strip off the brackets, put states in "controls_string"
%         [controls_string, handles.ic_controls] = strtok(handles.ic_controls, '[]');
%         if isempty(controls_string),  break;  end
%             disp(sprintf('%s', controls_string))
%             handles.ic_controls = controls_string;
%     end    
    
    for l = 1:10
        [str1, handles.ic_controls] = strtok(handles.ic_controls);
            switch l
                case 2
                    handles.throttle_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_throttle,'Value',handles.throttle_value);%set slider to new value
                    set(handles.IC_throttle_value, 'String', str1);
                case 3
                    handles.aileron_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_aileron,'Value',handles.aileron_value);%set slider to new value
                    set(handles.IC_aileron_value, 'String', str1);
                case 4
                    handles.elevator_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_elevator,'Value',handles.elevator_value);%set slider to new value
                    set(handles.IC_elevator_value, 'String', str1);
                case 5
                    handles.rudder_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_rudder,'Value',handles.rudder_value);%set slider to new value
                    set(handles.IC_rudder_value, 'String', str1);
                case 6
                    handles.mixture_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_mixture,'Value',handles.mixture_value);%set slider to new value
                    set(handles.IC_mixture_value, 'String', str1);
                case 7
                    handles.set_run_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_set_run,'Value',handles.set_run_value);%set radio button to new value
                    %set(handles.IC_set_run_value, 'String', str1);
                case 8
                    handles.flaps_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_flaps,'Value',handles.flaps_value);%set slider to new value
                    set(handles.IC_flaps_value, 'String', str1);
                case 9
                    handles.gear_value =  str2double(str1);%save new value as IC parameter
                    set(handles.IC_gear,'Value',handles.gear_value);%set slider to new value
                    set(handles.IC_gear_value, 'String', str1);
                
            end
            %j = j + 1;
    end
    
    for i = 1:6
        [str2, handles.ic_solver] = strtok(handles.ic_solver);
            switch i
                case 2
                    handles.rate_rot_value = str2double(str2);
                    set(handles.Sim_rate_rot_value, 'String', str2);
                case 3
                    handles.rate_trans_value = str2double(str2);
                    set(handles.Sim_rate_trans_value, 'String', str2);
                case 4
                    handles.pos_rot_value = str2double(str2);
                    set(handles.Sim_pos_rot_value, 'String', str2);
                case 5
                    handles.pos_trans_value = str2double(str2);
                    set(handles.Sim_pos_trans_value, 'String', str2);
            end
    end
    set(handles.AC_name,'String',handles.NameOfAircraft);
    set(handles.verbosity_name,'String',handles.NameOfVerbosity);
    set(handles.delta_T,'String',handles.ic_delta);
    handles.delta_t_value = str2double(handles.ic_delta);
    handles.multiplier_value = str2double(handles.ic_multiplier);
    set(handles.IC_multiple,'Value',handles.multiplier_value);%set slider to new value
    set(handles.IC_multiple_value,'String',handles.ic_multiplier);
    set(handles.FV_trim_enable,'Value',str2double(handles.fv_trim_enable_value));
    set(handles.FV_sim_enable,'Value',str2double(handles.fv_sim_enable_value));
%     handles.set_int_override = str2num(handles.ic_int_override);
%     set(handles.IC_int_override_value,'Value',handles.set_int_override);%
%     set radio button to new value
    handles.throttle_value_old = handles.throttle_value;
    handles.aileron_value_old = handles.aileron_value;
    handles.elevator_value_old = handles.elevator_value;
    handles.rudder_value_old = handles.rudder_value;
    handles.u_value_old = handles.u_value;
    handles.v_value_old = handles.v_value;
    handles.w_value_old = handles.w_value;
    handles.p_value_old = handles.p_value;
    handles.q_value_old = handles.q_value;
    handles.r_value_old = handles.r_value;
    handles.h_value_old = handles.h_value;
    handles.long_value_old = handles.long_value;
    handles.lat_value_old = handles.lat_value;
    handles.phi_value_old = handles.phi_value;
    handles.theta_value_old = handles.theta_value;
    handles.psi_value_old = handles.psi_value;
    guidata(hObject,handles);
    
% --- Executes on selection change in AC_files.
function AC_files_Callback(hObject, eventdata, handles)
% hObject    handle to AC_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns AC_files contents as cell array
%        contents{get(hObject,'Value')} returns selected item from AC_files


% --- Executes during object creation, after setting all properties.
function AC_files_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AC_files (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function AC_name_Callback(hObject, eventdata, handles)
% hObject    handle to AC_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.NameOfAircraft = get(hObject,'String');
    
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of AC_name as text
%        str2double(get(hObject,'String')) returns contents of AC_name as a double


% --- Executes during object creation, after setting all properties.
function AC_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AC_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.AC_name = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
    

% --- Executes on selection change in verbosity.
function verbosity_Callback(hObject, eventdata, handles)
% hObject    handle to verbosity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
   
    index = get(hObject,'Value');
    %contents = get(hObject,'String');
    new_verbosity = handles.verbosity_contents{index};
    set(handles.verbosity_name, 'String', new_verbosity);
    handles.NameOfVerbosity = new_verbosity;
    guidata(hObject,handles);
% Hints: contents = get(hObject,'String') returns verbosity contents as cell array
%        contents{get(hObject,'Value')} returns selected item from verbosity


% --- Executes during object creation, after setting all properties.
function verbosity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to verbosity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.verbosity_contents = get(hObject,'String');
    guidata(hObject,handles);
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function verbosity_name_Callback(hObject, eventdata, handles)
% hObject    handle to verbosity_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of verbosity_name as text
%        str2double(get(hObject,'String')) returns contents of verbosity_name as a double
    handles.NameOfVerbosity = get(hObject,'String');
    guidata(hObject,handles);
    

% --- Executes during object creation, after setting all properties.
function verbosity_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to verbosity_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.verbosity_name = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_T_Callback(hObject, eventdata, handles)
% hObject    handle to delta_T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_T as text
%        str2double(get(hObject,'String')) returns contents of delta_T as a double
    handles.delta_t_value = str2double(get(hObject,'String'));
    
    guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function delta_T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.delta_T = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function IC_multiple_Callback(hObject, eventdata, handles)
% hObject    handle to IC_multiple (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
         newval = get(hObject,'Value'); % get the new slider value
         handles.multiplier_value = newval;
        % simulink_delta_t = handles.multiplier_value * handles.delta_t_value;
         % printf('\nsetting multiplier');
        guidata(hObject,handles); %save the handles struct updates
        set(handles.IC_multiple_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_multiple_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_multiple (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.multiplier_min = get(hObject,'Min'); %make the slider min, 
     handles.multiplier_max = get(hObject,'Max');%max available to other functions
     handles.IC_multiple = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function IC_multiple_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_multiple_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.multiplier_max) || (newval < handles.multiplier_min),
          %retrieve slider value and store in handles
            handles.multiplier_value = get(handles.IC_multiple,'Value');
             set(hObject,'String',num2str(handles.multiplier_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_multiple,'Value',newval)%set slider to new value
            handles.multiplier_value =  newval;%save new value as IC parameter 
            guidata(hObject,handles);
      end
% Hints: get(hObject,'String') returns contents of IC_multiple_value as text
%        str2double(get(hObject,'String')) returns contents of IC_multiple_value as a double


% --- Executes during object creation, after setting all properties.
function IC_multiple_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_multiple_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_multiple_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
    
    
%%%%%%%%%%%%%%%%%% THROTTLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% --- Executes on slider movement.
function IC_throttle_Callback(hObject, eventdata, handles)
% hObject    handle to IC_throttle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
          
         newval = get(hObject,'Value'); % get the new slider value
         handles.throttle_value = newval;
        guidata(hObject,handles); %save the handles struct updates
        set(handles.IC_throttle_value,'String',newval);%pass the new value to the textbox
 
% function IC_throttle_Set(handles)
%     handles.ValueOfThrottleSlider = handles.ValueOfThrottleValue;
%     set(handles.IC_throttle,'Value', handles.ValueOfThrottleSlider, 'Value', 1);
%     guidata(hObject,handles);
    
% --- Executes during object creation, after setting all properties.
function IC_throttle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_throttle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.throttle_min = get(hObject,'Min'); %make the slider min, 
     handles.throttle_max = get(hObject,'Max');%max available to other functions
     handles.IC_throttle = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_throttle_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_throttle_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
     newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.throttle_max) || (newval < handles.throttle_min),
          %retrieve slider value and store in handles
            handles.throttle_value = get(handles.IC_throttle,'Value');
             set(hObject,'String',num2str(handles.throttle_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_throttle,'Value',newval)%set slider to new value
            handles.throttle_value =  newval;%save new value as IC parameter 
            guidata(hObject,handles);
      end
% Hints: get(hObject,'String') returns contents of IC_throttle_value as text
%        str2double(get(hObject,'String')) returns contents of
%        IC_throttle_value as a double
        

% --- Executes during object creation, after setting all properties.
function IC_throttle_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_throttle_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_throttle_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% AILERON
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_aileron_Callback(hObject, eventdata, handles)
% hObject    handle to ic_aileron (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.aileron_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_aileron_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_aileron_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ic_aileron (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.aileron_min = get(hObject,'Min'); %make the slider min, 
    handles.aileron_max = get(hObject,'Max');%max available to other functions
    handles.IC_aileron = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_aileron_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_aileron_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.aileron_max) || (newval < handles.aileron_min),
          %retrieve slider value and store in handles
            handles.aileron_value = get(handles.IC_aileron,'Value');
             set(hObject,'String',num2str(handles.aileron_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_aileron,'Value',newval);%set slider to new value
            handles.aileron_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_aileron_value as text
%        str2double(get(hObject,'String')) returns contents of IC_aileron_value as a double


% --- Executes during object creation, after setting all properties.
function IC_aileron_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_aileron_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_aileron_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% ELEVATOR
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_elevator_Callback(hObject, eventdata, handles)
% hObject    handle to IC_elevator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.elevator_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_elevator_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_elevator_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_elevator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.elevator_min = get(hObject,'Min'); %make the slider min, 
    handles.elevator_max = get(hObject,'Max');%max available to other functions
    handles.IC_elevator = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_elevator_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_elevator_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.elevator_max) || (newval < handles.elevator_min),
          %retrieve slider value and store in handles
            handles.elevator_value = get(handles.IC_elevator,'Value');
             set(hObject,'String',num2str(handles.elevator_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_elevator,'Value',newval);%set slider to new value
            handles.elevator_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_elevator_value as text
%        str2double(get(hObject,'String')) returns contents of IC_elevator_value as a double


% --- Executes during object creation, after setting all properties.
function IC_elevator_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_elevator_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_elevator_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% RUDDER
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_rudder_Callback(hObject, eventdata, handles)
% hObject    handle to IC_rudder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.rudder_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_rudder_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_rudder_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_rudder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.rudder_min = get(hObject,'Min'); %make the slider min, 
    handles.rudder_max = get(hObject,'Max');%max available to other functions
    handles.IC_rudder = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_rudder_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_rudder_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.rudder_max) || (newval < handles.rudder_min),
          %retrieve slider value and store in handles
            handles.rudder_value = get(handles.IC_rudder,'Value');
             set(hObject,'String',num2str(handles.rudder_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_rudder,'Value',newval);%set slider to new value
            handles.rudder_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_rudder_value as text
%        str2double(get(hObject,'String')) returns contents of IC_rudder_value as a double


% --- Executes during object creation, after setting all properties.
function IC_rudder_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_rudder_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_rudder_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% MIXTURE
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_mixture_Callback(hObject, eventdata, handles)
% hObject    handle to IC_mixture (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.mixture_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_mixture_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_mixture_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_mixture (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.mixture_min = get(hObject,'Min'); %make the slider min, 
    handles.mixture_max = get(hObject,'Max');%max available to other functions
    handles.IC_mixture = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_mixture_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_mixture_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.mixture_max) || (newval < handles.mixture_min),
          %retrieve slider value and store in handles
            handles.mixture_value = get(handles.IC_mixture,'Value');
             set(hObject,'String',num2str(handles.mixture_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_mixture,'Value',newval);%set slider to new value
            handles.mixture_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_mixture_value as text
%        str2double(get(hObject,'String')) returns contents of IC_mixture_value as a double


% --- Executes during object creation, after setting all properties.
function IC_mixture_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_mixture_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_mixture_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% GEAR
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_gear_Callback(hObject, eventdata, handles)
% hObject    handle to IC_gear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.gear_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_gear_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_gear_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_gear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.gear_min = get(hObject,'Min'); %make the slider min, 
    handles.gear_max = get(hObject,'Max');%max available to other functions
    handles.IC_gear = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_gear_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_gear_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.gear_max) || (newval < handles.gear_min),
          %retrieve slider value and store in handles
            handles.gear_value = get(handles.IC_gear,'Value');
             set(hObject,'String',num2str(handles.gear_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             %guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_gear,'Value',newval);%set slider to new value
            handles.gear_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end

      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_gear_value as text
%        str2double(get(hObject,'String')) returns contents of IC_gear_value as a double


% --- Executes during object creation, after setting all properties.
function IC_gear_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_gear_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_gear_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% FLAPS
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_flaps_Callback(hObject, eventdata, handles)
% hObject    handle to IC_flaps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.flaps_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_flaps_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_flaps_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_flaps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.flaps_min = get(hObject,'Min'); %make the slider min, 
    handles.flaps_max = get(hObject,'Max');%max available to other functions
    handles.IC_flaps = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_flaps_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_flaps_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.flaps_max) || (newval < handles.flaps_min),
          %retrieve slider value and store in handles
            handles.flaps_value = get(handles.IC_flaps,'Value');
             set(hObject,'String',num2str(handles.flaps_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_flaps,'Value',newval);%set slider to new value
            handles.flaps_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_flaps_value as text
%        str2double(get(hObject,'String')) returns contents of IC_flaps_value as a double


% --- Executes during object creation, after setting all properties.
function IC_flaps_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_flaps_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_flaps_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% SET RUNNING
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in IC_set_run.
function IC_set_run_Callback(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.set_run_value = get(hObject,'Value') %returns toggle state of IC_set_run
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of IC_set_run


% --- Executes during object creation, after setting all properties.
function IC_set_run_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_set_run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_set_run = hObject;
    %handles.set_run_value = get(hObject, 'Value');
    guidata(hObject,handles);
%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% U
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_u_Callback(hObject, eventdata, handles)
% hObject    handle to IC_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.u_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_u_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_u_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_u (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.u_min = get(hObject,'Min'); %make the slider min, 
    handles.u_max = get(hObject,'Max');%max available to other functions
    handles.IC_u = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function IC_u_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_u_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.u_max) || (newval < handles.u_min)
          %retrieve slider value and store in handles
            handles.u_value = get(handles.IC_u,'Value');
             set(hObject,'String',num2str(handles.u_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             %guidata(hObject,handles); 
      else %use new value to override slider value
          set(handles.IC_u,'Value',newval)%set slider to new value
            handles.u_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_u_value as text
%        str2double(get(hObject,'String')) returns contents of IC_u_value as a double


% --- Executes during object creation, after setting all properties.
function IC_u_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_u_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_u_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% U
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% V
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_v_Callback(hObject, eventdata, handles)
% hObject    handle to IC_v (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.v_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_v_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_v_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_v (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.v_min = get(hObject,'Min'); %make the slider min, 
    handles.v_max = get(hObject,'Max');%max available to other function
    handles.IC_v = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_v_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_v_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.v_max) || (newval < handles.v_min),
          %retrieve slider value and store in handles
            handles.v_value = get(handles.IC_v,'Value');
             set(hObject,'String',num2str(handles.v_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_v,'Value',newval);%set slider to new value
            handles.v_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_v_value as text
%        str2double(get(hObject,'String')) returns contents of IC_v_value as a double


% --- Executes during object creation, after setting all properties.
function IC_v_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_v_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_v_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% V
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%% W
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_w_Callback(hObject, eventdata, handles)
% hObject    handle to IC_w (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.w_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_w_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_w_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_w (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.w_min = get(hObject,'Min'); %make the slider min, 
    handles.w_max = get(hObject,'Max');%max available to other functions
    handles.IC_w = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function IC_w_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_w_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.w_max) || (newval < handles.w_min),
          %retrieve slider value and store in handles
            handles.w_value = get(handles.IC_w,'Value');
             set(hObject,'String',num2str(handles.w_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             
      else %use new value to override slider value
          set(handles.IC_w,'Value',newval);%set slider to new value
            handles.w_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles); 
% Hints: get(hObject,'String') returns contents of IC_w_value as text
%        str2double(get(hObject,'String')) returns contents of IC_w_value as a double


% --- Executes during object creation, after setting all properties.
function IC_w_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_w_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_w_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% W
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% P
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function IC_p_Callback(hObject, eventdata, handles)
% hObject    handle to IC_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.p_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_p_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_p_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_p (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.p_min = get(hObject,'Min'); %make the slider min, 
    handles.p_max = get(hObject,'Max');%max available to other functions
    handles.IC_p = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_p_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_p_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.p_max) || (newval < handles.p_min),
          %retrieve slider value and store in handles
            handles.p_value = get(handles.IC_p,'Value');
             set(hObject,'String',num2str(handles.p_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
             
      else %use new value to override slider value
          set(handles.IC_p,'Value',newval);%set slider to new value
            handles.p_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
       guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_p_value as text
%        str2double(get(hObject,'String')) returns contents of IC_p_value as a double


% --- Executes during object creation, after setting all properties.
function IC_p_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_p_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_p_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% P
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% Q
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_q_Callback(hObject, eventdata, handles)
% hObject    handle to IC_q (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.q_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_q_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_q_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_q (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.q_min = get(hObject,'Min'); %make the slider min, 
    handles.q_max = get(hObject,'Max');%max available to other functions
    handles.IC_q = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_q_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_q_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.q_max) || (newval < handles.q_min),
          %retrieve slider value and store in handles
            handles.q_value = get(handles.IC_q,'Value');
             set(hObject,'String',num2str(handles.q_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_q,'Value',newval);%set slider to new value
            handles.q_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_q_value as text
%        str2double(get(hObject,'String')) returns contents of IC_q_value as a double


% --- Executes during object creation, after setting all properties.
function IC_q_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_q_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_q_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% Q
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% R
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_r_Callback(hObject, eventdata, handles)
% hObject    handle to IC_r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.r_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_r_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_r_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.r_min = get(hObject,'Min'); %make the slider min, 
    handles.r_max = get(hObject,'Max');%max available to other functions
    handles.IC_r = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function IC_r_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_r_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.r_max) || (newval < handles.r_min),
          %retrieve slider value and store in handles
            handles.r_value = get(handles.IC_r,'Value');
             set(hObject,'String',num2str(handles.r_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_r,'Value',newval);%set slider to new value
            handles.r_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_r_value as text
%        str2double(get(hObject,'String')) returns contents of IC_r_value as a double


% --- Executes during object creation, after setting all properties.
function IC_r_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_r_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_r_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% R
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% H
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on slider movement.
function IC_h_Callback(hObject, eventdata, handles)
% hObject    handle to IC_h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.h_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_h_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_h_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.h_min = get(hObject,'Min'); %make the slider min, 
    handles.h_max = get(hObject,'Max');%max available to other functions
     handles.IC_h = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_h_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_h_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.h_max) || (newval < handles.h_min),
          %retrieve slider value and store in handles
            handles.h_value = get(handles.IC_h,'Value');
             set(hObject,'String',num2str(handles.h_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_h,'Value',newval);%set slider to new value
            handles.h_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_h_value as text
%        str2double(get(hObject,'String')) returns contents of IC_h_value as a double


% --- Executes during object creation, after setting all properties.
function IC_h_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_h_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_h_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% H
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% PHI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_phi_Callback(hObject, eventdata, handles)
% hObject    handle to IC_phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.phi_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_phi_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_phi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.phi_min = get(hObject,'Min'); %make the slider min, 
    handles.phi_max = get(hObject,'Max');%max available to other functions
     handles.IC_phi = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_phi_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_phi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.phi_max) || (newval < handles.phi_min),
          %retrieve slider value and store in handles
            handles.phi_value = get(handles.IC_phi,'Value');
             set(hObject,'String',num2str(handles.phi_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_phi,'Value',newval);%set slider to new value
            handles.phi_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_phi_value as text
%        str2double(get(hObject,'String')) returns contents of IC_phi_value as a double


% --- Executes during object creation, after setting all properties.
function IC_phi_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_phi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_phi_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% PHI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%%%%%%% THETA
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on slider movement.
function IC_theta_Callback(hObject, eventdata, handles)
% hObject    handle to IC_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.theta_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_theta_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.theta_min = get(hObject,'Min'); %make the slider min, 
    handles.theta_max = get(hObject,'Max');%max available to other functions
     handles.IC_theta = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_theta_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_theta_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.theta_max) || (newval < handles.theta_min),
          %retrieve slider value and store in handles
            handles.theta_value = get(handles.IC_theta,'Value');
             set(hObject,'String',num2str(handles.theta_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_theta,'Value',newval);%set slider to new value
            handles.theta_value =  newval;%save new value as IC parameter 
            %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_theta_value as text
%        str2double(get(hObject,'String')) returns contents of IC_theta_value as a double


% --- Executes during object creation, after setting all properties.
function IC_theta_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_theta_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_theta_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% THETA
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%% PSI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on slider movement.
function IC_psi_Callback(hObject, eventdata, handles)
% hObject    handle to IC_psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    newval = get(hObject,'Value'); % get the new slider value
    handles.psi_value = newval;
    guidata(hObject,handles); %save the handles struct updates
    set(handles.IC_psi_value,'String',newval);%pass the new value to the textbox
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function IC_psi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.psi_min = get(hObject,'Min'); %make the slider min, 
    handles.psi_max = get(hObject,'Max');%max available to other functions
     handles.IC_psi = hObject;%allow slider to be referenced by other functions 
     guidata(hObject,handles);
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


function IC_psi_value_Callback(hObject, eventdata, handles)
% hObject    handle to IC_psi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    newval = str2double(get(hObject,'String'));% if new value in textbox, evaluate it
      if isempty(newval)|| (newval > handles.psi_max) || (newval < handles.psi_min),
          %retrieve slider value and store in handles
            handles.psi_value = get(handles.IC_psi,'Value');
             set(hObject,'String',num2str(handles.psi_value));
             %if new value is invalid, override it with slider value
             %and display slider value in textbox instead
              
      else %use new value to override slider value
          set(handles.IC_psi,'Value',newval);%set slider to new value
            handles.psi_value =  newval;%save new value as IC parameter 
           %guidata(hObject,handles);
      end
      guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_psi_value as text
%        str2double(get(hObject,'String')) returns contents of IC_psi_value as a double


% --- Executes during object creation, after setting all properties.
function IC_psi_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_psi_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_psi_value = hObject; %allow textbox to be referenced by other functions
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% PSI
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%LAT
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function IC_lat_Callback(hObject, eventdata, handles)
% hObject    handle to IC_lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.lat_value = str2double(get(hObject,'String')); %returns contents of IC_lat as a double 
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_lat as text
%        str2double(get(hObject,'String')) returns contents of IC_lat as a double


% --- Executes during object creation, after setting all properties.
function IC_lat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_lat = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% LAT
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% LONG
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function IC_long_Callback(hObject, eventdata, handles)
% hObject    handle to IC_long (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.long_value = str2double(get(hObject,'String')) %returns contents of IC_long as a double  
    guidata(hObject,handles);
% Hints: get(hObject,'String') returns contents of IC_long as text
%        str2double(get(hObject,'String')) returns contents of IC_long as a double


% --- Executes during object creation, after setting all properties.
function IC_long_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IC_long (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.IC_long = hObject;
    guidata(hObject,handles);
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%%%%%%%%%%%%%%%%%% LONG
%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% --- Executes on button press in init_model.
% --- Sets the model IC parameters
function init_model_Callback(hObject, eventdata, handles)
% hObject    handle to init_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  global JSBSimTrim simulink_delta_t;  
  if handles.trim_set
      
      do_trim(hObject, handles);
      JSBSimGUI_Start;
    
    % Save the states and control settings previous to the trim in case we
    % want to discard the trim result.
    handles.throttle_value_old = handles.throttle_value;
    handles.aileron_value_old = handles.aileron_value;
    handles.elevator_value_old = handles.elevator_value;
    handles.rudder_value_old = handles.rudder_value;
    handles.u_value_old = handles.u_value;
    handles.v_value_old = handles.v_value;
    handles.w_value_old = handles.w_value;
    handles.p_value_old = handles.p_value;
    handles.q_value_old = handles.q_value;
    handles.r_value_old = handles.r_value;
    handles.h_value_old = handles.h_value;
    handles.long_value_old = handles.long_value;
    handles.lat_value_old = handles.lat_value;
    handles.phi_value_old = handles.phi_value;
    handles.theta_value_old = handles.theta_value;
    handles.psi_value_old = handles.psi_value;
    guidata(hObject,handles);
    
    % Take newly calculated trimmed control inputs and states and set them
    % as the new IC parameters
	handles.throttle_value = JSBSimTrim.Throttle;
    handles.aileron_value = JSBSimTrim.Aileron;
    handles.elevator_value = JSBSimTrim.Elevator;
    handles.rudder_value = JSBSimTrim.Rudder;
    handles.u_value = JSBSimTrim.U;
    handles.v_value = JSBSimTrim.V;
    handles.w_value = JSBSimTrim.W;
    handles.p_value = JSBSimTrim.P;
    handles.q_value = JSBSimTrim.Q;
    handles.r_value = JSBSimTrim.R;
    handles.h_value = JSBSimTrim.Height;
    handles.long_value = JSBSimTrim.Long;
    handles.lat_value = JSBSimTrim.Lat;
    handles.phi_value = JSBSimTrim.Phi;
    handles.theta_value = JSBSimTrim.Theta;
    handles.psi_value = JSBSimTrim.Psi;
    
    % Load u-fps with trimmed value
    set(handles.IC_u,'Value',handles.u_value);%set slider to new value
    set(handles.IC_u_value, 'String', handles.u_value);
    % Load v-fps with trimmed value
    set(handles.IC_v,'Value',handles.v_value);%set slider to new value
    set(handles.IC_v_value, 'String', handles.v_value);
    % Load w-fps with trimmed value
    set(handles.IC_w,'Value',handles.w_value);%set slider to new value
    set(handles.IC_w_value, 'String', handles.w_value);
    % Load p with trimmed value
    set(handles.IC_p,'Value',handles.p_value);%set slider to new value
    set(handles.IC_p_value, 'String', handles.p_value);
    % Load q with trimmed value
    set(handles.IC_q,'Value',handles.q_value);%set slider to new value
    set(handles.IC_q_value, 'String', handles.q_value);
    % Load r with trimmed value
    set(handles.IC_r,'Value',handles.r_value);%set slider to new value
    set(handles.IC_r_value, 'String', handles.r_value);
    % Load altitude with trimmed value
    set(handles.IC_h,'Value',handles.h_value);%set slider to new value
    set(handles.IC_h_value, 'String', handles.h_value);
    % Load long with trimmed value
    set(handles.IC_long,'Value',handles.long_value);%set slider to new value
    %set(handles.IC_long_value, 'String', handles.long_value);
    % Load lat with trimmed value
    set(handles.IC_lat,'Value',handles.lat_value);%set slider to new value
    %set(handles.IC_lat_value, 'String', handles.lat_value);
    % Load phi with trimmed value
    set(handles.IC_phi,'Value',handles.phi_value);%set slider to new value
    set(handles.IC_phi_value, 'String', handles.phi_value);
    % Load theta with trimmed value
    set(handles.IC_theta,'Value',handles.theta_value);%set slider to new value
    set(handles.IC_theta_value, 'String', handles.theta_value);
    % Load psi with trimmed value
    set(handles.IC_psi,'Value',handles.psi_value);%set slider to new value
    set(handles.IC_psi_value, 'String', handles.psi_value);
    % Load throttle with trimmed value
    set(handles.IC_throttle,'Value',handles.throttle_value);%set slider to new value
    set(handles.IC_throttle_value, 'String', handles.throttle_value);
    % Load aileron with trimmed value
    set(handles.IC_aileron,'Value',handles.aileron_value);%set slider to new value
    set(handles.IC_aileron_value, 'String', handles.aileron_value);
    % Load elevator with trimmed value
    set(handles.IC_elevator,'Value',handles.elevator_value);%set slider to new value
    set(handles.IC_elevator_value, 'String', handles.elevator_value);
    % Load rudder with trimmed value
    set(handles.IC_rudder,'Value',handles.rudder_value);%set slider to new value
    set(handles.IC_rudder_value, 'String', handles.rudder_value);
  end
    
    fprintf('\n');
    clearSF;
    import_data('model_data');% load model_data.mat file for initial control settings
    
    simulink_delta_t = handles.multiplier_value * handles.delta_t_value;
    fprintf('\nSimulink Fixed Step Fundamental Sample Size set to: %6.6f', simulink_delta_t);
    
    %parameters must be set as strings with brackets.  Spaces are embedded
    %to aid in parsing the initialization parameters.
    Sp = ' ';
    Sb = '[';
    Se = ']';
    u = num2str(handles.u_value);
    v = num2str(handles.v_value);
    w = num2str(handles.w_value);
    p = num2str(handles.p_value);
    q = num2str(handles.q_value);
    r = num2str(handles.r_value);
    long = num2str(handles.long_value);
    lat = num2str(handles.lat_value);
    h = num2str(handles.h_value);
    phi = num2str(handles.phi_value);
    theta = num2str(handles.theta_value);
    psi = num2str(handles.psi_value);
    thr = num2str(handles.throttle_value); 
    ail = num2str(handles.aileron_value); 
    el = num2str(handles.elevator_value);
    rud = num2str(handles.rudder_value); 
    mix = num2str(handles.mixture_value); 
    run_set = num2str(handles.set_run_value); 
    flap = num2str(handles.flaps_value); 
    gear = num2str(handles.gear_value);
    solver1 = num2str(handles.rate_rot_value);
    solver2 = num2str(handles.rate_trans_value);
    solver3 = num2str(handles.pos_rot_value);
    solver4 = num2str(handles.pos_trans_value);
    % set the control's step output inital values to match the inital
    % controls parameters.
    assignin('base', 'init_throttle', handles.throttle_value)
    assignin('base', 'init_aileron', handles.aileron_value)
    assignin('base', 'init_elevator', handles.elevator_value)
    assignin('base', 'init_rudder', handles.rudder_value)
    assignin('base', 'init_mixture', handles.mixture_value)
    assignin('base', 'init_run_set', handles.set_run_value)
    assignin('base', 'init_flaps', handles.flaps_value)
    assignin('base', 'init_gear', handles.gear_value)
    set_param(handles.ModelPath,'ac_name',...
                            handles.NameOfAircraft);
    set_param(handles.ModelPath,'IC_states',...
                            [Sb Sp u Sp v Sp w Sp p Sp q Sp r Sp h Sp long Sp lat Sp phi Sp theta Sp psi Sp Se]);%                   
    set_param(handles.ModelPath,'IC_controls',...
                            [Sb Sp thr Sp ail Sp el Sp rud Sp mix Sp run_set Sp flap Sp gear Sp Se]);                   
    set_param(handles.ModelPath,'IC_delta_t',...
                             num2str(handles.delta_t_value));
    set_param(handles.ModelPath,'IC_verbosity',...
                            handles.NameOfVerbosity);                    
    set_param(handles.ModelPath,'IC_sim_multiplier', num2str(handles.multiplier_value));
  
    set_param(handles.ModelPath,'IC_solver',...
                            [Sb Sp solver1 Sp solver2 Sp solver3 Sp solver4 Sp Se]);
   set_param(handles.ModelPath,'IC_fv_sim',...
                            num2str(handles.fv_sim_enable_value));
   set_param(handles.ModelPath,'IC_fv_trim',...
                            num2str(handles.fv_trim_enable_value));
%     set_param(handles.ModelPath,'IC_int_override', num2str(handles.set_int_override));

   guidata(hObject,handles);
% --- Executes during object creation, after setting all properties.
function init_model_CreateFcn(hObject, eventdata, handles)
% hObject    handle to init_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
     handles.init_model = hObject;
     guidata(hObject,handles);

% --- Executes on button press in clear_SF.
function clear_SF_Callback(hObject, eventdata, handles)
% hObject    handle to clear_SF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    clearSF;
    %fprintf('\nThis is a test');
    
    

% --- Executes during object creation, after setting all properties.
function text1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in help_button.
function help_button_Callback(hObject, eventdata, handles)
% hObject    handle to help_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    open('JSBSimGUI_Help')

% 
% % --- Executes on button press in IC_int_override_value.
% function IC_int_override_value_Callback(hObject, eventdata, handles)
% % hObject    handle to IC_int_override_value (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
%     handles.set_int_override = get(hObject,'Value'); %returns toggle state of IC_set_run
%     guidata(hObject,handles);
% % Hint: get(hObject,'Value') returns toggle state of IC_int_override_value
% 
% 
% % --- Executes during object creation, after setting all properties.
% function IC_int_override_value_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to IC_int_override_value (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
%     handles.IC_int_override_value = hObject;
%     guidata(hObject,handles);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over init_model.
function init_model_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to init_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function perform_trim_CreateFcn(hObject, eventdata, handles)
% hObject    handle to perform_trim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.perform_trim = hObject;
    handles.trim_set = get(hObject,'Value'); %returns toggle state of IC_set_run
    guidata(hObject,handles);

% --- Executes on button press in perform_trim.
function perform_trim_Callback(hObject, eventdata, handles)
% hObject    handle to perform_trim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.trim_set = get(hObject,'Value'); %returns toggle state of IC_set_run
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of perform_trim

function do_trim(hObject,handles)
% A Matlab script that trims the nonlinear JSBSim aircraft model for a 
% chosen flight condition and extracts the aircraft linear model.
% Uses the JSBSim_Sfunction_Trim MEX file.
% This script is based on the trim and linearization script used in the
% Aerosim Blockset of U-Dynamics by Marius Niculescu
%
% Brian Mills
% May, 2010

fprintf('\nJSBSimTrim: Initializing trim....');
handles.do_trim = hObject;
global JSBSimTrim;
clc;

JSBSimTrim.FlightViz = str2double(handles.fv_trim_enable_value);
DoPreTrim = handles.pretrim_enable_value;
OnlyPreTrim = handles.pretrim_only_value;
PreTrimType2 = handles.pre_trim_type_2_value;
PreTrimType1 = handles.pre_trim_type_1_value;
DoLinearization = handles.linearization_value;
%DoEigen = handles.eigen_value;
UseErrPresets = handles.err_presets_value;
MaxTASErr = handles.tas_err_value;
MaxAltErr = handles.alt_err_value;
MaxBankErr = handles.bank_err_value; 

UseLoGains = handles.gain_lo_value;
UseHiGains = handles.gain_hi_value;
ElGain = handles.el_gain_value;
AilGain = handles.ail_gain_value;
ThrGain = handles.thr_gain_value;
RudGain = handles.rud_gain_value; 

RunTime = handles.run_time_value;
Iterations = handles.iteration_value;

JSBSimTrim.SampleTime = handles.delta_t_value;
JSBSimTrim.FinalTime = 60;
% % Initial Conditions
JSBSimTrim.Throttle = handles.throttle_value;
JSBSimTrim.Aileron = handles.aileron_value;
JSBSimTrim.Elevator = handles.elevator_value;
JSBSimTrim.Rudder = handles.rudder_value;
JSBSimTrim.Mix = handles.mixture_value;
JSBSimTrim.SetRun = handles.set_run_value;
JSBSimTrim.Flaps = handles.flaps_value;
JSBSimTrim.Gear = handles.gear_value;

% % Initial Condition and State Initialization (State Vector Order) to find
JSBSimTrim.U = handles.u_value; % 1
JSBSimTrim.V = handles.v_value; % 2
JSBSimTrim.W = handles.w_value; % 3
JSBSimTrim.P = handles.p_value; % 4
JSBSimTrim.Q = handles.q_value; % 5
JSBSimTrim.R = handles.r_value; % 6
JSBSimTrim.Height = handles.h_value; % 7
JSBSimTrim.Long = handles.long_value; %8
JSBSimTrim.Lat = handles.lat_value; %9
JSBSimTrim.Phi = handles.phi_value; %10
JSBSimTrim.Theta = handles.theta_value; %11
JSBSimTrim.Psi = handles.psi_value; %12

% Wind velocities
JSBSimTrim.Winds = [0 0 0];

JSBSimTrim.Aircraft = strtok(handles.NameOfAircraft, char(39)); % get rid of the extra ' '
JSBSimTrim.Verbosity = strtok(handles.NameOfVerbosity, char(39)); % get rid of the extra ' '
%JSBSimTrim.Verbosity = 'verbose'; % Let's keep this at verbose for now

%if strcmp(handles.NameOfVerbosity,'verbose') || strcmp(handles.NameOfVerbosity, 'debug')
    fprintf('\nJSBSimTrim: The JSBSim aircraft model %s.mdl will be trimmed.', JSBSimTrim.Aircraft);
    fprintf('\nJSBSimTrim: To Airspeed of %f fps', JSBSimTrim.U);
    fprintf('\nJSBSimTrim: To Altitude of %f feet', JSBSimTrim.Height);
    fprintf('\nJSBSimTrim: To bank angle of %f rads', JSBSimTrim.Phi);
    newline = sprintf('\n');
%else
   fprintf('\nJSBSimTrim: Preparing to perform trim.....');
%end
   
TrimInput = [JSBSimTrim.Throttle JSBSimTrim.Aileron JSBSimTrim.Elevator JSBSimTrim.Rudder];

% These are input into the IC Parameters box for the S-Function
JSBSimTrim.ICInputs = [TrimInput(1), TrimInput(2), TrimInput(3), TrimInput(4), ...
    JSBSimTrim.Mix, JSBSimTrim.SetRun, JSBSimTrim.Flaps, JSBSimTrim.Gear];

JSBSimTrim.VelocitiesIni = [JSBSimTrim.U JSBSimTrim.V JSBSimTrim.W]';
JSBSimTrim.RatesIni = [JSBSimTrim.P JSBSimTrim.Q JSBSimTrim.R]';
JSBSimTrim.AttitudeIni = [JSBSimTrim.Phi JSBSimTrim.Theta JSBSimTrim.Psi]';
JSBSimTrim.PositionIni = [JSBSimTrim.Height 45*pi/180 -122*pi/180]';

JSBSimTrim.ICStates = [JSBSimTrim.VelocitiesIni(1) JSBSimTrim.VelocitiesIni(2) JSBSimTrim.VelocitiesIni(3)...
                        JSBSimTrim.RatesIni(1) JSBSimTrim.RatesIni(2) JSBSimTrim.RatesIni(3) JSBSimTrim.PositionIni(1)...
                        JSBSimTrim.PositionIni(2) JSBSimTrim.PositionIni(3) JSBSimTrim.AttitudeIni(1) JSBSimTrim.AttitudeIni(2)...
                        JSBSimTrim.AttitudeIni(3)];

JSBSimTrim.SimModel = 'jsbsim_trim_mod';
load_system('jsbsim_trim_mod');
JSBSimTrim.ModelPath = strcat(JSBSimTrim.SimModel, '/JSBSim S-Function1');
% Get the sim options structure
JSBSimTrim.SimOptions = simget(JSBSimTrim.SimModel);
JSBSimTrim.NAircraftStates = length(12);
JSBSimTrim.NSimulinkStates = 12;
JSBSimTrim.StateIdx = zeros(JSBSimTrim.NAircraftStates,1);

JSBSimTrim.StateIdx = zeros(JSBSimTrim.NAircraftStates,1);
JSBSimTrim.StateIdx(1) = 1;
JSBSimTrim.StateIdx(2) = 2;
JSBSimTrim.StateIdx(3) = 3;
JSBSimTrim.StateIdx(4) = 4;
JSBSimTrim.StateIdx(5) = 5;
JSBSimTrim.StateIdx(6) = 6;
JSBSimTrim.StateIdx(7) = 7;
JSBSimTrim.StateIdx(8) = 8;
JSBSimTrim.StateIdx(9) = 9;
JSBSimTrim.StateIdx(10) = 10;
JSBSimTrim.StateIdx(11) = 11;
JSBSimTrim.StateIdx(12) = 12;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following section performs a "pre-trim" routine using a simple
% proportional feedback algorithm.  This section will require some
% parameters to be modified based on the type of aircraft being trimmed and
% the trim condition.  This algorithm was part of the original Aerosim trim
% function and is something that needs to be addressed later.  At some
% point a GUI that lets the user to easily set some of these parameters may
% be desirable. - Brian Mills
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if((DoPreTrim)&&(PreTrimType1)) 
    
% if(UseErrPresets)% Use the error presets
%     MaxErrTAS = 0.25;
%     set(handles.Trim_tas_err,'Value',MaxErrTAS);%set slider to new value
%     set(handles.Trim_tas_err_value, 'String', num2str(MaxErrTAS));
%     MaxErrAlt = 0.25;
%     set(handles.Trim_alt_err,'Value',MaxErrAlt);%set slider to new value
%     set(handles.Trim_alt_err_value, 'String', num2str(MaxErrAlt));
%     MaxErrBank = 0.1*pi/180;
%     set(handles.Trim_bank_err,'Value',MaxErrBank);%set slider to new value
%     set(handles.Trim_bank_err_value, 'String', num2str(MaxErrBank));
% else % GUI defined error thresholds
    MaxErrTAS = MaxTASErr;
    MaxErrAlt = MaxAltErr;
    MaxErrBank = MaxBankErr;
% end

% if(UseLoGains)% Use the low speed gain presets
%     KElevator = 0.001;
% %     set(handles.Trim_el_gain,'Value',KElevator);%set slider to new value
%     set(handles.Trim_el_gain_value, 'String', num2str(KElevator));
%     KAileron = -0.02;
% %     set(handles.Trim_ail_gain,'Value',KAileron);%set slider to new value
%     set(handles.Trim_ail_gain_value, 'String', num2str(KAileron));
%     KThrottle = -0.002;
% %     set(handles.Trim_thr_gain,'Value',KThrottle);%set slider to new value
%     set(handles.Trim_thr_gain_value, 'String', num2str(KThrottle));
%     KRudder = 0.02;
% %     set(handles.Trim_rud_gain,'Value',KRudder);%set slider to new value
%     set(handles.Trim_rud_gain_value, 'String', num2str(KRudder));
% elseif(UseHiGains)% Use the hi speed gain presets
%     KElevator = 0.001;
% %     set(handles.Trim_el_gain,'Value',KElevator);%set slider to new value
%     set(handles.Trim_el_gain_value, 'String', num2str(KElevator));
%     KAileron = -0.01;
% %     set(handles.Trim_ail_gain,'Value',KAileron);%set slider to new value
%     set(handles.Trim_ail_gain_value, 'String', num2str(KAileron));
%     KThrottle = -0.001;
% %     set(handles.Trim_thr_gain,'Value',KThrottle);%set slider to new value
%     set(handles.Trim_thr_gain_value, 'String', num2str(KThrottle));
%     KRudder = 0.01;
% %     set(handles.Trim_rud_gain,'Value',KRudder);%set slider to new value
%     set(handles.Trim_rud_gain_value, 'String', num2str(KRudder));
% else% GUI defined gains
   KElevator = ElGain;
    KAileron = AilGain;
    KThrottle = ThrGain;
    KRudder = RudGain; 
% end    
  

fprintf('\nJSBSimTrim: Computing the initial estimates for the trim inputs...');
fprintf('\n');
fprintf('\n');
GoodGuess = 0; Niter = 0; 

%need to add some abort code in the while loop
while (~GoodGuess)&(Niter<Iterations)% May need to be changed by user! This determines number of times that the pretrim loop will be performed
    % Run Simulink model for a short time (15 s)
   [SimTime, SimStates, SimOutputs] = sim(JSBSimTrim.SimModel, [0 RunTime], JSBSimTrim.SimOptions, ...
       [0 TrimInput; RunTime TrimInput]);
    % Compute errors in trim
    ErrTAS = SimStates(end,1) - JSBSimTrim.U;
    ErrAlt = SimStates(end,7) - JSBSimTrim.Height;
    ErrBank = SimStates(end,10) - JSBSimTrim.Phi;
    ErrYaw = SimStates(end,12) - JSBSimTrim.Psi;
    fprintf('\nJSBSimTrim: Iteration #%2d, Airsp err = %6.6f fps, Alt err = %8.6f ft, phi err = %6.4f deg, psi err = %6.4f deg.', Niter, ErrTAS, ErrAlt, ErrBank*180/pi, ErrYaw*180/pi);  
    % If all errors are within threshold    
    if (abs(ErrTAS)<MaxErrTAS)&(abs(ErrAlt)<MaxErrAlt)&(abs(ErrBank)<MaxErrBank)
        % We are done with the initial guess
        GoodGuess = 1;
    else
        % Adjust aircraft controls and keep them within normalized limits
        TrimInput(4) = TrimInput(4) + KRudder * ErrYaw;
        if (TrimInput(4)>1)
            TrimInput(4) = 1;
        elseif (TrimInput(1)<-1)
            TrimInput(4) = -1;
        end
         %TrimInput(3) = TrimInput(3) + -KElevator * ErrTAS;
         TrimInput(3) = TrimInput(3) + KElevator * ErrAlt;
        if (TrimInput(3)>1)
            TrimInput(3) = 1;
        elseif (TrimInput(3)<-1)
            TrimInput(3) = -1;
        end
        TrimInput(2) = TrimInput(2) + KAileron * ErrBank;
        if (TrimInput(2)>1)
            TrimInput(2) = 1;
        elseif (TrimInput(2)<-1)
            TrimInput(2) = -1;
        end
        TrimInput(1) = TrimInput(1) + KThrottle * ErrTAS;
       %TrimInput(1) = TrimInput(1) + KThrottle * ErrAlt;
        if (TrimInput(1)>1)
            TrimInput(1) = 1;
        elseif (TrimInput(1)<0)
            TrimInput(1) = 0;
        end
        
    end 
    JSBSimTrim.ICInputs = [TrimInput(1), TrimInput(2), TrimInput(3), TrimInput(4), ...
    JSBSimTrim.Mix, JSBSimTrim.SetRun, JSBSimTrim.Flaps, JSBSimTrim.Gear];
    Niter = Niter + 1;
    fprintf('\nJSBSimTrim:      Pre-Trim Control Results:');
    fprintf('\nJSBSimTrim:      Throttle = %6.6f', TrimInput(1));
    fprintf('\nJSBSimTrim:      Aileron = %6.6f', TrimInput(2));
    fprintf('\nJSBSimTrim:      Elevator = %6.6f', TrimInput(3));
    fprintf('\nJSBSimTrim:      Rudder = %6.6f', TrimInput(4));
end
% Save initial guess
fprintf('\nJSBSimTrim: Saving initial velocities, attitude and inputs');
if(Iterations > 0)
JSBSimTrim.VelocitiesIni = SimStates(end,JSBSimTrim.StateIdx(1:3))'; % Save the velocities from the pre-trim run
%JSBSimTrim.RatesIni = SimStates(end,JSBSimTrim.StateIdx(4:6))';
% JSBSimTrim.Height = SimStates(end,JSBSimTrim.StateIdx(7))';
JSBSimTrim.AttitudeIni = SimStates(end, JSBSimTrim.StateIdx(10:12))';% Save the attitude from the pre-trim run
% Save the control inputs
JSBSimTrim.Throttle = TrimInput(1);
JSBSimTrim.Aileron = TrimInput(2);
JSBSimTrim.Elevator = TrimInput(3);
JSBSimTrim.Rudder = TrimInput(4);
clear SimTime SimStates SimOutputs TrimInput;
end
% Print our initial control input guesses
fprintf('\nJSBSimTrim: Initial guesses for trim inputs are:');
fprintf('\nJSBSimTrim:      Throttle = %6.6f', JSBSimTrim.Throttle);
fprintf('\nJSBSimTrim:      Aileron = %6.6f', JSBSimTrim.Aileron);
fprintf('\nJSBSimTrim:      Elevator = %6.6f', JSBSimTrim.Elevator);
fprintf('\nJSBSimTrim:      Rudder = %6.6f', JSBSimTrim.Rudder);

fprintf('\nJSBSimTrim: Initial guesses for trimmed states are:');
fprintf('\nJSBSimTrim:    u = %6.6f ft/s', JSBSimTrim.VelocitiesIni(1));
fprintf('\nJSBSimTrim:    v = %6.6f ft/s', JSBSimTrim.VelocitiesIni(2));
fprintf('\nJSBSimTrim:    w = %6.6f ft/s', JSBSimTrim.VelocitiesIni(3));
fprintf('\nJSBSimTrim:    p = %6.6f deg/s', JSBSimTrim.RatesIni(1)*180/pi);
fprintf('\nJSBSimTrim:    q = %6.6f deg/s', JSBSimTrim.RatesIni(2)*180/pi);
fprintf('\nJSBSimTrim:    r = %6.6f deg/s', JSBSimTrim.RatesIni(3)*180/pi);
fprintf('\nJSBSimTrim:    alt = %6.6f ft', JSBSimTrim.Height);
fprintf('\nJSBSimTrim:    phi = %6.6f deg, %6.6d rads', JSBSimTrim.AttitudeIni(1)*180/pi, JSBSimTrim.AttitudeIni(1));
fprintf('\nJSBSimTrim:    theta = %6.6f deg, %6.6d rads', JSBSimTrim.AttitudeIni(2)*180/pi, JSBSimTrim.AttitudeIni(2));
fprintf('\nJSBSimTrim:    psi = %6.6f deg, %6.6d rads', JSBSimTrim.AttitudeIni(3)*180/pi, JSBSimTrim.AttitudeIni(3));

else
    GoodGuess = 1;
end  

if(~OnlyPreTrim)
    
%% %%% PERFORM AIRCRAFT TRIM %%%
fprintf('\n');
fprintf('\nJSBSimTrim: Performing the aircraft trim...\n');
% Set initial states to mirror the ICStates
StateIni = zeros(JSBSimTrim.NSimulinkStates,1);
StateIni(JSBSimTrim.StateIdx(1)) = JSBSimTrim.VelocitiesIni(1);
StateIni(JSBSimTrim.StateIdx(2)) = JSBSimTrim.VelocitiesIni(2);
StateIni(JSBSimTrim.StateIdx(3)) = JSBSimTrim.VelocitiesIni(3);
StateIni(JSBSimTrim.StateIdx(4)) = JSBSimTrim.RatesIni(1);
StateIni(JSBSimTrim.StateIdx(5)) = JSBSimTrim.RatesIni(2);
StateIni(JSBSimTrim.StateIdx(6)) = JSBSimTrim.RatesIni(3);
StateIni(JSBSimTrim.StateIdx(7)) = JSBSimTrim.Height;
StateIni(JSBSimTrim.StateIdx(8)) = JSBSimTrim.PositionIni(2);
StateIni(JSBSimTrim.StateIdx(9)) = JSBSimTrim.PositionIni(3);
StateIni(JSBSimTrim.StateIdx(10)) = JSBSimTrim.Phi;
StateIni(JSBSimTrim.StateIdx(11)) = JSBSimTrim.AttitudeIni(2);
StateIni(JSBSimTrim.StateIdx(12)) = JSBSimTrim.AttitudeIni(3);

% ICStates parameter must match StateIni
JSBSimTrim.ICStates = [JSBSimTrim.VelocitiesIni(1) JSBSimTrim.VelocitiesIni(2) JSBSimTrim.VelocitiesIni(3)...
                      JSBSimTrim.RatesIni(1) JSBSimTrim.RatesIni(2) JSBSimTrim.RatesIni(3) JSBSimTrim.Height...
                      JSBSimTrim.PositionIni(2) JSBSimTrim.PositionIni(3) JSBSimTrim.Phi JSBSimTrim.AttitudeIni(2)...
                      JSBSimTrim.AttitudeIni(3)];

InputIni = [JSBSimTrim.Throttle; JSBSimTrim.Aileron; JSBSimTrim.Elevator; JSBSimTrim.Rudder]

%ICInputs 1-4 must match InputIni  
JSBSimTrim.ICInputs = [InputIni(1), InputIni(2), InputIni(3), InputIni(4), ...
                      JSBSimTrim.Mix, JSBSimTrim.SetRun, JSBSimTrim.Flaps, JSBSimTrim.Gear];

OutputIni = [JSBSimTrim.U; 0; 0; JSBSimTrim.Height; JSBSimTrim.Phi; 0; 0];
DerivIni = zeros(JSBSimTrim.NSimulinkStates,1);
% 
% % Set indices of fixed parameters
StateFixIdx = [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(10)];

InputFixIdx = []; % Allow all of the primary control inputs to vary
OutputFixIdx = [1 3 4 5]; % fix U, beta, altitude, phi
DerivFixIdx = [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(6) ...
              JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(11) JSBSimTrim.StateIdx(12)]
 
 % Set optimization parameters
 JSBSimTrim.Options(1)  = 1;     % show some output
 JSBSimTrim.Options(2)  = 1e-6;  % tolerance in X
 JSBSimTrim.Options(3)  = 1e-6;  % tolerance in F
 JSBSimTrim.Options(4)  = 1e-6;
 if(~GoodGuess)
    JSBSimTrim.Options(14) = 100;  % max iterations
 else
    JSBSimTrim.Options(14) = 100;  % max iterations
 end
 
 fprintf('\n');
 fprintf('\n');
 % Trim the aircraft model
[TrimOutput.States,TrimOutput.Inputs,TrimOutput.Outputs,TrimOutput.Derivatives] = trim(JSBSimTrim.SimModel,StateIni,InputIni,OutputIni,...
      StateFixIdx,InputFixIdx,OutputFixIdx,DerivIni,DerivFixIdx,JSBSimTrim.Options);

% Print the trim results
%if strcmp(handles.NameOfVerbosity,'verbose') || strcmp(handles.NameOfVerbosity, 'debug')
    fprintf('\nJSBSimTrim: Trim Finished. The trim results are:');
    fprintf('\nJSBSimTrim: INPUTS:');
    fprintf('\nJSBSimTrim:   Throttle = %6.6f', TrimOutput.Inputs(1));
    fprintf('\nJSBSimTrim:   Elevator = %6.6f', TrimOutput.Inputs(3));
    fprintf('\nJSBSimTrim:   Aileron = %6.6f', TrimOutput.Inputs(2));
    fprintf('\nJSBSimTrim:   Rudder = %6.6f', TrimOutput.Inputs(4));
    
    fprintf('\nJSBSimTrim: STATES:');
    fprintf('\nJSBSimTrim:   u = %6.6f ft/s', TrimOutput.States(JSBSimTrim.StateIdx(1)));
    fprintf('\nJSBSimTrim:   v = %6.6f ft/s', TrimOutput.States(JSBSimTrim.StateIdx(2)));
    fprintf('\nJSBSimTrim:   w = %6.6f ft/s', TrimOutput.States(JSBSimTrim.StateIdx(3)));
    fprintf('\nJSBSimTrim:   p = %6.6f deg/s', TrimOutput.States(JSBSimTrim.StateIdx(4))*180/pi);
    fprintf('\nJSBSimTrim:   q = %6.6f deg/s', TrimOutput.States(JSBSimTrim.StateIdx(5))*180/pi);
    fprintf('\nJSBSimTrim:   r = %6.6f deg/s', TrimOutput.States(JSBSimTrim.StateIdx(6))*180/pi);
    fprintf('\nJSBSimTrim:   alt = %6.6f ft', TrimOutput.States(JSBSimTrim.StateIdx(7)));
    fprintf('\nJSBSimTrim:    phi = %6.6f deg, %6.6f rads', TrimOutput.States(JSBSimTrim.StateIdx(10))*180/pi, TrimOutput.States(JSBSimTrim.StateIdx(10)));
    fprintf('\nJSBSimTrim:    theta = %6.6f deg, %6.6f rads', TrimOutput.States(JSBSimTrim.StateIdx(11))*180/pi, TrimOutput.States(JSBSimTrim.StateIdx(11)));
    fprintf('\nJSBSimTrim:    psi = %6.6f deg, %6.6f rads', TrimOutput.States(JSBSimTrim.StateIdx(12))*180/pi, TrimOutput.States(JSBSimTrim.StateIdx(12)));
    fprintf('\nJSBSimTrim: OUTPUTS:');
    fprintf('\nJSBSimTrim:   Airspeed = %6.6f fps', TrimOutput.Outputs(1));
    fprintf('\nJSBSimTrim:   AoA = %6.6f deg', TrimOutput.Outputs(2)*180/pi);
    fprintf('\nJSBSimTrim:   Sideslip = %6.6f deg', TrimOutput.Outputs(3)*180/pi);
    fprintf('\nJSBSimTrim:   Altitude = %6.6f ft', TrimOutput.Outputs(4));
    fprintf('\nJSBSimTrim:   Bank = %6.6f deg', TrimOutput.Outputs(5)*180/pi);
    fprintf('\nJSBSimTrim:   Pitch = %6.6f deg', TrimOutput.Outputs(6)*180/pi);
    fprintf('\nJSBSimTrim:   Heading = %6.6f deg', TrimOutput.Outputs(7)*180/pi);
%else
    fprintf('\nJSBSimTrim: Trim Finished.');
%end

%if strcmp(handles.NameOfVerbosity,'verbose') || strcmp(handles.NameOfVerbosity, 'debug')


%% LINEARIZATION
% Use the inputs and states found in the trim function and extract the linear model
if((~GoodGuess)||(~DoLinearization))
    fprintf('\n\n');
    fprintf('\nJSBSimTrim: Linearization aborted- trimmed states could not be found');
    clearSF;
else
        % Take newly calculated trimmed control inputs and states and set them
    % as the new IC parameters
	JSBSimTrim.Throttle = TrimOutput.Inputs(1);
    JSBSimTrim.Aileron = TrimOutput.Inputs(2);
    JSBSimTrim.Elevator = TrimOutput.Inputs(3);
    JSBSimTrim.Rudder = TrimOutput.Inputs(4);
    JSBSimTrim.U = TrimOutput.States(JSBSimTrim.StateIdx(1));
    JSBSimTrim.V = TrimOutput.States(JSBSimTrim.StateIdx(2));
    JSBSimTrim.W = TrimOutput.States(JSBSimTrim.StateIdx(3));
    JSBSimTrim.P = TrimOutput.States(JSBSimTrim.StateIdx(4));
    JSBSimTrim.Q = TrimOutput.States(JSBSimTrim.StateIdx(5));
    JSBSimTrim.R = TrimOutput.States(JSBSimTrim.StateIdx(6));
    JSBSimTrim.Height = TrimOutput.States(JSBSimTrim.StateIdx(7));
    JSBSimTrim.Long = TrimOutput.States(JSBSimTrim.StateIdx(8));
    JSBSimTrim.Lat = TrimOutput.States(JSBSimTrim.StateIdx(9));
    JSBSimTrim.Phi = TrimOutput.States(JSBSimTrim.StateIdx(10));
    JSBSimTrim.Theta = TrimOutput.States(JSBSimTrim.StateIdx(11));
    JSBSimTrim.Psi = TrimOutput.States(JSBSimTrim.StateIdx(12));
    
    guidata(hObject,handles);
    do_linearization(hObject, handles)
    

end

%     % Take newly calculated trimmed control inputs and states and set them
%     % as the new IC parameters
% 	handles.throttle_value = TrimOutput.Inputs(1);
%     handles.aileron_value = TrimOutput.Inputs(2);
%     handles.elevator_value = TrimOutput.Inputs(3);
%     handles.rudder_value = TrimOutput.Inputs(4);
%     handles.u_value = TrimOutput.States(JSBSimTrim.StateIdx(1));
%     handles.v_value = TrimOutput.States(JSBSimTrim.StateIdx(2));
%     handles.w_value = TrimOutput.States(JSBSimTrim.StateIdx(3));
%     handles.p_value = TrimOutput.States(JSBSimTrim.StateIdx(4));
%     handles.q_value = TrimOutput.States(JSBSimTrim.StateIdx(5));
%     handles.r_value = TrimOutput.States(JSBSimTrim.StateIdx(6));
%     handles.h_value = TrimOutput.States(JSBSimTrim.StateIdx(7));
%     handles.long_value = TrimOutput.States(JSBSimTrim.StateIdx(8));
%     handles.lat_value = TrimOutput.States(JSBSimTrim.StateIdx(9));
%     handles.phi_value = TrimOutput.States(JSBSimTrim.StateIdx(10));
%     handles.theta_value = TrimOutput.States(JSBSimTrim.StateIdx(11));
%     handles.psi_value = TrimOutput.States(JSBSimTrim.StateIdx(12))
    

    
else
    fprintf('\n');
    clearSF;

    
    % Take newly calculated trimmed control inputs and states and set them
    % as the new IC parameters
%  	JSBSimTrim.Throttle = TrimOutput.Inputs(1);
%      JSBSimTrim.Aileron = TrimOutput.Inputs(2);
%      JSBSimTrim.Elevator = TrimOutput.Inputs(3);
%      JSBSimTrim.Rudder = TrimOutput.Inputs(4);
    JSBSimTrim.U = JSBSimTrim.VelocitiesIni(1);
    JSBSimTrim.V = JSBSimTrim.VelocitiesIni(2);
    JSBSimTrim.W = JSBSimTrim.VelocitiesIni(3);
    JSBSimTrim.P = JSBSimTrim.RatesIni(1);
    JSBSimTrim.Q = JSBSimTrim.RatesIni(2);
    JSBSimTrim.R = JSBSimTrim.RatesIni(3);
%      JSBSimTrim.Height = TrimOutput.States(JSBSimTrim.StateIdx(7));
%      JSBSimTrim.Long = TrimOutput.States(JSBSimTrim.StateIdx(8));
%      JSBSimTrim.Lat = TrimOutput.States(JSBSimTrim.StateIdx(9));
    JSBSimTrim.Phi = JSBSimTrim.AttitudeIni(1);
    JSBSimTrim.Theta = JSBSimTrim.AttitudeIni(2);
    JSBSimTrim.Psi = JSBSimTrim.AttitudeIni(3);
    
    guidata(hObject,handles);
end

function do_linearization(hObject, handles)
    
fprintf('\nJSBSimTrim: Linearization Iniitializing');
fprintf('\nJSBSimTrim: Extracting aircraft linear model...\n');

handles.do_linearization = hObject;
global JSBSimTrim;

DoEigen = handles.eigen_value;

JSBSimTrim.ICStates = [JSBSimTrim.U JSBSimTrim.V JSBSimTrim.W...
    JSBSimTrim.P JSBSimTrim.Q JSBSimTrim.R JSBSimTrim.Height...
     JSBSimTrim.Long JSBSimTrim.Lat JSBSimTrim.Phi JSBSimTrim.Theta...
     JSBSimTrim.Psi];
 
JSBSimTrim.ICInputs = [JSBSimTrim.Throttle, JSBSimTrim.Aileron, JSBSimTrim.Elevator, JSBSimTrim.Rudder, ...
    JSBSimTrim.Mix, JSBSimTrim.SetRun, JSBSimTrim.Flaps, JSBSimTrim.Gear];

JSBSimTrim.Inputs = [JSBSimTrim.Throttle, JSBSimTrim.Aileron, JSBSimTrim.Elevator, JSBSimTrim.Rudder];

% Perturbation level
LinParam(1) = 10^-8;
% Perform the linearization
[A, B, C, D] = linmod(JSBSimTrim.SimModel, JSBSimTrim.ICStates, JSBSimTrim.Inputs, LinParam);

% Print the decoupled state-space matrices to the Matlab command window
% Longitudinal dynamics
% States: u w q h theta
% Inputs: throttle elevator
% Outputs: V alpha h theta
Alon = [
    A(JSBSimTrim.StateIdx(1), [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(11)])
    A(JSBSimTrim.StateIdx(3), [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(11)])
    A(JSBSimTrim.StateIdx(5), [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(11)])
    A(JSBSimTrim.StateIdx(7), [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(11)])
    A(JSBSimTrim.StateIdx(11), [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(11)])
    
];
%Throttle elevator
Blon = [
    B(JSBSimTrim.StateIdx(1), [1 3])
    B(JSBSimTrim.StateIdx(3), [1 3])
    B(JSBSimTrim.StateIdx(5), [1 3])
    B(JSBSimTrim.StateIdx(7), [1 3])
    B(JSBSimTrim.StateIdx(11), [1 3])
];
Clon = [
    C(1, [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(11)])
    C(2, [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(1)])
    zeros(2,5)
    C(4, [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(11)])
    
];
Clon(3,3) = 1; Clon(4,4) = 1;
Dlon = [
    zeros(5,1)
];
fprintf('\n');
fprintf('\nLongitudinal Dynamics');
fprintf('\n-----------------------');
fprintf('\nState vector: x = [u w q h theta]');
fprintf('\nInput vector: u = [throttle elevator]');
fprintf('\nOutput vector: y = [Va alpha h theta]');
fprintf('\nState matrix: A = \n');
disp(Alon);
fprintf('\n Control matrix: B = \n');
disp(Blon);
fprintf('\n Observation matrix: C = \n');
disp(Clon);

if(DoEigen)
% Eigenvalue analysis
eiglon = eig(Alon);
for i=1:length(eiglon)
    if imag(eiglon(i))>0
        [wd, T, wn, zeta] = eigparam(eiglon(i));
        fprintf('\nJSBSimTrim: Eigenvalue: %6.4f +/- %6.4f i', real(eiglon(i)), imag(eiglon(i)));
        fprintf('\nJSBSimTrim: Damping = %6.4f, natural frequency = %6.4f rad/s, period = %8.4f s', zeta, wn, T);
    elseif imag(eiglon(i))==0
        fprintf('\nJSBSimTrim: Eigenvalue: %6.4f', eiglon(i));
        fprintf('\nJSBSimTrim: Time constant = %6.4f s', -1/eiglon(i));
    end
end
end
% Lateral-directional dynamics
% States: v p r phi psi
% Inputs: aileron rudder
% Outputs: beta p r phi psi
Alat = [
    A(JSBSimTrim.StateIdx(2), [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(12)])
    A(JSBSimTrim.StateIdx(4), [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(12)])
    A(JSBSimTrim.StateIdx(6), [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(12)])
    A(JSBSimTrim.StateIdx(10), [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(12)])
    A(JSBSimTrim.StateIdx(12), [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(12)])
];
Blat = [
    B(JSBSimTrim.StateIdx(2), [2 4])
    B(JSBSimTrim.StateIdx(4), [2 4])
    B(JSBSimTrim.StateIdx(6), [2 4])
    B(JSBSimTrim.StateIdx(10), [2 4])
    B(JSBSimTrim.StateIdx(12), [2 4])
];
Clat = [
    C(3, [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(12)])
    zeros(4, 5)
];
Clat(2,2) = 1; Clat(3,3) = 1;
Clat(4,4) = 1; Clat(5,5) = 1;

fprintf('\n');
fprintf('\nLateral-directional Dynamics');
fprintf('\n------------------------------');
fprintf('\n State vector: x = [v p r phi psi]');
fprintf('\n Input vector: u = [aileron rudder]');
fprintf('\n Output vector: y = [beta phi psi]');
fprintf('\n State matrix: A = \n');
disp(Alat);
fprintf('\n Control matrix: B = \n');
disp(Blat);
fprintf('\n Observation matrix: C = \n');
disp(Clat);


% Eigenvalue analysis
if(DoEigen)
eiglat = eig(Alat);
 for i=1:length(eiglat)
    if imag(eiglat(i))>0
        [wd, T, wn, zeta] = eigparam(eiglat(i));
        fprintf('\nJSBSimTrim: Eigenvalue: %6.4f +/- %6.4f i', real(eiglat(i)), imag(eiglat(i)));
        fprintf('\nJSBSimTrim: Damping = %6.4f, natural frequency = %6.4f rad/s, period = %8.4f s', zeta, wn, T);
    elseif (imag(eiglat(i))==0)&&(real(eiglat(i))~=0)
        fprintf('\nJSBSimTrim: Eigenvalue: %6.4f', eiglat(i));
        fprintf('\nJSBSimTrim: Time constant = %6.4f s', -1/eiglat(i));
    end
 end
end
% --- Executes during object creation, after setting all properties.
function FV_trim_enable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FV_trim_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.FV_trim_enable_value = 1;
    handles.FV_trim_enable = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in FV_trim_enable.
function FV_trim_enable_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_linearize_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.fv_trim_enable_value = get(hObject,'Value');
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of Trim_linearize_enable


% --- Executes during object creation, after setting all properties.
function FV_sim_enable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FV_enable_sim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    handles.FV_sim_enable_value = 0;
    handles.FV_sim_enable = hObject;
    guidata(hObject,handles);
    
% --- Executes on button press in FV_sim_enable.
function FV_sim_enable_Callback(hObject, eventdata, handles)
% hObject    handle to Trim_linearize_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    handles.fv_sim_enable_value = get(hObject,'Value');
    guidata(hObject,handles);
% Hint: get(hObject,'Value') returns toggle state of Trim_linearize_enable


