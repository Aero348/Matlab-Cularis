% A Matlab script that trims the nonlinear JSBSim aircraft model for a 
% chosen flight condition and extracts the aircraft linear model.
% Uses the JSBSim_Sfunction_Trim MEX file.
% This script is based heavily on the trim and linearization script used in the
% Aerosim Blockset by U-Dynamics by Marius Niculescu
%
% Brian Mills
% May, 2010
%
% The trim parameters structure has the following components:
% 1. Simulation settings
% JSBSimTrim.SampleTime = simulation sample time
% JSBSimTrim.FinalTime = simulation final time (used only for trim function)
% JSBSimTrim.SimModel = Simulink model name
% 2. Flight condition definition
% JSBSimTrim.VelocitiesIni
% JSBSimTrim.RatesIni
% JSBSimTrim.AttitudeIni
% JSBSimTrim.PositionIni
% JSBSimTrim.FuelIni
% JSBSimTrim.EngineSpeedIni
% JSBSimTrim.Airspeed
% JSBSimTrim.Altitude
% JSBSimTrim.Phi
% JSBSimTrim.Elevator
% JSBSimTrim.Aileron
% JSBSimTrim.Rudder
% JSBSimTrim.Throttle
% JSBSimTrim.Flap
% JSBSimTrim.Mix
% JSBSimTrim.Ign
% JSBSimTrim.Winds
% 3. Miscellaneous parameters:
% JSBSimTrim.StateIdx = state order index (order of states in Simulink block diagram usually
%                       different than the desired state order - velocities, angular rates,
%                       attitude angles, position, fuel, and engine speed)
% JSBSimTrim.Options = trim function options
% JSBSimTrim.SimOptions = the sim function options
% JSBSimTrim.NAircraftStates = the size of the aircraft state vector,
%                               currently 12
% JSBSimTrim.NSimulinkStates = the size of the state vector in the Simulink
%                               model (could be larger than aircraft state vector)
%
% The trim output structure has the following components:
% TrimOutput.States = Simulink model states at trim condition
% TrimOutput.Inputs = inputs at trim condition
% TrimOutput.Outputs = outputs at trim condition
% TrimOutput.Derivatives = model derivatives at trim condition

clear all;
clc;

fprintf('\nJSBSimTrim: Initializing trim....');
fprintf('\nJSBSimTrim: Setting initial trim parameters...');

% Simulation time settings
JSBSimTrim.SampleTime = 0.00833;
JSBSimTrim.FinalTime = 60;

%% Initial guess for control inputs
% TrimInput = [throttle left_aileron elevator rudder]
% These initial control input guesses are very important to finding a good
% trimmed state for the aircraft model
JSBSimTrim.Mix = 0.7;
JSBSimTrim.SetRun = 1;
% JSBSimTrim.Flaps = 0;
% JSBSimTrim.Gear = 0;
TrimInput = [0.47 0.0225 0.04 -0.0114]; % C172r 150 fps, 5500ft level cruise
%TrimInput = [0.4139 0.0225 0.0365 -0.0114]; % C172r 150 fps, 5500ft level cruise
%TrimInput = [0.595 0 -0.0713 0]; %Trim input guess for f16 925 fps, 30000 ft level cruise
%TrimInput = [0.555 0 -0.0461 0]; %Trim input guess for f16 825 fps, 20000 ft level cruise
%TrimInput = [0.575 0 -0.055 0];
% JSBSimTrim.Throttle = TrimInput(1);
% JSBSimTrim.Aileron = TrimInput(2);
% JSBSimTrim.Elevator = TrimInput(3);
% JSBSimTrim.Rudder = TrimInput(4);
 
%% State Initialization 
JSBSimTrim.U = 0; % 1
JSBSimTrim.V = 0; % 2
JSBSimTrim.W = 0; % 3
JSBSimTrim.P = 0; % 4
JSBSimTrim.Q = 0; % 5
JSBSimTrim.R = 0; % 6
JSBSimTrim.Height = 0; %7
JSBSimTrim.Long = 45*pi/180; % 8
JSBSimTrim.Lat = -122*pi/180; % 9
JSBSimTrim.Phi = 0; % 10
JSBSimTrim.Theta = 0; % 11
JSBSimTrim.Psi = 0; % 12

%% Wind velocities
% JSBSimTrim.Winds = [0 0 0];

%% ************************************************************************
% Prompt the user for the aircraft model to trim
% ************************************************************************

newline = sprintf('\n');
disp('To test the trim function use the values that are in parenthesis at each prompt')
disp('Which JSBSim aircraft model would you like to trim:(f16) ')
JSBSimTrim.Aircraft = input('Your Selection:  ');
fprintf('\nJSBSimTrim: The JSBSim aircraft model %s.mdl will be trimmed.', JSBSimTrim.Aircraft);
newline = sprintf('\n');

JSBSimTrim.Height = input('\nEnter the altitude for the simulation (ft): (30000) ');
JSBSimTrim.U = input('\nEnter the velocity for the simulation (fps): (925) ');
bankangle = 0;

% Let's concentrate on wings level flight first :) 
% bankangle = input('\nEnter the bank angle for the simulation (deg):  ');

% Lets calculate the pitch angle and associated body W velocity based on an estimate of the alpha
% needed for straight and level flight with a flight path angle of zero
alpha = input('Enter the estimated angle-of-attack in deg for straight and level flight (deg): (0.61) ');
JSBSimTrim.Theta = alpha/(180/pi); % alpha deg to radians
JSBSimTrim.W = (atan(JSBSimTrim.Theta))*JSBSimTrim.U;
JSBSimTrim.Phi = bankangle/(180/pi);
% fpathangle = input('Enter the flight path angle for the simulation (deg):  ');
% JSBSimTrim.FPA = fpathangle/(180/pi);
% ADD compute theta based on flight path angle
JSBSimTrim.Flaps = input('\nFlap setting norm[0 - 1]: (0) ');
JSBSimTrim.Gear = input('\nGear setting norm.  Gear up is 0, Gear down is 1 [0 - 1]: (0) ');


%% #########################################################################
% #########################################################################
% SET THE BLOCK PARAMETERS FOR THE S-FUNCTION 
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

 % Set the verbosity level for JSBSim output. this can be set to 'silent',
 % 'verbose', 'very verbose' or 'debug'
JSBSimTrim.Verbosity = 'verbose';

% Select the Simulink model that will be used for trimming
JSBSimTrim.SimModel = 'jsbsim_trim_mod';
% Get the sim options structure
JSBSimTrim.SimOptions = simget(JSBSimTrim.SimModel);

% Set the state vector ID
JSBSimTrim.NAircraftStates = length(12);
JSBSimTrim.NSimulinkStates = 12;
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
% The pre-trim algorithm is set to run for a certain period of time
% (currently 15 sec) and will run iteratively until the conditions set for
% maximum TAS error, Alt error and Bank error are met. If they are not met,
% the pre-trim algorithm will run until the maximum number of iterations
% are completed.
% After the pre-trim algorithm exists, the resulting control inputs and
% states will be saved and used for the Matlab trim function

% The trim error threshold for the pre-trim routine
% These values are maximum allowed errors for the 
% May need to be changed by user!
MaxErrTAS = 0.1;
MaxErrAlt = 0.5;
MaxErrBank = 0.25*pi/180;

% The control surface gains
% These are simply proportional feedback gains that will be used during the
% pre-trim stabilization runs

% Gains for low speed C172
KElevator = 0.001;
KAileron = -0.02;
KThrottle = -0.002;
KRudder = 0.02;

% Gains for high speed flight ie F16 etc
% KElevator = 0.001;
% KAileron = -0.01;
% KThrottle = -0.001;
% KRudder = 0.01;

% No gains
% KElevator = 0;
% KAileron = 0;
% KThrottle = 0;
% KRudder = 0;

fprintf('\nJSBSimTrim: Computing the initial estimates for the trim inputs...');

GoodGuess = 0; Niter = 1;
while (~GoodGuess)&(Niter<35)% May need to be changed by user! This determines number of times that the pretrim loop will be performed
    % Run Simulink model for a short time (15 s)
   [SimTime, SimStates, SimOutputs] = sim(JSBSimTrim.SimModel, [0 10], JSBSimTrim.SimOptions, ...
       [0 TrimInput; 10 TrimInput]);
    % Compute errors in trim
    ErrTAS = SimOutputs(end,1) - JSBSimTrim.U;
    ErrAlt = SimOutputs(end,4) - JSBSimTrim.Height;
    ErrBank = SimOutputs(end,5) - JSBSimTrim.Phi;
    ErrYaw = SimOutputs(end,7) - JSBSimTrim.Psi;
    fprintf('\nJSBSimTrim: Iteration #%2d, Airsp err = %6.2f fps, Alt err = %8.2f ft, phi err = %6.2f deg, psi err = %6.2f deg.', Niter, ErrTAS, ErrAlt, ErrBank*180/pi, ErrYaw*180/pi);  
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
        if (TrimInput(1)>1)
            TrimInput(1) = 1;
        elseif (TrimInput(1)<0)
            TrimInput(1) = 0;
        end
        
    end 
    JSBSimTrim.ICInputs = [TrimInput(1), TrimInput(2), TrimInput(3), TrimInput(4), ...
    JSBSimTrim.Mix, JSBSimTrim.SetRun, JSBSimTrim.Flaps, JSBSimTrim.Gear];
    Niter = Niter + 1;
end

%% Save initial guess
fprintf('\nJSBSimTrim: Saving initial velocities, attitude and inputs');
JSBSimTrim.VelocitiesIni = SimStates(end,JSBSimTrim.StateIdx(1:3))'; % Save the velocities from the pre-trim run
JSBSimTrim.AttitudeIni = SimStates(end, JSBSimTrim.StateIdx(10:12))';% Save the attitude from the pre-trim run
% Save the control inputs
JSBSimTrim.Throttle = TrimInput(1);
JSBSimTrim.Aileron = TrimInput(2);
JSBSimTrim.Elevator = TrimInput(3);
JSBSimTrim.Rudder = TrimInput(4);
clear SimTime SimStates SimOutputs TrimInput;
% Print our initial control input guesses
fprintf('\nJSBSimTrim: Initial guesses for trim inputs are:');
fprintf('\nJSBSimTrim:      Throttle = %6.4f', JSBSimTrim.Throttle);
fprintf('\nJSBSimTrim:      Aileron = %6.4f', JSBSimTrim.Aileron);
fprintf('\nJSBSimTrim:      Elevator = %6.4f', JSBSimTrim.Elevator);
fprintf('\nJSBSimTrim:      Rudder = %6.4f', JSBSimTrim.Rudder);

fprintf('\nJSBSimTrim: Initial guesses for trimmed states are:');
fprintf('\nJSBSimTrim:    u = %6.2f ft/s', JSBSimTrim.VelocitiesIni(1));
fprintf('\nJSBSimTrim:    v = %6.2f ft/s', JSBSimTrim.VelocitiesIni(2));
fprintf('\nJSBSimTrim:    w = %6.2f ft/s', JSBSimTrim.VelocitiesIni(3));
fprintf('\nJSBSimTrim:    p = %6.2f deg/s', JSBSimTrim.RatesIni(1)*180/pi);
fprintf('\nJSBSimTrim:    q = %6.2f deg/s', JSBSimTrim.RatesIni(2)*180/pi);
fprintf('\nJSBSimTrim:    r = %6.2f deg/s', JSBSimTrim.RatesIni(3)*180/pi);
fprintf('\nJSBSimTrim:    alt = %6.2f ft', JSBSimTrim.Height);
fprintf('\nJSBSimTrim:    phi = %6.2f deg, %6.2d rads', JSBSimTrim.AttitudeIni(1)*180/pi, JSBSimTrim.AttitudeIni(1));
fprintf('\nJSBSimTrim:    theta = %6.2f deg, %6.2d rads', JSBSimTrim.AttitudeIni(2)*180/pi, JSBSimTrim.AttitudeIni(2));
fprintf('\nJSBSimTrim:    psi = %6.2f deg, %6.2d rads', JSBSimTrim.AttitudeIni(3)*180/pi, JSBSimTrim.AttitudeIni(3));

  
%% %%% PERFORM AIRCRAFT TRIM %%%
fprintf('\n');
fprintf('\nJSBSimTrim: Performing the aircraft trim...\n');

%  Set initial states to mirror the ICStates
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
 
% % Set indices of fixed parameters
StateFixIdx = [JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(6) JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(10)];

InputFixIdx = [];
OutputFixIdx = [1 3 4 5];
DerivFixIdx = [JSBSimTrim.StateIdx(1) JSBSimTrim.StateIdx(2) JSBSimTrim.StateIdx(3) JSBSimTrim.StateIdx(4) JSBSimTrim.StateIdx(5) JSBSimTrim.StateIdx(6) ...
     JSBSimTrim.StateIdx(7) JSBSimTrim.StateIdx(10) JSBSimTrim.StateIdx(11) JSBSimTrim.StateIdx(12)];
 
 % Set optimization parameters
 JSBSimTrim.Options(1)  = 1;     % show some output
 JSBSimTrim.Options(2)  = 1e-6;  % tolerance in X
 JSBSimTrim.Options(3)  = 1e-6;  % tolerance in F
 JSBSimTrim.Options(4)  = 1e-6;
 if(~GoodGuess)
    JSBSimTrim.Options(14) = 100;  % max iterations
 else
    JSBSimTrim.Options(14) = 5000;  % max iterations
 end
%JSBSimTrim.Options(14) = 5000;  % max iterations

 % Trim the airplane
[TrimOutput.States,TrimOutput.Inputs,TrimOutput.Outputs,TrimOutput.Derivatives] = trim(JSBSimTrim.SimModel,StateIni,InputIni,OutputIni,...
      StateFixIdx,InputFixIdx,OutputFixIdx,DerivIni,DerivFixIdx,JSBSimTrim.Options);

% Print the trim results
fprintf('\nJSBSimTrim: Finished. The trim results are:');
fprintf('\nJSBSimTrim: INPUTS:');
fprintf('\nJSBSimTrim:    Throttle = %6.4f', TrimOutput.Inputs(1));
fprintf('\nJSBSimTrim:    Aileron = %6.4f', TrimOutput.Inputs(2));
fprintf('\nJSBSimTrim:    Elevator = %6.4f', TrimOutput.Inputs(3));
fprintf('\nJSBSimTrim:    Rudder = %6.4f', TrimOutput.Inputs(4));

fprintf('\nJSBSimTrim: STATES:');
fprintf('\nJSBSimTrim:    u = %6.2f ft/s', TrimOutput.States(JSBSimTrim.StateIdx(1)));
fprintf('\nJSBSimTrim:    v = %6.2f ft/s', TrimOutput.States(JSBSimTrim.StateIdx(2)));
fprintf('\nJSBSimTrim:    w = %6.2f ft/s', TrimOutput.States(JSBSimTrim.StateIdx(3)));
fprintf('\nJSBSimTrim:    p = %6.2f deg/s', TrimOutput.States(JSBSimTrim.StateIdx(4))*180/pi);
fprintf('\nJSBSimTrim:    q = %6.2f deg/s', TrimOutput.States(JSBSimTrim.StateIdx(5))*180/pi);
fprintf('\nJSBSimTrim:    r = %6.2f deg/s', TrimOutput.States(JSBSimTrim.StateIdx(6))*180/pi);
fprintf('\nJSBSimTrim:    alt = %6.2f ft', TrimOutput.States(JSBSimTrim.StateIdx(7)));
fprintf('\nJSBSimTrim:    phi = %6.2f deg, %6.2d rads', TrimOutput.States(JSBSimTrim.StateIdx(10))*180/pi, TrimOutput.States(JSBSimTrim.StateIdx(10)));
fprintf('\nJSBSimTrim:    theta = %6.2f deg, %6.2d rads', TrimOutput.States(JSBSimTrim.StateIdx(11))*180/pi, TrimOutput.States(JSBSimTrim.StateIdx(11)));
fprintf('\nJSBSimTrim:    psi = %6.2f deg, %6.2d rads', TrimOutput.States(JSBSimTrim.StateIdx(12))*180/pi, TrimOutput.States(JSBSimTrim.StateIdx(12)));

fprintf('\nJSBSimTrim: OUTPUTS:');
fprintf('\nJSBSimTrim:    Airspeed = %6.2d fps', TrimOutput.Outputs(1));
fprintf('\nJSBSimTrim:    AoA = %6.2d deg', TrimOutput.Outputs(2)*180/pi);
fprintf('\nJSBSimTrim:    Sideslip = %6.2d deg', TrimOutput.Outputs(3)*180/pi);
fprintf('\nJSBSimTrim:    Altitude = %6.2d ft', TrimOutput.Outputs(4));
fprintf('\nJSBSimTrim:    Bank = %6.2f deg', TrimOutput.Outputs(5)*180/pi);
fprintf('\nJSBSimTrim:    Pitch = %6.2f deg', TrimOutput.Outputs(6)*180/pi);
fprintf('\nJSBSimTrim:    Heading = %8.2f deg', TrimOutput.Outputs(7)*180/pi);

fprintf('\nJSBSimTrim: Trim Finished.');

%% LINEARIZATION
% Use the inputs and states found in the trim function and extract the linear model
if(~GoodGuess)
    fprintf('\n\n');
    fprintf('\nJSBSimTrim: Linearization aborted- trimmed states could not be found');
    clearSF;
else
    fprintf('\nJSBSimTrim: Linearization Iniitializing');
    fprintf('\nJSBSimTrim: Extracting aircraft linear model...\n');

    JSBSimTrim.ICStates = [TrimOutput.States(JSBSimTrim.StateIdx(1)) TrimOutput.States(JSBSimTrim.StateIdx(2)) TrimOutput.States(JSBSimTrim.StateIdx(3))...
        TrimOutput.States(JSBSimTrim.StateIdx(4)) TrimOutput.States(JSBSimTrim.StateIdx(5)) TrimOutput.States(JSBSimTrim.StateIdx(6)) TrimOutput.States(JSBSimTrim.StateIdx(7))...
         JSBSimTrim.PositionIni(2) JSBSimTrim.PositionIni(3) TrimOutput.States(JSBSimTrim.StateIdx(10)) TrimOutput.States(JSBSimTrim.StateIdx(11))...
         TrimOutput.States(JSBSimTrim.StateIdx(12))];

    JSBSimTrim.ICInputs = [TrimOutput.Inputs(1), TrimOutput.Inputs(2), TrimOutput.Inputs(3), TrimOutput.Inputs(4), ...
        JSBSimTrim.Mix, JSBSimTrim.SetRun, JSBSimTrim.Flaps, JSBSimTrim.Gear];

    % Perturbation level
    LinParam(1) = 10^-8;
    % Perform the linearization
    [A, B, C, D] = linmod(JSBSimTrim.SimModel, TrimOutput.States, TrimOutput.Inputs, LinParam);

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
    fprintf('\n Longitudinal Dynamics');
    fprintf('\n-----------------------');
    fprintf('\n  State vector: x = [u w q h theta]');
    fprintf('\n  Input vector: u = [throttle elevator]');
    fprintf('\n Output vector: y = [Va alpha h theta]');
    fprintf('\n State matrix: A = \n');
    disp(Alon);
    fprintf('\n Control matrix: B = \n');
    disp(Blon);
    fprintf('\n Observation matrix: C = \n');
    disp(Clon);

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
    fprintf('\n Lateral-directional Dynamics');
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

    % Perform the eigenvalue analysis 
    % Eigenvalue analysis
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

%% Print the trim results
fprintf('\n\n');
clearSF;
% 
% JSBSimTrim.Throttle = TrimOutput.Inputs(1);
% JSBSimTrim.Aileron = TrimOutput.Inputs(2);
% JSBSimTrim.Elevator = TrimOutput.Inputs(3);
% JSBSimTrim.Rudder = TrimOutput.Inputs(4);;
% % fprintf('\nSTATES:');
% JSBSimTrim.U = TrimOutput.States(JSBSimTrim.StateIdx(1));
% JSBSimTrim.V = TrimOutput.States(JSBSimTrim.StateIdx(2));
% JSBSimTrim.W = TrimOutput.States(JSBSimTrim.StateIdx(3));
% JSBSimTrim.P = TrimOutput.States(JSBSimTrim.StateIdx(4));
% JSBSimTrim.Q = TrimOutput.States(JSBSimTrim.StateIdx(5));
% JSBSimTrim.R = TrimOutput.States(JSBSimTrim.StateIdx(6));
% JSBSimTrim.Height = TrimOutput.States(JSBSimTrim.StateIdx(7));
% JSBSimTrim.Long = TrimOutput.States(JSBSimTrim.StateIdx(8));
% JSBSimTrim.Lat = TrimOutput.States(JSBSimTrim.StateIdx(9));
% JSBSimTrim.Phi = TrimOutput.States(JSBSimTrim.StateIdx(10));
% JSBSimTrim.Theta = TrimOutput.States(JSBSimTrim.StateIdx(11));
% JSBSimTrim.Psi = TrimOutput.States(JSBSimTrim.StateIdx(12));
% JSBSimGUI;