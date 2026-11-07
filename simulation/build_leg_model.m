function build_leg_model
% BUILD_LEG_MODEL Create the standalone dc_leg.slx model in this folder.
% Requires MATLAB, Simulink, Simscape, and Simscape Multibody.
% The DC motor uses circuit equations without FOC, PWM switching, or gears.
% All defaults are illustrative assumptions, not calibrated hardware data.

%% All editable parameters are in leg_parameters.m.
p = leg_parameters;

%% Model parameters are embedded in the SLX; no initialization script is needed.
folder = fileparts(mfilename('fullpath'));
mdl = 'dc_leg';
if bdIsLoaded(mdl), close_system(mdl,0); end
modelFile = fullfile(folder,[mdl '.slx']);
if isfile(modelFile)
    load_system(modelFile);
    Simulink.BlockDiagram.deleteContents(mdl); % Rebuilding replaces this generated model.
else
    new_system(mdl);
end
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
set_param(mdl,'StopTime','p.T','Solver','ode15s','MaxStep','0.001', ...
    'RelTol','1e-6','AbsTol','1e-8','ReturnWorkspaceOutputs','on', ...
    'SaveOutput','off','SaveTime','off','SignalLogging','off');

% Top level: target current -> DC motor -> leg and ground -> recorded signals.
add_block('simulink/Sources/Constant',[mdl '/Target current'], ...
    'Value','p.torque/p.Kt','Position',[40 140 130 180]);
motor = [mdl '/DC motor'];
plant = [mdl '/Leg and ground'];
empty_subsystem(motor,[210 120 380 260]);
empty_subsystem(plant,[480 95 680 305]);
make_motor(motor);
make_mechanics(plant);
add_line(mdl,'Target current/1','DC motor/1','autorouting','on');
add_line(mdl,'DC motor/1','Leg and ground/1','autorouting','on');
add_line(mdl,'Leg and ground/2','DC motor/2','autorouting','on');

add_block('simulink/Signal Routing/Mux',[mdl '/Recorded signals'], ...
    'Inputs','7','Position',[780 80 785 340]);
% Recorded columns: I, q, dq, ddq, z, voltage, electromagnetic torque.
% Contact force is logged separately as F_ground for direct access.
sources = {'DC motor/2','Leg and ground/1','Leg and ground/2', ...
    'Leg and ground/3','Leg and ground/5', ...
    'DC motor/3','DC motor/1'};
for k=1:numel(sources)
    add_line(mdl,sources{k},['Recorded signals/' num2str(k)],'autorouting','on');
end
add_block('simulink/Sinks/To Workspace',[mdl '/Save data'], ...
    'VariableName','sim_data','SaveFormat','Timeseries', ...
    'SampleTime','p.Ts','MaxDataPoints','inf','Position',[875 160 1000 210]);
add_line(mdl,'Recorded signals/1','Save data/1');
add_block('simulink/Sinks/To Workspace',[mdl '/Save ground force'], ...
    'VariableName','F_ground','SaveFormat','Timeseries', ...
    'SampleTime','p.Ts','MaxDataPoints','inf','Position',[875 275 1000 325]);
add_line(mdl,'Leg and ground/4','Save ground force/1','autorouting','on');
add_block('simulink/Sinks/Scope',[mdl '/Ground force N'], ...
    'Position',[735 390 845 440]);
add_line(mdl,'Leg and ground/4','Ground force N/1','autorouting','on');
note(mdl,[40 25 980 70], ...
    '10 kg moving mass | DC current control | 60 deg initial | 8 Nm target | 3 s');
note(mdl,[40 470 1000 535],sprintf([ ...
    'Run simulation/run_simulation.m to simulate and export raw signals.\n' ...
    'Read contact force directly: out.F_ground.Data (N).\n' ...
    'Demo assumptions: known joint damping, frictionless sliding foot, ideal motion sensing.']));
set_param(mdl,'ZoomFactor','100');
save_system(mdl,modelFile);
open_system(mdl);
fprintf('Built %s\nNext: run run_simulation.m in this folder.\n',modelFile);
end

function make_motor(s)
% Circuit: Le*dI/dt = V-R*I-Ke*w; electromagnetic torque = Kt*I.
% Current PI with back-EMF feedforward and a limited drive voltage.
% Current is an integrated electrical state, not inferred from commanded torque.
add_block('simulink/Ports & Subsystems/In1',[s '/I target'],'Port','1','Position',[25 45 55 65]);
add_block('simulink/Ports & Subsystems/In1',[s '/Speed'],'Port','2','Position',[25 235 55 255]);
add_block('simulink/Math Operations/Sum',[s '/Current error'],'Inputs','+-','Position',[110 40 130 80]);
add_block('simulink/Math Operations/Gain',[s '/P'],'Gain','p.Kp','Position',[180 30 230 65]);
add_block('simulink/Math Operations/Gain',[s '/I gain'],'Gain','p.Ki','Position',[180 110 230 145]);
add_block('simulink/Continuous/Integrator',[s '/PI integral'], ...
    'InitialCondition','p.R*p.I0','LimitOutput','on', ...
    'UpperSaturationLimit','p.Vmax','LowerSaturationLimit','-p.Vmax','Position',[270 110 305 145]);
add_block('simulink/Math Operations/Gain',[s '/Back EMF'],'Gain','p.Ke','Position',[180 225 230 260]);
add_block('simulink/Math Operations/Sum',[s '/Voltage request'],'Inputs','+++','Position',[355 40 375 110]);
add_block('simulink/Discontinuities/Saturation',[s '/Voltage limit'], ...
    'UpperLimit','p.Vmax','LowerLimit','-p.Vmax','Position',[415 45 470 90]);
add_block('simulink/Math Operations/Sum',[s '/V minus RI minus EMF'],'Inputs','+--','Position',[520 40 540 110]);
add_block('simulink/Math Operations/Gain',[s '/One over L'],'Gain','1/p.Le','Position',[585 50 640 90]);
add_block('simulink/Continuous/Integrator',[s '/Armature current'], ...
    'InitialCondition','p.I0','Position',[685 50 725 90]);
add_block('simulink/Math Operations/Gain',[s '/Resistance'],'Gain','p.R','Position',[585 170 640 205]);
add_block('simulink/Math Operations/Gain',[s '/Torque constant'],'Gain','p.Kt','Position',[785 40 845 80]);
add_block('simulink/Ports & Subsystems/Out1',[s '/Torque'],'Port','1','Position',[900 50 930 70]);
add_block('simulink/Ports & Subsystems/Out1',[s '/Current'],'Port','2','Position',[900 130 930 150]);
add_block('simulink/Ports & Subsystems/Out1',[s '/Voltage'],'Port','3','Position',[900 285 930 305]);
edges={'I target/1','Current error/1'; 'Armature current/1','Current error/2'; ...
    'Current error/1','P/1'; 'Current error/1','I gain/1'; ...
    'I gain/1','PI integral/1'; 'P/1','Voltage request/1'; ...
    'PI integral/1','Voltage request/2'; 'Speed/1','Back EMF/1'; ...
    'Back EMF/1','Voltage request/3'; 'Voltage request/1','Voltage limit/1'; ...
    'Voltage limit/1','V minus RI minus EMF/1'; ...
    'Resistance/1','V minus RI minus EMF/2'; ...
    'Back EMF/1','V minus RI minus EMF/3'; ...
    'V minus RI minus EMF/1','One over L/1'; ...
    'One over L/1','Armature current/1'; 'Armature current/1','Resistance/1'; ...
    'Armature current/1','Torque constant/1'; 'Torque constant/1','Torque/1'; ...
    'Armature current/1','Current/1'; 'Voltage limit/1','Voltage/1'};
for k=1:size(edges,1), add_line(s,edges{k,1},edges{k,2},'autorouting','on'); end
note(s,[30 350 910 415],sprintf('Current PI -> limited average voltage -> R-L-back-EMF circuit\nLe*dI/dt = V - R*I - Ke*speed; electromagnetic torque = Kt*I'));
end

function make_mechanics(s)
% World Z points up; the joint axis is world Y.
% q increases downward from horizontal; the leg is vertical at q=90 deg.
add_block('simulink/Ports & Subsystems/In1',[s '/Torque'],'Position',[20 100 50 120]);
blk('nesl_utility','Simulink-PS Converter',s,'Torque input',[80 90 125 130],'Unit','N*m');
add_line(s,'Torque/1','Torque input/1');
blk('sm_lib','World Frame',s,'World',[40 330 85 370]);
blk('nesl_utility','Solver Configuration',s,'Solver',[25 440 120 485]);
blk('sm_lib','Mechanism Configuration',s,'Gravity',[175 430 285 485],'GravityVector','[0 0 -p.g]');
blk('sm_lib','Prismatic Joint',s,'Vertical guide',[190 300 260 380], ...
    'PositionTargetSpecify','on','PositionTargetValue','p.z0', ...
    'VelocityTargetSpecify','on','VelocityTargetValue','0','SensePosition','on');
blk('sm_lib','Brick Solid',s,'Carriage',[360 400 445 460], ...
    'BrickDimensions','[0.10 0.08 0.06]','BasedOnType','Mass','Mass','p.M-p.m', ...
    'GraphicDiffuseColor','[0.2 0.4 0.8]');
blk('sm_lib','Rigid Transform',s,'Axis orientation',[340 290 425 350], ...
    'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90');
blk('sm_lib','Revolute Joint',s,'Leg joint',[485 245 555 355], ...
    'PositionTargetSpecify','on','PositionTargetValue','p.q0','PositionTargetValueUnits','rad', ...
    'VelocityTargetSpecify','on','VelocityTargetValue','0', ...
    'TorqueActuationMode','InputTorque','SensePosition','on','SenseVelocity','on','SenseAcceleration','on', ...
    'DampingCoefficient','p.b','DampingCoefficientUnits','N*m/(rad/s)');
blk('sm_lib','Rigid Transform',s,'Leg centre',[625 370 700 420], ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset','[p.lc 0 0]');
blk('sm_lib','Brick Solid',s,'Leg mass',[760 370 845 430], ...
    'BrickDimensions','[p.L p.width p.width]','BasedOnType','Mass','Mass','p.m', ...
    'GraphicDiffuseColor','[0.9 0.5 0.1]');
blk('sm_lib','Rigid Transform',s,'Foot position',[630 480 705 530], ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset','[p.L 0 0]');
blk('sm_lib','Point',s,'Foot',[765 480 830 530],'MarkerSize','4');
blk('sm_lib','Infinite Plane',s,'Ground',[185 585 265 640], ...
    'GraphicPlaneWidth','0.6','GraphicPlaneHeight','0.4','GraphicDiffuseColor','[0.5 0.65 0.5]');
blk('sm_lib','Spatial Contact Force',s,'Ground contact',[915 510 1010 585], ...
    'NormalStiffness','p.kGround','NormalDamping','p.dGround', ...
    'NormalTransitionRegionWidth','p.transition','FrictionType','None', ...
    'SenseNormalForceMagnitude','on');

% B/F mean base/follower frames; the contact block connects ground/foot geometry.
wire(s,'World','RConn',1,'Solver','RConn',1);
wire(s,'World','RConn',1,'Gravity','RConn',1);
wire(s,'World','RConn',1,'Vertical guide','LConn',1);
wire(s,'World','RConn',1,'Ground','LConn',1);
wire(s,'Vertical guide','RConn',1,'Carriage','RConn',1);
wire(s,'Vertical guide','RConn',1,'Axis orientation','LConn',1);
wire(s,'Axis orientation','RConn',1,'Leg joint','LConn',1);
wire(s,'Torque input','RConn',1,'Leg joint','LConn',2);
wire(s,'Leg joint','RConn',1,'Leg centre','LConn',1);
wire(s,'Leg centre','RConn',1,'Leg mass','RConn',1);
wire(s,'Leg joint','RConn',1,'Foot position','LConn',1);
wire(s,'Foot position','RConn',1,'Foot','LConn',1);
wire(s,'Ground','RConn',1,'Ground contact','LConn',1);
wire(s,'Foot','RConn',1,'Ground contact','RConn',1);

signals={'Angle','Speed','Acceleration','Reference force','Height'};
units={'rad','rad/s','rad/s^2','N','m'};
origin={'Leg joint','Leg joint','Leg joint','Ground contact','Vertical guide'};
index=[2 3 4 2 2];
for k=1:5
    y=30+85*(k-1);
    blk('nesl_utility','PS-Simulink Converter',s,[signals{k} ' converter'], ...
        [1110 y 1155 y+35],'Unit',units{k});
    wire(s,origin{k},'RConn',index(k),[signals{k} ' converter'],'LConn',1);
    add_block('simulink/Ports & Subsystems/Out1',[s '/' signals{k}], ...
        'Port',num2str(k),'Position',[1220 y+5 1250 y+25]);
    add_line(s,[signals{k} ' converter/1'],[signals{k} '/1']);
end
note(s,[360 650 1190 705],sprintf([ ...
    'One vertical carriage + one rotating rigid leg; foot slides without friction.\n' ...
    'Ground force is fnm from Ground contact, using the built-in contact law.']));
end

%% Helpers for placing and connecting blocks; no additional physical equations
function empty_subsystem(path,pos)
add_block('built-in/Subsystem',path,'Position',pos);
end
function blk(lib,libraryName,parent,name,pos,varargin)
% Library block names may contain newlines; match their normalized display names.
persistent cache
if isempty(cache), cache=struct; end
if ~isfield(cache,lib)
    load_system(lib);
    paths=find_system(lib,'LookUnderMasks','all','FollowLinks','off','Type','Block');
    cache.(lib).paths=paths;
    cache.(lib).names=regexprep(get_param(paths,'Name'),'\s+',' ');
end
entry=cache.(lib); ix=find(strcmp(entry.names,libraryName),1);
assert(~isempty(ix),'Cannot find library block: %s',libraryName);
add_block(entry.paths{ix},[parent '/' name],'Position',pos,varargin{:});
end
function wire(parent,a,sideA,ia,b,sideB,ib)
pa=get_param([parent '/' a],'PortHandles');
pb=get_param([parent '/' b],'PortHandles');
add_line(parent,pa.(sideA)(ia),pb.(sideB)(ib),'autorouting','on');
end
function note(parent,pos,words)
n=Simulink.Annotation(parent,words); n.Position=pos; n.FontSize=11;
end
